#include "hal/adc_types.h"
#include "sdkconfig.h"

#include "esp_log.h"
#include <cstdint>
#include <functional>
#include <stdio.h>
#include <driver/timer.h>
#include <variant>
#include <esp_adc/adc_oneshot.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "BLEBackend.h"
#include "Packets.h"
#include "TelometerImpl.h"
#include "Telometer.h"
#include "ESP_CRSF.h"

#include "HD107S.h"
#include "H3LIS331DL.h"
#include "MMC5983MA.h"
#include "MathUtils.h"
#include "orientator.h"
#include "donutPhysics.h"
#include "DShot_ESC.h"
#include "soc/gpio_num.h"

#define LED_COUNT 20U
#define LED_DATA_PIN 11
#define LED_CLOCK_PIN 12
#define LED_CLOCK_SPEED 40000000 // Damn!!! 

#define ESC_L_GPIO GPIO_NUM_38
#define ESC_R_GPIO GPIO_NUM_35

HD107S LED;

H3LIS331DL accel1;
H3LIS331DL accel2;

MMC5983MA mag;

adc_oneshot_unit_handle_t adc2_handle;

struct TelemetryPackets packets = initTelemetryPackets();
Telometer::Backend *myBackend;


Telometer::TelometerInstance telemetry;
extern "C" {

float getBattVoltage() {
    int voltageReading = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, ADC_CHANNEL_3, &voltageReading));
    return (float)voltageReading * 19 / 4095; // 1.1V ref, "6dB" attenuation (actually 4.75dB), 1:10 voltage divider
}
    
void app_main(void) {
    vec3<int32_t> vec_m = {0,0,0};
    vec3<int32_t> vec_m_min = {0,0,0};
    vec3<int32_t> vec_m_max = {0,0,0};
    vec3<int16_t> accel_val_1, accel_val_2, accel_off_1 = {0,0,0}, accel_off_2 = {0,0,0};
    vec3<float> accel_avg_1, accel_avg_2;
    vec2<float> CoR;
    DonutPhysics physics;
    system_state_t state;

    myBackend = (Telometer::Backend*) new Telometer::BLEBackend();
    telemetry = {
        .backend  = myBackend,
        .count = TelemetryPacketCount,
        .nextPacket = 0,
        .packetStruct = (Telometer::Data*)&packets
    };
    Telometer::init(&telemetry);
    uint8_t telo_delay = 0;
    bool enabled = 0;
    float time = 0;
    float deltaTime = 0.001;
    cal_en cal = {
        .XL_l_z_up = 0,
        .XL_r_z_up = 0,
    };
    packets.calibration.pointer = &cal;
    packets.calibration.queued = true;
    packets.enabled.pointer = &enabled;
    packets.time.pointer = &time;
    packets.deltaTime.pointer = &deltaTime;
    packets.accel1.pointer = &accel_val_1;
    packets.accel2.pointer = &accel_val_2;
    packets.accel1Avg.pointer = &accel_avg_1;
    packets.accel2Avg.pointer = &accel_avg_2;
    packets.centerOfRotation.pointer = &CoR;
    packets.mag.pointer = &vec_m;
    packets.state.pointer = &state;

    hd107s_config_t LED_config;
    LED_config.dataPin = LED_DATA_PIN;
    LED_config.clockPin = LED_CLOCK_PIN;
    LED_config.numLEDs = LED_COUNT;
    LED_config.clockSpeed = LED_CLOCK_SPEED;
    LED.setup(LED_config);

    h3lis331dl_config_t accel_config;
    accel_config.clock_pin = 5;
    accel_config.miso_pin = 3;
    accel_config.mosi_pin = 4;
    accel_config.cs_pin = 37;
    accel1.setup(accel_config);
    accel_config.cs_pin = 2;
    accel2.setup(accel_config);

    mmc5983ma_config_t mag_config;
    mag_config.clock_pin = 5;
    mag_config.miso_pin = 3;
    mag_config.mosi_pin = 4;
    mag_config.cs_pin = 33;
    mag.setup(mag_config);
    
    adc_oneshot_unit_init_cfg_t adc2_config = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc2_config, &adc2_handle));
    adc_oneshot_chan_cfg_t adc2_channel_config = {
        .atten = ADC_ATTEN_DB_6,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_3, &adc2_channel_config));

    dshot_frequency_t ESC_freq = DSHOT300;
    bool bidirectional = true;
    bool TXS_Buffer = true;

    ESCData ESC_L_data;
    ESCData ESC_R_data;
    DShot_ESC L_ESC(ESC_freq, bidirectional, ESC_L_GPIO, TXS_Buffer, ESC_L_data);
    DShot_ESC R_ESC(ESC_freq, bidirectional, ESC_R_GPIO, TXS_Buffer, ESC_R_data);
    vTaskDelay(2000/portTICK_PERIOD_MS);

   
    state = {
        .angle = 0,
        .angular_velocity = 0,
        .angular_acceleration = 0,
        .upright = true,
        .motor_torque = 0,
        .motor_percentage = 48,
        .wheel_velocity = 0,
        .battery_voltage = 17,
        .time = (uint64_t)esp_timer_get_time(),
    };
    packets.state.queued = true;
    
    // mag.getXYZ(x_m_zero, y_m_zero, z_m_zero);
    ESP_LOGI(TAG, "Completed Init");
    int led_step = 0;

    while(true) {
        state.battery_voltage = getBattVoltage();
        physics.step(state, deltaTime);
        packets.state.queued = true;
        mag.getXYZ(vec_m);
        for (int i = 0; i < LED_COUNT; i++) {
        if (enabled) {
            LED.setLED(i, RGBL(0,25,0,255));
            // if (x_m < x_m_min) x_m_min = x_m;
            // else if (x_m > x_m_max) x_m_max = x_m;
            // if (y_m < y_m_min) y_m_min = y_m;
            // else if (y_m > y_m_max) y_m_max = y_m;
            // if (z_m < z_m_min) z_m_min = z_m;            
            // else if (z_m > z_m_max) z_m_max = z_m;
            
        } else {
            LED.setLED(i, LED.HSVL((int)(((double)i/40 + (double)led_step/1000) * 360) % 360, 1, .1, 16));
            // x_m -= (x_m_max + x_m_min);
            // y_m -= (y_m_max + y_m_min);
            // z_m -= (z_m_max + z_m_min);
        }
        }
        LED.update();
        led_step++;
        if (led_step > 1000) led_step = 0;


        accel1.getXYZ(accel_val_1);
        accel2.getXYZ(accel_val_2);
        packets.accel1.queued = true;
        packets.accel2.queued = true;
        if (enabled) {
            accel_avg_1 = accel1.getXYZ100Avg();
            accel_avg_2 = accel2.getXYZ100Avg();
        }
        packets.accel1Avg.queued = true;
        packets.accel2Avg.queued = true;

        CoR = orientator::getCenterOfRotation(accel_avg_2, accel_avg_1);
        packets.centerOfRotation.queued = true;

        if (cal.XL_r_z_up) {
            accel_off_1.x = -accel1.getXYZ100Avg().x;
            accel_off_1.y = -accel1.getXYZ100Avg().y;
            accel_off_1.z = -(accel1.getXYZ100Avg().z + 1/LSB_TO_G);
            accel1.setOffset(vec3<int16_t>{0,0,0});           
        } else {
            accel1.setOffset(accel_off_1);           
        }
        if (cal.XL_l_z_up) {
            accel_off_2.x = -accel2.getXYZ100Avg().x;
            accel_off_2.y = -accel2.getXYZ100Avg().y;
            accel_off_2.z = -(accel2.getXYZ100Avg().z + 1/LSB_TO_G);
            accel2.setOffset(vec3<int16_t>{0,0,0});           
        } else {
            accel2.setOffset(accel_off_2);           
        }

        L_ESC.Throttle_Write((uint16_t)state.motor_percentage);
        R_ESC.Throttle_Write((uint16_t)state.motor_percentage);

        packets.mag.queued = true;
        
        packets.enabled.queued = true;
        deltaTime = ((float)esp_timer_get_time()) / 1000000 - time;
        time = ((float)esp_timer_get_time()) / 1000000;
        packets.time.queued = true;
        packets.deltaTime.queued = true;
        if (telo_delay >= 30) {
            Telometer::update(&telemetry);

            telo_delay = 1;
            if (ESC_L_data.data_valid) {
                ESC_L_data.data_valid = false;
                ESP_LOGI("L_ESC", "eRPM: %u",(unsigned int)ESC_L_data.latest_eRPM);
            }
            if (ESC_R_data.data_valid) {
                ESC_R_data.data_valid = false;
                ESP_LOGI("R_ESC", "eRPM: %u",(unsigned int)ESC_R_data.latest_eRPM);
            }
        } else {
            telo_delay ++;
        }
        vTaskDelay(1);
    }
}
}
