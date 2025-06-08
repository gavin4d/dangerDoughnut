#include "driver/spi_common.h"
#include "hal/adc_types.h"
#include "sdkconfig.h"

#include "esp_log.h"
#include <cstdint>
#include <functional>
#include <stdio.h>
#include <variant>
#include <esp_adc/adc_oneshot.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "BLEBackend.h"
#include "Packets.h"
#include "TelometerImpl.h"
#include "Telometer.h"
extern "C" {
    #include "ESP_CRSF.h"
}

#include "HD107S.h"
#include "H3LIS331DL.h"
#include "MMC5983MA.h"
#include "MathUtils.h"
#include "orientator.h"
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

orientator orientator;
// DonutPhysics physics;

adc_oneshot_unit_handle_t adc2_handle;

struct TelemetryPackets packets = initTelemetryPackets();
Telometer::Backend *myBackend;


Telometer::TelometerInstance telemetry;

float getBattVoltage() {
    int voltageReading = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, ADC_CHANNEL_3, &voltageReading));
    return (float)voltageReading * 19 / 4095; // 1.1V ref, "6dB" attenuation (actually 4.75dB), 1:10 voltage divider
}
    
extern "C" {
void app_main(void) {

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
    packets.accel1.pointer = &orientator.accel_val_1;
    packets.accel2.pointer = &orientator.accel_val_2;
    packets.accel1Avg.pointer = &orientator.accel_avg_1;
    packets.accel2Avg.pointer = &orientator.accel_avg_2;
    packets.centerOfRotation.pointer = &orientator.CoR;
    packets.mag.pointer = &orientator.vec_m;
    packets.state.pointer = &orientator.state;

    crsf_config_t config = {
        .uart_num = UART_NUM_0,
        .tx_pin = 43,
        .rx_pin = 44
    };
    CRSF_init(&config);
    crsf_channels_t channels = {0};
    crsf_battery_t battery = {0};

    hd107s_config_t LED_config;
    LED_config.dataPin = LED_DATA_PIN;
    LED_config.clockPin = LED_CLOCK_PIN;
    LED_config.numLEDs = LED_COUNT;
    LED_config.clockSpeed = LED_CLOCK_SPEED;
    LED.setup(LED_config);

    sensor_config_t accel_config;
    accel_config.clock_pin = 5;
    accel_config.miso_pin = 3;
    accel_config.mosi_pin = 4;
    accel_config.cs_pin = 37;
    accel1.setup(accel_config);
    accel1.angle = MathUtils::angleFromDegrees(66.347);
    accel1.pos = {0.015038, -0.03848};
    accel_config.cs_pin = 2;
    accel2.setup(accel_config);
    accel2.angle = MathUtils::angleFromDegrees(23.654);
    accel2.pos = {-0.015038, -0.03848};

    sensor_config_t mag_config;
    mag_config.clock_pin = 5;
    mag_config.miso_pin = 3;
    mag_config.mosi_pin = 4;
    mag_config.cs_pin = 33;
    mag.setup(mag_config);

    orientator.setup(&accel1, &accel2, &mag);
    
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

    packets.state.queued = true;
    
    // mag.getXYZ(x_m_zero, y_m_zero, z_m_zero);
    ESP_LOGI(TAG, "Completed Init");
    int led_step = 0;
    bool reversed = false;
    vec3<int16_t> accel_val_1, accel_val_2;

    while(true) {
        time = ((float)esp_timer_get_time()) / 1000000;
        orientator.state.battery_voltage = getBattVoltage();
        // physics.step(state, deltaTime); // TODO: calc delta time here
        orientator.update();
        // accel1.getXYZ(accel_val_1);
        // accel2.getXYZ(accel_val_2);
        packets.accel1.queued = true;
        packets.accel2.queued = true;
        packets.accel1Avg.queued = true;
        packets.accel2Avg.queued = true;
        packets.centerOfRotation.queued = true;
        packets.state.queued = true;
        // mag.getXYZ(vec_m);

        if(CRSF_receive_channels(&channels) && CRSF_convert_switch(channels.ch8)) {
            // TODO: prevent motor activation is controller connects with nonzero motor power
            // printf("CH1: %0.3f, CH2: %0.3f, CH3: %0.3f, CH4: %0.3f, CH5: %d, CH6: %d, CH7: %d, CH8: %d, CH9: %d, CH10: %0.3f\n", CRSF_convert_stick(channels.ch1), CRSF_convert_stick(channels.ch2), CRSF_convert_stick(channels.ch3), CRSF_convert_stick(channels.ch4), CRSF_convert_switch(channels.ch5), CRSF_convert_switch(channels.ch6), CRSF_convert_switch(channels.ch7), CRSF_convert_switch(channels.ch8), CRSF_convert_switch(channels.ch9), CRSF_convert_analog(channels.ch10));
            float motor_percentage = 0.1*(CRSF_convert_stick(channels.ch3)+1)/2;
            L_ESC.Throttle_Write((uint16_t)(3999*motor_percentage + 48));
            R_ESC.Throttle_Write((uint16_t)(3999*motor_percentage + 48));
            orientator.state.motor_percentage = motor_percentage;
            // if (motor_percentage < 0.001 && !CRSF_convert_switch(channels.ch5) == reversed) {
                // TODO: set motors to reversed
            // }
            enabled = true;
        } else {
            enabled = false;
            orientator.state.motor_percentage = 0;
            L_ESC.Throttle_Write(48);
            R_ESC.Throttle_Write(48);
        }

        battery.voltage = orientator.state.battery_voltage*10; //voltage 10*V
        battery.remaining = (orientator.state.battery_voltage-12)/17.4 * 100; //remaining % of battery

        CRSF_send_battery_data(CRSF_DEST_FC, &battery);        

        
        for (int i = 0; i < LED_COUNT; i++) {
        if (enabled) {
            LED.setLED(i, LED.HSVL((int)(((double)i/40 + (double)led_step/1000) * 360) % 360, 1, .1, 16));
        } else {
            LED.setLED(i, RGBL(25,0,0,255));
        }
        }
        LED.update();
        led_step++;
        if (led_step > 1000) led_step = 0;

        // if (cal.XL_r_z_up) {
        //     accel_off_1.x = -accel1.getXYZ100Avg().x;
        //     accel_off_1.y = -accel1.getXYZ100Avg().y;
        //     accel_off_1.z = -(accel1.getXYZ100Avg().z + 1/LSB_TO_G);
        //     accel1.setOffset(vec3<int16_t>{0,0,0});           
        // } else {
        //     accel1.setOffset(accel_off_1);           
        // }
        // if (cal.XL_l_z_up) {
        //     accel_off_2.x = -accel2.getXYZ100Avg().x;
        //     accel_off_2.y = -accel2.getXYZ100Avg().y;
        //     accel_off_2.z = -(accel2.getXYZ100Avg().z + 1/LSB_TO_G);
        //     accel2.setOffset(vec3<int16_t>{0,0,0});           
        // } else {
        //     accel2.setOffset(accel_off_2);           
        // }

        packets.mag.queued = true;
        
        packets.enabled.queued = true;
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
        deltaTime = ((float)esp_timer_get_time()) / 1000000 - time;
        vTaskDelay(1);
    }
}
}
