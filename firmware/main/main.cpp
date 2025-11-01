#include "driver/spi_common.h"
#include "esp_err.h"
#include "hal/adc_types.h"
#include "sdkconfig.h"

#include "esp_log.h"
#include <cstdint>
#include <functional>
#include <numbers>
#include <stdio.h>
#include <variant>
#include <esp_adc/adc_oneshot.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "BLEBackend.h"
#include "Packets.h"
#include "TelometerImpl.h"
#include "Telometer.h"
#include "soc/clk_tree_defs.h"
extern "C" {
    #include "ESP_CRSF.h"
}
#include <format>
#include <iostream>

#include "HD107S.h"
#include "H3LIS331DL.h"
#include "MMC5983MA.h"
#include "MathUtils.h"
#include "math.h"
#include "donutPhysics.h"
#include "orientator.h"
#include "DShot_ESC.h"
// #include "POVDisplay.h"
#include "donutDisplay.h"
#include "soc/gpio_num.h"

#define LED_COUNT 20U
#define LED_DATA_PIN 11
#define LED_CLOCK_PIN 12
#define LED_CLOCK_SPEED 40000000 // Damn!!! 

#define MAX_MELTY_POWER 0.5

#define ESC_L_GPIO GPIO_NUM_38
#define ESC_R_GPIO GPIO_NUM_35

HD107S LED;

H3LIS331DL accel1;
H3LIS331DL accel2;

MMC5983MA mag;

Orientator orientator;

adc_oneshot_unit_handle_t adc2_handle;

struct TelemetryPackets packets = initTelemetryPackets();
Telometer::Backend *myBackend;

Telometer::TelometerInstance telemetry;

float getBattVoltage() {
    int voltageReading = 0;
    esp_err_t err = adc_oneshot_read(adc2_handle, ADC_CHANNEL_3, &voltageReading);
    return err == ESP_OK ? (float)voltageReading * 19 / 4095 : 0; // 1.1V ref, "6dB" attenuation (actually 4.75dB), 1:10 voltage divider
}

extern "C" {
void app_main(void) {

    DonutPhysics physics;
    orientator = Orientator(&physics);

    myBackend = (Telometer::Backend*) new Telometer::BLEBackend();
    telemetry = {
        .backend  = myBackend,
        .count = TelemetryPacketCount,
        .nextPacket = 0,
        .packetStruct = (Telometer::Data*)&packets
    };
    Telometer::init(&telemetry);

    // open NVS flash for config 
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW("NVS", "NVS partition was truncated and has been erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    
    uint8_t telo_delay = 0;
    bool enabled = 0;
    float time = 0;
    float deltaTime = 0.001;
    cal_en cal = {
        .XL = 0,
        .Mag = 1,
        .save = 0,
    };
    vec2<float> wheel_power = {0,0};
    
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
    packets.process_noise.pointer = &physics.process_noise;
    packets.accel_varience.pointer = &orientator.accel_varience;
    packets.mag_varience.pointer = &orientator.mag_varience;    
    packets.state.pointer = &orientator.state;
    packets.measuredState.pointer = &orientator.measured_state;
    packets.wheel_power.pointer = &wheel_power;
    

    crsf_config_t config = {
        .uart_num = UART_NUM_0,
        .tx_pin = 43,
        .rx_pin = 44
    };
    CRSF_init(&config);
    crsf_channels_t channels = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    crsf_battery_t battery = {0,0,0,0};

    sensor_config_t accel_config;
    accel_config.clock_pin = 5;
    accel_config.miso_pin = 3;
    accel_config.mosi_pin = 4;
    accel_config.clock_speed = 10000000; // 10 MHz
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
    mag_config.clock_speed = 10000000;
    mag_config.cs_pin = 33;
    mag.setup(mag_config);

    orientator.setup(&accel1, &accel2, &mag);
    orientator.readConfig();
    
    hd107s_config_t LED_config;
    LED_config.dataPin = LED_DATA_PIN;
    LED_config.clockPin = LED_CLOCK_PIN;
    LED_config.numLEDs = LED_COUNT;
    LED_config.clockSpeed = LED_CLOCK_SPEED;
    LED.setup(LED_config);
    DonutDisplay display = DonutDisplay(&LED, &orientator);

    packets.PLL_bias.pointer = &display.PLL_bias;    
    
    adc_oneshot_unit_init_cfg_t adc2_config = {
        .unit_id = ADC_UNIT_2,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
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
    packets.calibration.queued = true;
    orientator.state.battery_voltage = getBattVoltage();
    
    ESP_LOGI(TAG, "Completed Init");
    // bool reversed = false;

    display.drawText(10, 0, 0, "\1 DANGER\nDOUGHNUT", HD107S_RGBL(255, 255, 0, 1));
    display.setPixel(0, 0, HD107S_RGBL(0, 255, 0, 15));
    // display.drawLine(90, 15, 150, 10, HD107S_RGBL(127, 255, 255, 1));
    // for (int i = 0; i < 20; i++) {
    //     display.render();
    //     display.printFrameBuffer();
    //     display.clearFrameBuffer(!display.displayed_buffer);
    //     vTaskDelay(1);
    //     display.animate();
    // }
    // display.render();
    display.printFrameBuffer();
    display.swapBuffer();

    display.display(75, 0);
    while(true) {
        time = ((float)esp_timer_get_time()) / 1000000;
        orientator.update();
        packets.accel1.queued = true;
        packets.accel2.queued = true;
        packets.accel1Avg.queued = true;
        packets.accel2Avg.queued = true;
        packets.centerOfRotation.queued = true;
        packets.mag_varience.queued = true;
        packets.accel_varience.queued = true;
        packets.process_noise.queued = true;
        packets.state.queued = true;
        packets.measuredState.queued = true;

        if(CRSF_receive_channels(&channels) && CRSF_convert_switch(channels.ch8)) {
            // TODO: prevent motor activation if controller connects with nonzero motor power
            // printf("CH1: %0.3f, CH2: %0.3f, CH3: %0.3f, CH4: %0.3f, CH5: %d, CH6: %d, CH7: %d, CH8: %d, CH9: %d, CH10: %0.3f\n", CRSF_convert_stick(channels.ch1), CRSF_convert_stick(channels.ch2), CRSF_convert_stick(channels.ch3), CRSF_convert_stick(channels.ch4), CRSF_convert_switch(channels.ch5), CRSF_convert_switch(channels.ch6), CRSF_convert_switch(channels.ch7), CRSF_convert_switch(channels.ch8), CRSF_convert_switch(channels.ch9), CRSF_convert_analog(channels.ch10));
            const float motor_percentage = CRSF_convert_analog(channels.ch10)*(CRSF_convert_stick(channels.ch3)+1)/2;
            const float x = -CRSF_convert_stick(channels.ch1);
            const float y = -CRSF_convert_stick(channels.ch2);
            // printf("x: %f, y: %f\n", x, y);
            float drive_angle = atan2(y, x) + LSB2RAD*orientator.getHeading();
            if (drive_angle > 2 * std::numbers::pi) drive_angle -= 2 * std::numbers::pi;
            const float drive_power = MAX_MELTY_POWER * hypot(x, y);
            // printf("a: %f, p: %f\n", drive_angle, drive_power);
            const uint16_t left_motor_power = (uint16_t)(1999*(motor_percentage*(1-drive_power*sin(drive_angle))/fmax(1, motor_percentage + drive_power)) + 48);
            // wheel_power = {cos(LSB2RAD*orientator.getHeading()), sin(LSB2RAD*orientator.getHeading())};
            wheel_power = MathUtils::angleFromRadians(LSB2RAD*orientator.getHeading()).angle;
            wheel_power = left_motor_power * wheel_power;
            packets.wheel_power.queued = true;
            const uint16_t right_motor_power = (uint16_t)(1999*(motor_percentage*(1+drive_power*sin(drive_angle))/fmax(1, motor_percentage + drive_power)) + 48);
            L_ESC.Throttle_Write(left_motor_power);
            R_ESC.Throttle_Write(right_motor_power);
            orientator.state.motor_percentage = motor_percentage;
            // if (motor_percentage < 0.01 && !CRSF_convert_switch(channels.ch5) == reversed) {
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
        battery.remaining = (orientator.state.battery_voltage-12)/5.4 * 100; //remaining % of battery

        CRSF_send_battery_data(CRSF_DEST_FC, &battery);        

        orientator.enableAccelCalibration(cal.XL);
        orientator.enableMagCalibration(cal.Mag);
        if (cal.save) {
            cal.save = false;
            packets.calibration.queued = true;
            orientator.writeConfig();
        }

        packets.mag.queued = true;
        
        packets.enabled.queued = true;
        packets.time.queued = true;
        packets.deltaTime.queued = true;
        if (telo_delay >= 30) {
            Telometer::update(&telemetry);
            display.display(orientator.getVelocity(), orientator.state.angular_acceleration);
            packets.PLL_bias.queued = true;
            orientator.state.battery_voltage = getBattVoltage();
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
