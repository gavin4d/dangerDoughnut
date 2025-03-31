#include "sdkconfig.h"

#include "esp_log.h"
#include <cstdint>
#include <functional>
#include <stdio.h>
#include <driver/timer.h>
#include <variant>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "BLEBackend.h"
#include "Packets.h"
#include "TelometerImpl.h"
#include "Telometer.h"

#include "HD107S.h"
#include "H3LIS331DL.h"
#include "MMC5983MA.h"
#include "MathUtils.h"
#include "orientator.h"

#define LED_COUNT 20U
#define LED_DATA_PIN 11
#define LED_CLOCK_PIN 12
#define LED_STRIP_RMT_INTR_NUM 11U
#define LED_CLOCK_SPEED 32000000
#define RMT_CHANNEL	RMT_CHANNEL_0

HD107S LED;

H3LIS331DL accel1;
H3LIS331DL accel2;

MMC5983MA mag;

struct TelemetryPackets packets = initTelemetryPackets();
Telometer::Backend *myBackend;


Telometer::TelometerInstance telemetry;
extern "C" {
void app_main(void) {
    vec3<int32_t> vec_m = {0,0,0};
    vec3<int32_t> vec_m_min = {0,0,0};
    vec3<int32_t> vec_m_max = {0,0,0};
    vec3<int16_t> accel_val_1, accel_val_2, accel_off_1 = {0,0,0}, accel_off_2 = {0,0,0};
    vec3<float> accel_avg_1, accel_avg_2;
    vec2<float> CoR;

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
    float deltaTime = 0;
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
    packets.xMag.pointer = &vec_m.x;
    packets.yMag.pointer = &vec_m.y;
    packets.zMag.pointer = &vec_m.z;

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
    
    // mag.getXYZ(x_m_zero, y_m_zero, z_m_zero);
    ESP_LOGI(TAG, "Completed Init");

    while(true) {
        mag.getXYZ(vec_m);
        if (enabled) {
            LED.setLED(2, RGBL(0,255,0,255));
            // if (x_m < x_m_min) x_m_min = x_m;
            // else if (x_m > x_m_max) x_m_max = x_m;
            // if (y_m < y_m_min) y_m_min = y_m;
            // else if (y_m > y_m_max) y_m_max = y_m;
            // if (z_m < z_m_min) z_m_min = z_m;            
            // else if (z_m > z_m_max) z_m_max = z_m;
            
        } else {
            LED.setLED(2, RGBL(255,0,0,255));
            
            // x_m -= (x_m_max + x_m_min);
            // y_m -= (y_m_max + y_m_min);
            // z_m -= (z_m_max + z_m_min);
        }
        LED.update();

        accel1.getXYZ(accel_val_1);
        accel2.getXYZ(accel_val_2);
        packets.accel1.queued = true;
        packets.accel2.queued = true;
        accel_avg_1 = accel1.getXYZ100Avg();
        accel_avg_2 = accel2.getXYZ100Avg();
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

        packets.xMag.queued = true;
        packets.yMag.queued = true;
        packets.zMag.queued = true;
        
        packets.enabled.queued = true;
        deltaTime = ((float)esp_timer_get_time()) / 1000000 - time;
        time = ((float)esp_timer_get_time()) / 1000000;
        packets.time.queued = true;
        packets.deltaTime.queued = true;
        if (telo_delay >= 30) {
            Telometer::update(&telemetry);
            telo_delay = 0;
        } else {
            telo_delay ++;
        }
        vTaskDelay(1);
    }
}
}
