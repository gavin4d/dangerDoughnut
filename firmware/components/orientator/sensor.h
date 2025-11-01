#pragma once
#include "driver/spi_master.h"
#include "MathUtils.h"
#include <cstdint>

struct sensor_config_t{
    uint8_t miso_pin;
    uint8_t mosi_pin;
    uint8_t clock_pin;
    uint8_t cs_pin;
    uint32_t clock_speed = 10000000;
    uint8_t DMA_channel = SPI_DMA_CH_AUTO;
    spi_host_device_t SPI_host = SPI2_HOST;
};

class Sensor {
    public:
        vec3<float> offset = {0,0,0};
        vec2<float> pos = {0,0}; // meters
        angle_t angle;
        virtual bool setup(sensor_config_t config) = 0;
        virtual uint8_t getDeviceID(void) = 0;
        virtual int16_t readX(void) = 0;
        virtual int16_t readY(void) = 0;
        virtual int16_t readZ(void) = 0;
        virtual bool readXY(vec2<int16_t> &vec) = 0;
        virtual bool readXYZ(vec3<int16_t> &vec) = 0;
        bool getXYZ(vec3<int16_t> &vec) {
            bool return_value = readXYZ(vec);
            vec = vec - (vec3<int16_t>) offset;
            return return_value;
        }
};