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
        vec3<int16_t> offset;
        vec2<float> pos; // meters
        angle_t angle;
        virtual bool setup(sensor_config_t config) = 0;
        virtual uint8_t getDeviceID(void) = 0;
        virtual int16_t getX(void) = 0;
        virtual int16_t getY(void) = 0;
        virtual int16_t getZ(void) = 0;
        virtual bool getXY(vec2<int16_t> &vec) = 0;
        virtual bool getXYZ(vec3<int16_t> &vec) = 0;
};