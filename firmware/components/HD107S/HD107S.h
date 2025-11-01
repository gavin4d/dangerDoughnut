#ifndef HD107S_H
#define HD107S_H
#include "sdkconfig.h"
#include <cstdint>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_system.h>
#include <esp_log.h>
#include <driver/spi_master.h>

// https://www.rose-lighting.com/wp-content/uploads/sites/53/2021/04/HD107s-2020-LED-Specificaion-V1.1.0-Rose-Lighting11.pdf

// typedef uint32_t hd107s_color_t;
union hd107s_color_t {
    struct {
        uint8_t lum : 5 = 0b11111; // from 0 to 31
        uint8_t start : 3 = 0b111; // LEDs need three set frame start bits
        uint8_t b = 0;
        uint8_t g = 0;
        uint8_t r = 0;
    };
    uint32_t data;
    // hd107s_color_t& operator =(const hd107s_color_t& a)
    // {
    //     data = (a.data & 0xfffffff8) | 0x7;
    //     return *this;
    // }
};

#define HD107S_RGBL(red,green,blue,luminosity) (hd107s_color_t){.lum = luminosity, .b = blue, .g = green, .r = red}
#define HD107S_HEXL(hex,luminosity) (hd107s_color_t){.lum = luminosity, .b = (hex >> 16) & 0xff, .g = (hex >> 8) & 0xff, .r = hex & 0xff}
// #define HD107S_INTL(int,lum) (((int & 0xFF)<<16) | (((int >> 8) & 0xFF)<<8) | ((int >> 16) & 0xFF) | ((((lum & 0xFF) >> 3) | 0xE0) << 24))

#define RGBL HD107S_RGBL

struct hd107s_config_t{
    uint8_t dataPin;
    uint8_t clockPin;
    uint16_t numLEDs = 1;
    uint32_t clockSpeed = 1000000;
    uint8_t DMAChannel = SPI_DMA_CH_AUTO;
    spi_host_device_t SPIHost = SPI3_HOST;
};

class HD107S {

    public:
        HD107S();
        ~HD107S();
		// HD107S& operator=(const HD107S &inputHD107S);

        void setup(hd107s_config_t config);
        void setLED(uint16_t index, hd107s_color_t color);
        void update(hd107s_color_t* ex_buffer);
        hd107s_color_t HSVL(float h, float s, float v, uint8_t lum);

    private:
        hd107s_color_t* strip_buffer;
        int16_t numLEDs;
        spi_host_device_t SPIHost;
	    int DMAChannel;
        uint32_t clockSpeed;
        spi_bus_config_t bus_config;
        spi_device_interface_config_t dev_config;
        spi_transaction_t transaction;
        spi_device_handle_t device;
};
#endif