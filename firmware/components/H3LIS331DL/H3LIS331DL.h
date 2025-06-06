#ifndef H3LIS331DL_H
#define H3LIS331DL_H

#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "MathUtils.h"
#include <cstdint>

// https://www.st.com/resource/en/datasheet/h3lis331dl.pdf

// Registers
#define H3LIS_WHO_AM_I (0x0F)
#define H3LIS_CTRL_REG1 (0x20)
#define H3LIS_CTRL_REG2 (0x21)
#define H3LIS_CTRL_REG3 (0x22)
#define H3LIS_CTRL_REG4 (0x23)
#define H3LIS_CTRL_REG5 (0x24)
#define H3LIS_HP_FILTER_RESET (0x25)
#define H3LIS_REFERENCE (0x26)
#define H3LIS_STATUS_REG (0x27)
#define H3LIS_OUT_X_L (0x28)
#define H3LIS_OUT_X_H (0x29)
#define H3LIS_OUT_Y_L (0x2A)
#define H3LIS_OUT_Y_H (0x2B)
#define H3LIS_OUT_Z_L (0x2C)
#define H3LIS_OUT_Z_H (0x2D)
#define H3LIS_INT1_CFG (0x30)
#define H3LIS_INT1_SRC (0x31)
#define H3LIS_INT1_THS (0x32)
#define H3LIS_INT1_DURATION (0x33)
#define H3LIS_INT2_CFG (0x34)
#define H3LIS_INT2_SRC (0x35)
#define H3LIS_INT2_THS (0x36)
#define H3LIS_INT2_DURATION (0x37)

#define H3LIS_WRITE (0x00)           /**< write bit */
#define H3LIS_READ (0x80)            /**< read bit */
#define H3LIS_MULTIPLE_BYTES (0x40)  /**< read multiple bytes */

#define LSB_TO_G (0.195) /**< 195mg per lsb */
#define LSB_TO_MPS2 (0.48) /**< 0.48 m/s/s per lsb */

typedef enum {
  H3LIS_DATARATE_1000_HZ = 0b11000, /**< 1000Hz Bandwidth */
  H3LIS_DATARATE_400_HZ = 0b10000, /**< 400Hz Bandwidth */
  H3LIS_DATARATE_100_HZ = 0b01000, /**< 100Hz Bandwidth */
  H3LIS_DATARATE_50_HZ = 0b00000  /**< 50Hz Bandwidth (default value) */
} dataRate_t;

typedef enum {
  H3LIS_POWER_DOWN = 0b00000000,
  H3LIS_NORMAL_POWER = 0b00100000,
  H3LIS_LOW_POWER_10_HZ= 0b11000000,
  H3LIS_LOW_POWER_5_HZ= 0b10100000,
} power_t;

typedef enum {
  H3LIS_X_EN = 0b001,
  H3LIS_Y_EN = 0b010,
  H3LIS_Z_EN = 0b100,
  H3LIS_XYZ_EN = 0b111,
} axesEnable_t;

typedef enum {
  H3LIS_400_G = 0b110000,
  H3LIS_200_G = 0b010000,
  H3LIS_100_G = 0b000000,
} h3lisFullScale_t;

struct h3lis331dl_config_t{
    uint8_t miso_pin;
    uint8_t mosi_pin;
    uint8_t clock_pin;
    uint8_t cs_pin;
    uint32_t clock_speed = 10000000;
    uint8_t DMA_channel = SPI_DMA_CH_AUTO;
    spi_host_device_t SPI_host = SPI2_HOST;
    vec3<int16_t> offset;
};

class H3LIS331DL {

private:
    esp_err_t ret;
    spi_host_device_t SPI_host;
    int DMA_channel;
    uint32_t clock_speed;
    spi_bus_config_t bus_config;
    spi_device_interface_config_t dev_config;
    spi_device_handle_t device;
    uint8_t *rx_buffer;
    uint8_t *tx_buffer;
    vec3<int16_t> offset;
    vec3<float> avg;
public:
    H3LIS331DL();
    ~H3LIS331DL();
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    int16_t read16(uint8_t reg);
    uint8_t getDeviceID(void);
    int16_t getX(void);
    int16_t getY(void);
    int16_t getZ(void);
    bool getXY(vec2<int16_t> &vec);
    bool getXYZ(vec3<int16_t> &vec);
    vec3<float> getXYZ100Avg();
    void setOffset(vec3<int16_t> offset);
    void setOffset(vec3<float> offset);
    bool setup(h3lis331dl_config_t config);
};


#endif
