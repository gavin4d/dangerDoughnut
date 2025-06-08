#ifndef MMC5983MA_H
#define MMC5983MA_H

#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "MathUtils.h"
#include "sensor.h"

// https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/333/MMC5983MA_RevA_4-3-19.pdf

// Registers
#define MMC_X_OUT_0 (0x00)
#define MMC_X_OUT_1 (0x01)
#define MMC_Y_OUT_0 (0x02)
#define MMC_Y_OUT_1 (0x03)
#define MMC_Z_OUT_0 (0x04)
#define MMC_Z_OUT_1 (0x05)
#define MMC_XYZ_UT_2 (0x06)
#define MMC_T_OUT (0x07)
#define MMC_STATUS (0x08)
#define MMC_INTERNAL_CONTROL_0 (0x09)
#define MMC_INTERNAL_CONTROL_1 (0x0A)
#define MMC_INTERNAL_CONTROL_2 (0x0B)
#define MMC_INTERNAL_CONTROL_3 (0x0C)
#define MMC_PRODUCT_ID_1 (0x2F)

#define MMC_WRITE (0x00)           /**< write bit */
#define MMC_READ (0x80)            /**< read bit */

typedef enum {
  MMC_AUTO_SR = 0b00100000,
  MMC_SET = 0b00001000,
  MMC_RESET = 0b00010000,
  MMC_INT_EN = 0b00000100,
  MMC_TAKE_MEASURMENT_M = 0b00000001,
  MMC_TAKE_MEASURMENT_T = 0b00000010,
  MMC_READ_OTP = 0b01000000
} mmc_control_0_t;

#define MMC_CONTINUOUS_MODE (0b1000)
typedef enum {
  MMC_DATARATE_1000_HZ = 0b111, /**< 1000Hz */
  MMC_DATARATE_200_HZ = 0b110, /**< 200Hz */
  MMC_DATARATE_100_HZ = 0b101, /**< 100Hz */
  MMC_DATARATE_50_HZ = 0b100,  /**< 50Hz */
  MMC_DATARATE_20_HZ = 0b011, /**< 20Hz */
  MMC_DATARATE_10_HZ = 0b010, /**< 10Hz */
  MMC_DATARATE_1_HZ = 0b001, /**< 1Hz */
  MMC_DATARATE_0_HZ = 0b000 /**< Continuous mode off */
} mmc_dataRate_t;

typedef enum {
  MMC_BANDWIDTH_800_HZ = 0b11, /**< 800Hz */
  MMC_BANDWIDTH_400_HZ = 0b10, /**< 400Hz */
  MMC_BANDWIDTH_200_HZ = 0b01, /**< 200Hz */
  MMC_BANDWIDTH_100_HZ = 0b00,  /**< 100Hz */
} mmc_bandwidth_t;

// struct mmc5983ma_config_t{
//     uint8_t miso_pin;
//     uint8_t mosi_pin;
//     uint8_t clock_pin;
//     uint8_t cs_pin;
//     uint32_t clock_speed = 10000000;
//     uint8_t DMA_channel = SPI_DMA_CH_AUTO;
//     spi_host_device_t SPI_host = SPI2_HOST;
// };

class MMC5983MA : public Sensor {

private:
    esp_err_t ret;
    spi_host_device_t SPI_host;
    int DMA_channel;
    uint32_t clock_speed;
    spi_bus_config_t bus_config = {};
    spi_device_handle_t device;
    uint8_t *rx_buffer;
    uint8_t *tx_buffer;
    vec3<int16_t> offset = {0,0,0};
public:
    MMC5983MA();
    ~MMC5983MA();
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    uint8_t getDeviceID(void);
    int16_t getX(void);
    int16_t getY(void);
    int16_t getZ(void);
    bool getXYZ(vec3<int16_t> &vec);
    bool getXY(vec2<int16_t> &vec);
    bool setup(sensor_config_t config);
};


#endif
