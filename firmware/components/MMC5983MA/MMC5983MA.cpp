#include "MMC5983MA.h"
#include "MathUtils.h"
#include "esp_log.h"
#include <bits/std_thread.h>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "freertos/task.h"

MMC5983MA::MMC5983MA() {
}

MMC5983MA::~MMC5983MA() {
    free(rx_buffer);
    free(tx_buffer);
}

/*************************************************************************/
/*!
    @brief  Writes 8-bits to the specified destination register

    @param reg The register to write to
    @param value The value to write to the register
*/
/**************************************************************************/

void MMC5983MA::writeRegister(uint8_t reg, uint8_t value) {
    spi_transaction_t transaction = {0};
    transaction.flags = SPI_TRANS_USE_TXDATA;
    transaction.addr = 0;
    transaction.tx_data[0] = reg | MMC_WRITE;
    transaction.tx_data[1] = value;
    transaction.length = 8*2;
    ESP_LOGI("addr_w", "%02x", (uint8_t)transaction.tx_data[0]);

    ret=spi_device_transmit(device, &transaction);
    assert(ret==ESP_OK);
}

/**************************************************************************/
/*!
    @brief  Reads 8-bits from the specified register

    @param reg register to read

    @return The results of the register read request
*/
/**************************************************************************/
uint8_t MMC5983MA::readRegister(uint8_t reg) {
    spi_transaction_t transaction = {0};
    transaction.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    transaction.addr = 0;
    transaction.length = 8*2;
    transaction.tx_data[0] = reg | MMC_READ;
    transaction.tx_data[1] = 0xFF;
    transaction.rxlength = 8;
    ESP_LOGI("addr_r", "%2x", (uint8_t)transaction.tx_data[0]);

    ret=spi_device_transmit(device, &transaction);
    assert(ret==ESP_OK);
    ESP_LOGI("read", "%2x", transaction.rx_data[1]);

    return transaction.rx_data[1];
}

/**************************************************************************/
/*!
    @brief  Read the device ID (can be used to check connection)

    @return The 8-bit device ID
*/
/**************************************************************************/
uint8_t MMC5983MA::getDeviceID(void) {
  // Check device ID register
  return readRegister(MMC_PRODUCT_ID_1);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent X axis value

    @return The 16-bit signed value for the X axis
*/
/**************************************************************************/
int16_t MMC5983MA::getX(void) { return 0; }

/**************************************************************************/
/*!
    @brief  Gets the most recent Y axis value

    @return The 16-bit signed value for the Y axis
*/
/**************************************************************************/
int16_t MMC5983MA::getY(void) { return 0; }

/**************************************************************************/
/*!
    @brief  Gets the most recent Z axis value

    @return The 16-bit signed value for the Z axis
*/
/**************************************************************************/
int16_t MMC5983MA::getZ(void) { return 0; }

/**************************************************************************/
/*!
    @brief  Reads 3x16-bits from the x, y, and z data register
    @param vec reference to return vector of magnetic field data
    @return True if the operation was successful, otherwise false.
*/
/**************************************************************************/
bool MMC5983MA::getXYZ(vec3<int16_t> &vec) {
    tx_buffer[0] = MMC_X_OUT_0 | MMC_READ;
    spi_transaction_t transaction;
    transaction.flags = 0;
    transaction.addr = 0;
    transaction.length = 8*8;
    transaction.rxlength = 0; // same as length
    transaction.tx_buffer = tx_buffer;
    transaction.rx_buffer = rx_buffer;

    ret=spi_device_acquire_bus(device, portMAX_DELAY);
    assert(ret==ESP_OK);
    ret=spi_device_polling_transmit(device, &transaction);
    assert(ret==ESP_OK);
    spi_device_release_bus(device);

    vec.x = (rx_buffer[1] << 10 | rx_buffer[2] << 2 | ((rx_buffer[7] >> 6) & 0b11)) - (1 << 17) + offset.x;
    vec.y = (rx_buffer[3] << 10 | rx_buffer[4] << 2 | ((rx_buffer[7] >> 4) & 0b11)) - (1 << 17) + offset.y;
    vec.z = (rx_buffer[5] << 10 | rx_buffer[6] << 2 | ((rx_buffer[7] >> 2) & 0b11)) - (1 << 17) + offset.z;
    return true;
}

/**************************************************************************/
/*!
    @brief  Reads 3x16-bits from the x, y, and z data register
    @param x reference to return x magnetic field data
    @param y reference to return y magnetic field data
    @param z reference to return z magnetic field data
    @return True if the operation was successful, otherwise false.
*/
/**************************************************************************/
bool MMC5983MA::getXY(vec2<int16_t> &vec) {
    tx_buffer[0] = MMC_X_OUT_0 | MMC_READ;
    spi_transaction_t transaction;
    transaction.flags = 0;
    transaction.addr = 0;
    transaction.length = 8*8;
    transaction.rxlength = 0; // same as length
    transaction.tx_buffer = tx_buffer;
    transaction.rx_buffer = rx_buffer;

    ret=spi_device_acquire_bus(device, portMAX_DELAY);
    assert(ret==ESP_OK);
    ret=spi_device_polling_transmit(device, &transaction);
    assert(ret==ESP_OK);
    spi_device_release_bus(device);

    vec.x = (rx_buffer[1] << 10 | rx_buffer[2] << 2 | ((rx_buffer[7] >> 6) & 0b11)) - (1 << 17) + offset.x;
    vec.y = (rx_buffer[3] << 10 | rx_buffer[4] << 2 | ((rx_buffer[7] >> 4) & 0b11)) - (1 << 17) + offset.y;
    return true;
}

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
    @return True if the sensor was successfully initialised.
*/
/**************************************************************************/
bool MMC5983MA::setup(sensor_config_t config) {

    rx_buffer = (uint8_t*)heap_caps_aligned_alloc(32, 8, MALLOC_CAP_DMA);
    tx_buffer = (uint8_t*)heap_caps_aligned_alloc(32, 8, MALLOC_CAP_DMA);

    DMA_channel = config.DMA_channel;
    SPI_host = config.SPI_host;
    clock_speed = config.clock_speed;
    bus_config = {
        .mosi_io_num = config.mosi_pin,
        .miso_io_num = config.miso_pin,
        .sclk_io_num = config.clock_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };
    
    ret=spi_bus_initialize(SPI_host, &bus_config, DMA_channel);
    assert(ret==ESP_OK || ret==ESP_ERR_INVALID_STATE); // continue if bus is already initialized

    spi_device_interface_config_t dev_config = {
        .command_bits = 0,
        .address_bits = 0,
        .mode = 3,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = (int)config.clock_speed, // why is clock speed a signed int????
        .spics_io_num = config.cs_pin,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL,
    };

    ret=spi_bus_add_device(SPI_host, &dev_config, &device);
    assert(ret==ESP_OK);

    // writeRegister(MMC_INTERNAL_CONTROL_1, 0b10000000); // Reset
    // vTaskDelay(2);
    
    /* Check connection */
    if (getDeviceID() != 0x30) {
        /* No MMC5983MA detected ... return false */
        return false;
    }

    // Data rate
    writeRegister(MMC_INTERNAL_CONTROL_1, MMC_BANDWIDTH_800_HZ);

    vec3<int16_t> vec_1 = {0,0,0};
    vec3<int16_t> vec_2 = {0,0,0};
    // SET and RESET to find sensor offsets
    writeRegister(MMC_INTERNAL_CONTROL_0, MMC_SET | MMC_TAKE_MEASURMENT_M);
    vTaskDelay(1);
    writeRegister(MMC_INTERNAL_CONTROL_0, MMC_SET);
    vTaskDelay(1);
    writeRegister(MMC_INTERNAL_CONTROL_0, MMC_TAKE_MEASURMENT_M);
    vTaskDelay(1);
    getXYZ(vec_1);
    writeRegister(MMC_INTERNAL_CONTROL_0, MMC_RESET);
    vTaskDelay(1);
    writeRegister(MMC_INTERNAL_CONTROL_0, MMC_TAKE_MEASURMENT_M);
    vTaskDelay(1);
    getXYZ(vec_2);
    offset = (vec3<int16_t>)((vec3<int16_t>)vec_1 + (vec3<int16_t>)vec_2) * 0.5;
    ESP_LOGI("offsets", "x = %d, y = %d, z = %d", offset.x, offset.y, offset.z);
    writeRegister(MMC_INTERNAL_CONTROL_0, MMC_SET);
    vTaskDelay(1);

    writeRegister(MMC_INTERNAL_CONTROL_0, 0b00100000);
    vTaskDelay(1);

    writeRegister(MMC_INTERNAL_CONTROL_2, MMC_CONTINUOUS_MODE | MMC_DATARATE_1000_HZ | 0b10100000);

    // for (int i = 0; i <= 0xC; i++) {
    //     ESP_LOGI("mag", "%d: %02x", i, readRegister(i));
    // }

    return true;
}
