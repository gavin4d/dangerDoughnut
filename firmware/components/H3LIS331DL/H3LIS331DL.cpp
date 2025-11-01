#include "H3LIS331DL.h"
#include "MathUtils.h"
#include "esp_log.h"
#include <cstdint>
#include <cstring>


H3LIS331DL::H3LIS331DL() {
}

H3LIS331DL::~H3LIS331DL() {
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

void H3LIS331DL::writeRegister(uint8_t reg, uint8_t value) {
    spi_transaction_t transaction = {0};
    transaction.flags = SPI_TRANS_USE_TXDATA;
    transaction.addr = 0;
    transaction.tx_data[0] = reg | H3LIS_WRITE;
    transaction.tx_data[1] = value;
    transaction.length = 8*2;
    ESP_LOGI("addr_w", "%2x", (uint8_t)transaction.tx_data[0]);

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
uint8_t H3LIS331DL::readRegister(uint8_t reg) {
    spi_transaction_t transaction = {0};
    transaction.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    transaction.addr = 0;
    transaction.length = 8*2;
    transaction.tx_data[0] = reg | H3LIS_READ;
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
    @brief  Reads 16-bits from the specified register

    @param reg The register to read two bytes from

    @return The 16-bit value read from the reg starting address
*/
/**************************************************************************/

int16_t H3LIS331DL::read16(uint8_t reg) {
    spi_transaction_t transaction;
    transaction.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    transaction.addr = 0;
    transaction.length = 8*3;
    transaction.tx_data[0] = reg | H3LIS_READ | H3LIS_MULTIPLE_BYTES;
    transaction.tx_data[1] = 0xFF;
    transaction.tx_data[2] = 0xFF;
    transaction.rxlength = 8*2;

    ret=spi_device_transmit(device, &transaction);
    assert(ret==ESP_OK);
    
    uint32_t data;
    std::memcpy(&data, transaction.rx_data, sizeof(int));
    //ESP_LOGI("read", "%4X", transaction.rx_data[1] | (transaction.rx_data[2] << 8));

    return (int16_t)(transaction.rx_data[1] | (transaction.rx_data[2] << 8));
}

/**************************************************************************/
/*!
    @brief  Read the device ID (can be used to check connection)

    @return The 8-bit device ID
*/
/**************************************************************************/
uint8_t H3LIS331DL::getDeviceID(void) {
  // Check device ID register
  return readRegister(H3LIS_WHO_AM_I);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent X axis value

    @return The 16-bit signed value for the X axis
*/
/**************************************************************************/
int16_t H3LIS331DL::readX(void) { return read16(H3LIS_OUT_X_L) >> 4; }

/**************************************************************************/
/*!
    @brief  Gets the most recent Y axis value

    @return The 16-bit signed value for the Y axis
*/
/**************************************************************************/
int16_t H3LIS331DL::readY(void) { return read16(H3LIS_OUT_Y_L) >> 4; }

/**************************************************************************/
/*!
    @brief  Gets the most recent Z axis value

    @return The 16-bit signed value for the Z axis
*/
/**************************************************************************/
int16_t H3LIS331DL::readZ(void) { return read16(H3LIS_OUT_Z_L) >> 4; }

/**************************************************************************/
/*!
    @brief  Reads 3x16-bits from the x, y, and z data register
    @param vec reference to return vector acceleration data
    @return True if the operation was successful, otherwise false.
*/
/**************************************************************************/
bool H3LIS331DL::readXYZ(vec3<int16_t> &vec) {
    *tx_buffer = H3LIS_OUT_X_L | H3LIS_READ | H3LIS_MULTIPLE_BYTES;
    spi_transaction_t transaction;
    transaction.flags = 0;
    transaction.addr = 0;
    transaction.length = 8*7;
    transaction.rxlength = 0; // same as length
    transaction.tx_buffer = tx_buffer;
    transaction.rx_buffer = rx_buffer;

    ret=spi_device_acquire_bus(device, portMAX_DELAY);
    assert(ret==ESP_OK);
    ret=spi_device_polling_transmit(device, &transaction);
    assert(ret==ESP_OK);
    spi_device_release_bus(device);

    vec.x = ((int16_t)(rx_buffer[1] | rx_buffer[2] << 8) >> 4);
    vec.y = ((int16_t)(rx_buffer[3] | rx_buffer[4] << 8) >> 4);
    vec.z = ((int16_t)(rx_buffer[5] | rx_buffer[6] << 8) >> 4);

    // avg = avg + ((vec3<float>)vec - avg)/AVERAGE_SIZE; // rolling average of x axis accel
    // avg.y += (vec.y - avg.y)/AVERAGE_SIZE; // rolling average of y axis accel
    // avg.z += (vec.z - avg.z)/AVERAGE_SIZE; // rolling average of z axis accel
    
    return true; // TODO: return false on read fail
}

/**************************************************************************/
/*!
    @brief  Reads 2x16-bits from the x, and y data registers
    @param vec reference to return vector acceleration data
    @return True if the operation was successful, otherwise false.
*/
/**************************************************************************/
bool H3LIS331DL::readXY(vec2<int16_t> &vec) {
    *tx_buffer = H3LIS_OUT_X_L | H3LIS_READ | H3LIS_MULTIPLE_BYTES;
    spi_transaction_t transaction;
    transaction.flags = 0;
    transaction.addr = 0;
    transaction.length = 8*5;
    transaction.rxlength = 0; // same as length
    transaction.tx_buffer = tx_buffer;
    transaction.rx_buffer = rx_buffer;

    ret=spi_device_polling_transmit(device, &transaction);
    assert(ret==ESP_OK);

    vec.x = ((int16_t)(rx_buffer[1] | rx_buffer[2] << 8) >> 4);
    vec.y = ((int16_t)(rx_buffer[3] | rx_buffer[4] << 8) >> 4);
    return true; // TODO: return false on read fail
}

/**************************************************************************/
/*!
    @brief  Provides the average of X, Y, and Z from the last 100 readings
    @param x reference to return average x acceleration data
    @param y reference to return average y acceleration data
    @param z reference to return average z acceleration data
*/
/**************************************************************************/
// vec3<float> H3LIS331DL::getXYZAvg() {
//     return avg;
// }

// void H3LIS331DL::setOffset(vec3<int16_t> offset) {
//     H3LIS331DL::offset = offset;
// }

// void H3LIS331DL::setOffset(vec3<float> offset) {
//     H3LIS331DL::offset = (vec3<int16_t>)offset;
// }

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
    @return True if the sensor was successfully initialised.
*/
/**************************************************************************/
bool H3LIS331DL::setup(sensor_config_t config) {

    rx_buffer = (uint8_t*)heap_caps_aligned_alloc(32, 7, MALLOC_CAP_DMA);
    tx_buffer = (uint8_t*)heap_caps_aligned_alloc(32, 7, MALLOC_CAP_DMA);

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
    
    dev_config = {
        .address_bits = 0,
        .mode = 3,
        .clock_speed_hz = (int) config.clock_speed,
        .spics_io_num = config.cs_pin,
        .queue_size = 1,
    };
    
    ret=spi_bus_add_device(SPI_host, &dev_config, &device);
    assert(ret==ESP_OK);

    /* Check connection */
    uint8_t deviceid = getDeviceID();
    if (deviceid != 0x32) {
        /* No H3LIS331DL detected ... return false */
        return false;
    }

    // Power and data rate
    writeRegister(H3LIS_CTRL_REG1, H3LIS_NORMAL_POWER | H3LIS_DATARATE_1000_HZ | H3LIS_XYZ_EN);

    // 400g full scale
    writeRegister(H3LIS_CTRL_REG4, 0b10000000 | H3LIS_400_G); 

    return true;
}
