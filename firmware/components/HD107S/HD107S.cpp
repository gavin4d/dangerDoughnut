#include "HD107S.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "portmacro.h"
#include <string.h>

#define TAG "HD107S"

using namespace std;

HD107S::HD107S() {

}

HD107S::~HD107S() {
	free(strip_buffer);
}

// HD107S& HD107S::operator=(const HD107S &inputHD107S) {
// 	if (this != &inputHD107S) {
//         free(buffer);
//         size_t size = (inputHD107S.numLEDs + 2) * sizeof(hd107s_color_t);
//         buffer = (hd107s_color_t*)heap_caps_malloc(size, MALLOC_CAP_DMA | MALLOC_CAP_32BIT);
//         std::copy(&inputHD107S.buffer[0], &inputHD107S.buffer[0]+size, &buffer[0]);
// 	}
// 	return *this;
// }

void HD107S::setup(hd107s_config_t config) {
	HD107S::numLEDs = config.numLEDs;
	HD107S::DMAChannel = config.DMAChannel;
	HD107S::SPIHost = config.SPIHost;
	HD107S::clockSpeed = config.clockSpeed;
	transaction.length = (8 * ((1 + numLEDs) * sizeof(hd107s_color_t)));
	transaction.addr = 0;
    bus_config = {
		.mosi_io_num = config.dataPin,
        .miso_io_num = -1,
        .sclk_io_num = config.clockPin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };
    dev_config = {
        .address_bits = 32,
        .mode = 3,
        .clock_speed_hz = (int) config.clockSpeed,
        .spics_io_num = -1,
        .flags = SPI_DEVICE_NO_DUMMY,
        .queue_size = 1,
    };
	bus_config.max_transfer_sz = transaction.length;

	spi_bus_initialize(SPIHost, &bus_config, DMAChannel);
	spi_bus_add_device(SPIHost, &dev_config, &device);

	size_t size = (numLEDs + 1) * sizeof(hd107s_color_t);
	strip_buffer = (hd107s_color_t*)heap_caps_malloc(size, MALLOC_CAP_DMA | MALLOC_CAP_32BIT);
	memset(strip_buffer, 0, size);
	strip_buffer[numLEDs].data = 0xFFFFFFFF; // end frame
}

void HD107S::setLED(uint16_t index, hd107s_color_t color) {
	strip_buffer[index] = color;
}

void HD107S::update(hd107s_color_t* ex_buffer) {
    spi_transaction_t* t;
	// skip write instead of blocking if previous write is not finished
	
    if (ex_buffer)
        transaction.tx_buffer = ex_buffer;
    else
    	transaction.tx_buffer = strip_buffer;

    spi_device_queue_trans(device, &transaction, 0);
	if (spi_device_get_trans_result(device, &t, portMAX_DELAY) != ESP_OK)
	    ;
}

hd107s_color_t HD107S::HSVL(float h, float s, float v, uint8_t lum) {
	float      hh, p, q, t, ff;
    long        i;
	hd107s_color_t color;
	color.lum = lum;

    if(s <= 0.0) {       // < is bogus, just shuts up warnings
        color.r = v*255;
        color.g = v*255;
        color.b = v*255;
        return color;
    }
    hh = h;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = v * (1.0 - s);
    q = v * (1.0 - (s * ff));
    t = v * (1.0 - (s * (1.0 - ff)));

    switch(i) {
    case 0:
        color.r = v*255;
        color.g = t*255;
        color.b = p*255;
        break;
    case 1:
        color.r = q*255;
        color.g = v*255;
        color.b = p*255;
        break;
    case 2:
        color.r = p*255;
        color.g = v*255;
        color.b = t*255;
        break;
    case 3:
        color.r = p*255;
        color.g = q*255;
        color.b = v*255;
        break;
    case 4:
        color.r = t*255;
        color.g = p*255;
        color.b = v*255;
        break;
    case 5:
    default:
        color.r = v*255;
        color.g = p*255;
        color.b = q*255;
        break;
    }
	return color;
} 
