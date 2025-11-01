#include "donutDisplay.h"
#include "HD107S.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "fontStore.h"
#include "orientator.h"
#include <cstdint>
#include <cstdio>
#include <regex>

HD107S* DonutDisplay::LED;
hd107s_color_t* DonutDisplay::frame_buffer[2];
bool DonutDisplay::displayed_buffer;
Orientator* DonutDisplay::orientator;
esp_timer_handle_t DonutDisplay::displayTimer;
uint8_t DonutDisplay::col_index = 0;

DonutDisplay::scene_t scene = {
	.fps = 3,
	.objects = (DonutDisplay::scene_object_t[]){
		DonutDisplay::scene_object_t{
			.type = DonutDisplay::scene_object_t::TEXT,
			.pos = {
				.x = 15,
				.y = 1,
				.rotation = 0,
			},
			.color = HD107S_RGBL(255, 255, 0, 31),
			.data = (uint8_t*)"\1hello%world",
			.animations = (DonutDisplay::animation_t[]){
				DonutDisplay::animation_t{
					.animated_variable = DonutDisplay::animation_t::DATA,
			        .duration = 19,
			        .frames_active = 0,
			        .delay = 2,
			        .next_ptr = nullptr,
			        .initial_data = (uint8_t*)"\1hello%world",
			        .final_data = (uint8_t*)"\1HELLO WORLD",
				},
				// DonutDisplay::animation_t{
				// 	.animated_variable = DonutDisplay::animation_t::COLOR,
				// 	.duration = 10,
				// 	.frames_active = 0,
				// 	.delay = 0,
				// 	.next_ptr = nullptr,
				// 	.initial_data = HD107S_RGBL(255, 255, 0, 31),
				// },
				{DonutDisplay::animation_t::END}
			},
		},
		{DonutDisplay::scene_object_t::END}
	},
};

DonutDisplay::DonutDisplay(HD107S* LED, Orientator* orientator) {
	this->LED = LED;
	this->orientator = orientator;
	const size_t buf_size = sizeof(hd107s_color_t) * BUFFER_HEIGHT * FRAME_WIDTH;
	ESP_LOGI("DonutDisplay", "Frame buffer size: %zu", buf_size);
	frame_buffer[0] = (hd107s_color_t*)heap_caps_malloc(buf_size, MALLOC_CAP_DMA | MALLOC_CAP_32BIT);
    // memset(frame_buffer[0], 0, buf_size);
	frame_buffer[1] = (hd107s_color_t*)heap_caps_malloc(buf_size, MALLOC_CAP_DMA | MALLOC_CAP_32BIT);
    // memset(frame_buffer[1], 0, buf_size);
    clearFrameBuffer(0);
    clearFrameBuffer(1);
    esp_timer_create_args_t new_timer;
    new_timer.callback = &displayColumn;
    new_timer.dispatch_method = ESP_TIMER_TASK;
    esp_timer_create(&new_timer, &displayTimer);
}

DonutDisplay::~DonutDisplay() {
    free(frame_buffer[0]);
    free(frame_buffer[1]);
}

void IRAM_ATTR DonutDisplay::displayColumn(void* args) {
	if (orientator->getVelocity() < SPINNING_THRESHOLD)
		col_index = orientator->getHeading() * FRAME_WIDTH / ((1 << 16) - 1);
	else {
		if (col_index >= FRAME_WIDTH - 1)
			col_index = 0;
		else
			col_index++;
	}
	LED->update(frame_buffer[displayed_buffer] + BUFFER_HEIGHT * col_index);
}

void DonutDisplay::render() {
	uint8_t obj_index = 0;
	while (scene.objects[obj_index].type != scene_object_t::END) {
		scene_object_t* obj = scene.objects + obj_index;
		if (obj->color.lum == 0) continue;
		scene_object_t::position_t end_pos;
		switch (obj->type) {
			case scene_object_t::SPRITE:
				drawSprite(obj->pos.x, obj->pos.y, obj->pos.rotation, obj->data);
				break;
			case scene_object_t::TEXT:
				drawText(obj->pos.x, obj->pos.y, obj->pos.rotation, (const char*)obj->data, obj->color);
				break;
			case scene_object_t::LINE:
				end_pos = *((scene_object_t::position_t*)obj->data);
				drawLine(obj->pos.x, obj->pos.y, end_pos.x, end_pos.y, obj->color);
				break;
			case scene_object_t::END:
				break;
		}
		obj_index++; 
	}
	// swapBuffer();
}

void DonutDisplay::animate() {
	uint8_t obj_index = 0;
	while (scene.objects[obj_index].type != scene_object_t::END) {
		scene_object_t* obj = scene.objects + obj_index;
		uint8_t ani_index = 0;
		while (obj->animations[ani_index].animated_variable != animation_t::END) {
			animation_t* ani = obj->animations + ani_index;
			if (ani->frames_active >= ani->duration + ani->delay) continue; // skip if animation is over
			ani->frames_active++;

			// TODO: add auto set initial state
			// if (ani->frames_active == 0 && !ani->initial_data) {
			// }
			
			// TODO: test
			if (ani->frames_active <= ani->duration) {
				float progress = (float)(ani->frames_active) / ani->duration;
				ESP_LOGI("progress", "%f", progress);
				switch (ani->animated_variable) {
					case animation_t::DATA:
						switch (obj->type) {
							case scene_object_t::SPRITE:
								 for (uint8_t i = 0; i < obj->data[0] * obj->data[1]; i++) {
								 	obj->data[i] = ani->final_data[i] * progress + ani->initial_data[i] * (1-progress);
								 }
								break;
							case scene_object_t::TEXT:
								 for (uint8_t i = 0; obj->data[i]; i++) {
								 	obj->data[i] = (progress * (float)ani->final_data[i] + (1.f - progress) * (float)ani->initial_data[i]);
								 }
								break;
							case scene_object_t::LINE:
								((scene_object_t::position_t*)obj->data)->x = ((scene_object_t::position_t*)ani->final_data)->x * progress + ((scene_object_t::position_t*)ani->initial_data)->x * (1 - progress);
								((scene_object_t::position_t*)obj->data)->y = ((scene_object_t::position_t*)ani->final_data)->y * progress + ((scene_object_t::position_t*)ani->initial_data)->y * (1 - progress);
								break;
							case scene_object_t::END:
								break;
						}
						break;
					case animation_t::POS:
							obj->pos.x = ((scene_object_t::position_t*)ani->final_data)->x * progress + ((scene_object_t::position_t*)ani->initial_data)->x * (1 - progress);
							obj->pos.y = ((scene_object_t::position_t*)ani->final_data)->y * progress + ((scene_object_t::position_t*)ani->initial_data)->y * (1 - progress);
						break;
					case animation_t::COLOR:
							obj->color.r = ((hd107s_color_t*)ani->final_data)->r * progress + ((hd107s_color_t*)ani->initial_data)->r * (1 - progress);
							obj->color.g = ((hd107s_color_t*)ani->final_data)->g * progress + ((hd107s_color_t*)ani->initial_data)->g * (1 - progress);
							obj->color.b = ((hd107s_color_t*)ani->final_data)->b * progress + ((hd107s_color_t*)ani->initial_data)->b * (1 - progress);
							obj->color.lum = ((hd107s_color_t*)ani->final_data)->lum * progress + ((hd107s_color_t*)ani->initial_data)->lum * (1 - progress);
						break;
					case animation_t::END:
						break;
				}
			}
			
			if (ani->next_ptr && ani->frames_active == ani->duration + ani->delay) ani->next_ptr->frames_active = 0; // start next animation;
			ani_index++;
		}
		obj_index++; 
	}
}

void DonutDisplay::display(float angular_velocity, float angular_acceleration) {
	uint64_t timer_timeout = 6283185.f / FRAME_WIDTH / SPINNING_THRESHOLD;
	if (angular_velocity > SPINNING_THRESHOLD) {

		update_time = esp_timer_get_time();

		// TODO: remove magic values
		// Apply a PLL to keep the display and robot phase locked
		int16_t column_phase_offset = col_index - orientator->getHeading() * FRAME_WIDTH / ((1 << 16) - 1);
		if (column_phase_offset > FRAME_WIDTH/2) column_phase_offset -= FRAME_WIDTH; 
		if (column_phase_offset < -FRAME_WIDTH/2) column_phase_offset += FRAME_WIDTH; 
		PLL_bias = - 0.1*column_phase_offset; // sorta a PI controller PLL_bias*0.1
		
		timer_timeout = 6283185.f / FRAME_WIDTH / (angular_velocity + PLL_bias + 0.75*(angular_velocity - previous_velocity)); // this should use state acceleration but that is not correct currently
	}
	if (esp_timer_restart(displayTimer, timer_timeout) == ESP_ERR_INVALID_STATE) {
		esp_timer_start_periodic(displayTimer, timer_timeout);
	}
	previous_velocity = angular_velocity;
}

void DonutDisplay::clearFrameBuffer(bool buffer_num) {
    for (uint16_t i = 0; i < BUFFER_HEIGHT * FRAME_WIDTH; i++) {
    	if (i % BUFFER_HEIGHT == FRAME_HEIGHT) {
	    	frame_buffer[buffer_num][i] = HD107S_HEXL(0xffffff, 0b11111);
    	} else {
	    	frame_buffer[buffer_num][i] = HD107S_HEXL(0x0, 0b0);
	    }
    }
}

// https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
void DonutDisplay::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, hd107s_color_t color) {
	if (abs(y1 - y0) < abs(x1 - x0)) {
        if (x0 > x1) {
            drawLineLow(x1, y1, x0, y0, color);
        } else {
            drawLineLow(x0, y0, x1, y1, color);
        }
    } else {
        if (y0 > y1) {
            drawLineHigh(x1, y1, x0, y0, color);
        } else {
            drawLineHigh(x0, y0, x1, y1, color);
        }
    }
}

void DonutDisplay::drawLineHigh(int16_t x0, int16_t y0, int16_t x1, int16_t y1, hd107s_color_t color) {
	int16_t dx = x1 - x0;
    int16_t dy = y1 - y0;
    int16_t xi = 1;
    if (dx < 0) {
        xi = -1;
        dx = -dx;
    }
    int16_t D = (2 * dx) - dy;
    int16_t x = x0;

    for (int16_t y = y0; y <= y1; y++) {
        setPixel(x, y, color);
        if (D > 0) {
            x = x + xi;
            D = D + (2 * (dx - dy));
        } else {
            D = D + 2*dx;
        }
    }
}

void DonutDisplay::drawLineLow(int16_t x0, int16_t y0, int16_t x1, int16_t y1, hd107s_color_t color) {
	int16_t dx = x1 - x0;
    int16_t dy = y1 - y0;
    int16_t yi = 1;
    if (dy < 0) {
        yi = -1;
        dy = -dy;
    }
    int16_t D = (2 * dy) - dx;
    int16_t y = y0;

    for (int16_t x = x0; x <= x1; x++) {
        setPixel(x, y, color);
        if (D > 0) {
            y = y + yi;
            D = D + (2 * (dy - dx));
        } else {
            D = D + 2*dy;
        }
    }
}

void DonutDisplay::drawText(int16_t x, int8_t y, uint8_t rotation, const char* text, hd107s_color_t color) {
	// TODO: rotation
	const uint8_t* font_ptr = font_array[text[0] - 1];
	const uint8_t font_width = font_size_array[text[0] - 1][0];
	const uint8_t font_height = font_size_array[text[0] - 1][1];
	const uint8_t bytes_pre_char = font_width * ((font_height + 0b111) >> 3); // width * ceil(height/8)
	const int16_t x0 = x;
	for (uint8_t char_index = 1; text[char_index] != 0; char_index++) {
		if (text[char_index] == '\n') {
			y += font_height + 1;
			x = x0;
			continue;
		}
		for (uint8_t i = 0; i < font_width; i++) {
			for (uint8_t j = 0; j < font_height; j++) {
				if ((font_ptr[(text[char_index] - ' ') * bytes_pre_char + i] >> j) & 1)
					setPixel(x+i, y+j, color); 
			}
		}
		x += font_width + 1;
	}
}

void DonutDisplay::drawSprite(int16_t x, int8_t y, uint8_t rotation, uint8_t* data) {
	uint8_t width = data[0];
	uint8_t height = data[1];
	// TODO: rotation
	for (uint8_t i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
        	uint16_t index = i + j*width;
        	if (data[index] == 0xff) continue;
			setPixel(x+i, y+j, palette[data[index]]);
        }		
	}
}

void DonutDisplay::setPixel(int16_t x, int8_t y, hd107s_color_t color) {
	frame_buffer[!displayed_buffer][tinyMod(x, FRAME_WIDTH)*BUFFER_HEIGHT + tinyMod(FRAME_HEIGHT - 1 - y, FRAME_HEIGHT)] = color;
}

void DonutDisplay::swapBuffer() {
	// TODO: seme take
	displayed_buffer = !displayed_buffer;
}

void DonutDisplay::printFrameBuffer() {
    for (int j = BUFFER_HEIGHT - 1; j >= 0 ; j--) {
        for (int i = 0; i < FRAME_WIDTH; i++) {
            hd107s_color_t test_color = frame_buffer[!displayed_buffer][j + BUFFER_HEIGHT * i];
			printf("\033[48;2;%u;%u;%um ", test_color.r, test_color.g, test_color.b);
        }
        printf("\n");
    }
}

uint16_t DonutDisplay::tinyMod(int16_t value, uint8_t mod) {
	if (value < 0) {
		return value + mod;
	}
	if (value >= mod) {
		return value - mod;
	}
	return value;
}
