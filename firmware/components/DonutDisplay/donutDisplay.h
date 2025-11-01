#pragma once
#include "HD107S.h"
#include "fontStore.h"
#include "orientator.h"
#include "esp_timer.h"
#include <cstdint>
#include <stdint.h>
#include <stddef.h>

#define FRAME_HEIGHT 20
#define FRAME_WIDTH 157
#define BUFFER_HEIGHT (FRAME_HEIGHT + 1)
#define LUM 31

class DonutDisplay {

public:
    struct animation_t {
        enum animated_variable_t {
            DATA,
            POS,
            COLOR,
            END, // signals the end of the animation list
        } animated_variable; // object variable to animate
        uint16_t duration; // number of frames the animation should last
        uint16_t frames_active; // number of frames since animation start
        uint16_t delay; // number of frames to wait after animation before starting next animation
        animation_t* next_ptr; // animation at this pointer will be restarted after duration and delay
        uint8_t* initial_data; // initial state of the selected vaiable. Set to nullptr to use current state
        uint8_t* final_data; // final state of the selected variable
    };
    
    struct scene_object_t {
        enum object_type_t {
            SPRITE,
            TEXT,
            LINE,
            END, // signals the end of the scene object list
        } type; // type of object to render
        // uint8_t scale; // scale of each spite pixel or width of line 
        struct position_t {
            int16_t x;
            int8_t y;
            int8_t rotation; // rotation in 90 degree increments. 0: upright, 1: rotated left 90 degrees ...
        } pos; // position of the object in the frame buffer. Negative numbers will wrap
        hd107s_color_t color; // color of the text of line. Brightness must be greater than zero to render for all object types
        // data for the selected object type
        // SPRITE: width, height, 1 byte/px indexed color
        // TEXT: 1 indexed font, char array
        // LINE: position_t cast into uint8_t
        uint8_t* data;
        animation_t* animations; // null terminated list of animations
    };

    struct scene_t {
        uint8_t fps; // frames per second to render
        scene_object_t* objects; // null terminated list of objects in the scene
    };

    DonutDisplay(HD107S* LED, Orientator* orientator);
    DonutDisplay() = delete;
    ~DonutDisplay();

    void setScene(scene_t* scene_ptr);
    void animate();
    void render();
    void clearFrameBuffer(bool frame_buffer);
    void startAnimationTimer();
    void stopAnimationTimer();

    void display(float angular_velocity, float angular_acceleration);
    void swapBuffer();
    
    void setPixel(int16_t x, int8_t y, hd107s_color_t color);
    void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, hd107s_color_t color);
    void drawText(int16_t x, int8_t y, uint8_t rotation, const char* text, hd107s_color_t color);
    void drawSprite(int16_t x, int8_t y, uint8_t rotation, uint8_t* data);

    void printFrameBuffer();

    static hd107s_color_t* frame_buffer[2];
    static bool displayed_buffer;
    float PLL_bias;
private:
    static Orientator* orientator;
    static uint8_t col_index;
    static HD107S* LED;
    uint64_t update_time;
    float previous_velocity = 0;
    static esp_timer_handle_t displayTimer;
    static esp_timer_handle_t renderTimer;

    static void displayColumn(void * args);
    void drawLineHigh(int16_t x0, int16_t y0, int16_t x1, int16_t y1, hd107s_color_t color);
    void drawLineLow(int16_t x0, int16_t y0, int16_t x1, int16_t y1, hd107s_color_t color);
    uint16_t tinyMod(int16_t value, uint8_t mod);
    
    // palette source: https://lospec.com/palette-list/axulart-32-color-palette
    constexpr static const hd107s_color_t palette[] = {
        HD107S_HEXL(0x000000, LUM),
        HD107S_HEXL(0xfcfcfc, LUM),
        HD107S_HEXL(0xc4c7ee, LUM),
        HD107S_HEXL(0x9a8fe0, LUM),
        HD107S_HEXL(0x635d96, LUM),
        HD107S_HEXL(0x292f65, LUM),
        HD107S_HEXL(0x1b1d34, LUM),
        HD107S_HEXL(0xffe3ae, LUM),
        HD107S_HEXL(0xcdbbab, LUM),
        HD107S_HEXL(0xa6858f, LUM),
        HD107S_HEXL(0xcf5d8b, LUM),
        HD107S_HEXL(0x964968, LUM),
        HD107S_HEXL(0xffb482, LUM),
        HD107S_HEXL(0xdd867d, LUM),
        HD107S_HEXL(0xb2696f, LUM),
        HD107S_HEXL(0xf6c65e, LUM),
        HD107S_HEXL(0xe49057, LUM),
        HD107S_HEXL(0xc46833, LUM),
        HD107S_HEXL(0xb0d07e, LUM),
        HD107S_HEXL(0x66aa5d, LUM),
        HD107S_HEXL(0x52b5ab, LUM),
        HD107S_HEXL(0x2a8379, LUM),
        HD107S_HEXL(0x1c5659, LUM),
        HD107S_HEXL(0x7be1f6, LUM),
        HD107S_HEXL(0x589ffc, LUM),
        HD107S_HEXL(0x5069e4, LUM),
        HD107S_HEXL(0x2e44ae, LUM),
        HD107S_HEXL(0x8056d4, LUM),
        HD107S_HEXL(0x5a3b96, LUM),
        HD107S_HEXL(0xffbae1, LUM),
        HD107S_HEXL(0xe687c5, LUM),
        HD107S_HEXL(0xa759b9, LUM)
    };
};

// two frame buffers
// switch buffers to render next frame
// rerender on object change
// switch as soon as frame ready
// step animation on animation ticks
