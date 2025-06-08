#include "orientator.h"
#include "H3LIS331DL.h"
#include "MMC5983MA.h"
#include "MathUtils.h"
#include "donutPhysics.h"
#include "esp_timer.h"
#include "kalmanFilter.h"
#include "portmacro.h"
#include "sensor.h"
#include <cmath>
#include <cstdint>
#include <cstdio>

esp_timer_handle_t orientator::zeroHeadingTimer;
void (* orientator::zeroCrossCallback)() = emptyFunction;
void (* orientator::onStopCallback)() = emptyFunction;

using namespace std;

orientator::orientator() {
}

orientator::~orientator() {
}

void orientator::update() {
    // get sensor inputs
    if (!accel_r->getXYZ(accel_val_1)) accel_val_1 = {0,0,0}; // zero if read fails
    if (!accel_l->getXYZ(accel_val_2)) accel_val_2 = {0,0,0}; // TODO: do something else if read fails
    if (!mag->getXYZ(vec_m)) vec_m = {0,0,0};
    accel_avg_1 = accel_avg_1 + ((vec3<float>)accel_val_1 - accel_avg_1)/ROLLING_AVERAGE_SIZE; // rolling average of accel
    accel_avg_2 = accel_avg_2 + ((vec3<float>)accel_val_2 - accel_avg_2)/ROLLING_AVERAGE_SIZE;    

    // process sensor values into usable data
    CoR = state.angular_velocity > SPINNING_THRESHOLD ? getCenterOfRotation(accel_avg_2, accel_avg_1) : vec2<float>{0,0};
    system_state_t measured_state = {
        .angle = getMagHeading(),
        .angular_velocity = getAccelVelocity(),
        .variance_angle = SENSOR_MAG_VARIENCE,
        .variance_velocity = SENSOR_XL_VARIENCE,
    };

    // step physics and state
    physics->step(state);
    kalmanFilter::stateUpdate(state, measured_state);

    // TODO: start timers
    // esp_timer_stop(zeroHeadingTimer);
    // if (currentState.angular_velocity > 13) {
    //     rotationPeriod = (double)(1000*2*M_PI)/currentState.angular_velocity;
    //     zeroCrossingTime = esp_timer_get_time() - (double)currentState.angle*LSB2ROT*rotationPeriod*RESOLUTION;
    //     int64_t startDelay = zeroCrossingTime - esp_timer_get_time() - (int)(offset*rotationPeriod*RESOLUTION);
    //     if (startDelay < 0) startDelay = (startDelay % (int)(abs(rotationPeriod*RESOLUTION))) + (int)(abs(rotationPeriod*RESOLUTION));
    //     esp_timer_start_once(initTimer, startDelay);
    // } else {
    //     // onStopCallback();
    //     rotationPeriod = 0;
    //     zeroCrossingTime = esp_timer_get_time();
    // }
}

void orientator::zeroHeadingCallback(void *args) {
    zeroCrossCallback();
}

void orientator::stopZeroCrossCallback() {
    esp_timer_stop(zeroHeadingTimer);
}

void orientator::setZeroCrossCallback(void (* callback)()) {
    zeroCrossCallback = callback;
}

void orientator::setOnStopCallback(void (* callback)()) {
    onStopCallback = callback;
}

void orientator::adjustAngle(float angle) {
    state.angle += angle;
}

void orientator::adjustVelocity(float velocity) {
    state.angular_velocity += velocity;
}

void orientator::adjustAccel(float accel) {
    state.angular_acceleration += accel;
}

// calculates the point that the robot is spinning around by finding the intersection point of both acceleration vectors
vec2<float> orientator::getCenterOfRotation(vec3<float> XL_left, vec3<float> XL_right) {
    vec2<float> XL_l, XL_r;
    XL_l.x = -XL_left.x; 
    XL_l.y = -XL_left.y; 
    XL_r.x = -XL_right.x; 
    XL_r.y = -XL_right.y; 
    XL_l = MathUtils::rotate(XL_l, accel_l->angle);
    XL_l = XL_l + accel_l->pos;
    XL_r = MathUtils::rotate(XL_r, accel_r->angle);
    XL_r = XL_r + accel_r->pos;
    float t = ((accel_l->pos.x - accel_r->pos.x)*XL_r.y)/(XL_r.x * XL_l.y - XL_l.x * XL_r.y);
    return XL_l*t + accel_l->pos;
}

// returns the current angular velocity in rad/s
float orientator::getVelocity() {
    return state.angular_velocity > SPINNING_THRESHOLD ? state.angular_velocity : 0; // might change this behavior later
}

// returns the current heading in LSBs using linear approximation from last state
uint16_t orientator::getHeading() {
    return state.angle + (RAD2LSB*(esp_timer_get_time() - state.time) * state.angular_velocity)/1000000;
    
}

void orientator::setup(Sensor* accel_r, Sensor* accel_l, Sensor* mag) {
    this->accel_l = accel_l;
    this->accel_r = accel_r;
    this->mag = mag;

    esp_timer_create_args_t new_timer;
    new_timer.callback = &zeroHeadingCallback;
    new_timer.dispatch_method = ESP_TIMER_TASK;
    esp_timer_create(&new_timer, &zeroHeadingTimer);

    state = system_state_t{
        .angle = 0,
        .angular_velocity = 0,
        .angular_acceleration = 0,
        .variance_angle = 999, // we have no idea which direction we are facing when powered on
        .variance_velocity = 0,
        .variance_acceleration = 0,
        .time = (uint64_t)esp_timer_get_time(),
    };
}


void orientator::enableAccelCalibration(bool enable) {
    // TODO: implement
}

void orientator::enableMagCalibration(bool enable) {
    // TODO: implement
}

float orientator::getAccelVelocity() {
    
    float left_velocity = sqrt(MathUtils::fastLength((vec2<int16_t>)accel_val_2)*LSB_TO_MPS2/(MathUtils::fastLength(accel_l->pos - CoR))); // radians per second
    float right_velocity = sqrt(MathUtils::fastLength((vec2<int16_t>)accel_val_1)*LSB_TO_MPS2/(MathUtils::fastLength(accel_r->pos - CoR))); // radians per second

    return (left_velocity + right_velocity)/2;
}

uint16_t orientator::getMagHeading() {
    return RAD2LSB*atan2(vec_m.y, vec_m.x);
}