#include "orientator.h"
#include "H3LIS331DL.h"
#include "MMC5983MA.h"
#include "MathUtils.h"
#include "donutPhysics.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "kalmanFilter.h"
#include "nvs.h"
#include "portmacro.h"
#include "sensor.h"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

esp_timer_handle_t Orientator::zeroHeadingTimer;
void (* Orientator::zeroCrossCallback)() = emptyFunction;
void (* Orientator::onStopCallback)() = emptyFunction;

using namespace std;

Orientator::Orientator() {
    physics = new DonutPhysics();
}

Orientator::Orientator(DonutPhysics* physics) {
    this->physics = physics;
}

Orientator::~Orientator() {
}

void Orientator::update() {
    // get sensor inputs
    // TODO: use interupt SPI to prevent busy waiting
    if (!accel_r->getXYZ(accel_val_1)) accel_val_1 = {0,0,0}; // zero if read fails
    if (!accel_l->getXYZ(accel_val_2)) accel_val_2 = {0,0,0}; // TODO: do something else if read fails
    if (!mag->getXYZ(vec_m)) vec_m = {0,0,0};
    accel_avg_1 = accel_avg_1 + ((vec3<float>)accel_val_1 - accel_avg_1)/ROLLING_AVERAGE_SIZE; // rolling average of accel
    accel_avg_2 = accel_avg_2 + ((vec3<float>)accel_val_2 - accel_avg_2)/ROLLING_AVERAGE_SIZE;    

    // process sensor values into usable data
    CoR = state.angular_velocity > SPINNING_THRESHOLD ? getCenterOfRotation(accel_avg_2, accel_avg_1) : vec2<float>{0,0};
    measured_state.angle = getMagHeading();
    measured_state.angular_velocity = getAccelVelocity();
    measured_state.varience_angle = mag_varience;
    measured_state.varience_velocity = SENSOR_XL_VARIENCE_SCALAR * 1/(measured_state.angular_velocity) * accel_varience;
    measured_state.time = 0; // fix telometer crash

    // step physics and state
    physics->step(state);
    kalmanFilter::stateUpdate(state, measured_state);
    // ESP_LOGI("mag", "m %03x, %03x, %03x, time: %lld", vec_m.x, vec_m.y, vec_m.z, state.time);

    // calibration
    if (accel_calibration) {
        accel_r->offset = accel_r->offset + ((vec3<float>)((vec3<float>)accel_val_1 - vec3<float>{0,0,(int16_t)(1/LSB_TO_G)} )/ROLLING_AVERAGE_SIZE);
        accel_l->offset = accel_l->offset + ((vec3<float>)((vec3<float>)accel_val_2 - vec3<float>{0,0,(int16_t)(1/LSB_TO_G)} )/ROLLING_AVERAGE_SIZE);
    }
    if (mag_calibration && state.angular_velocity > SPINNING_THRESHOLD) {
        mag->offset = mag->offset + (vec3<float>)vec_m/(ROLLING_AVERAGE_SIZE);
    }

    // TODO: start timers
    if (state.angular_velocity > SPINNING_THRESHOLD && state.angle > 32000 && state.angle < 62000) { 
        // float rotationPeriod = (float)(1000000*2*M_PI)/state.angular_velocity;
        // zeroCrossingTime = esp_timer_get_time() - (float)state.angle*LSB2ROT*rotationPeriod;
        // int64_t startDelay = zeroCrossingTime - esp_timer_get_time();
        // if (startDelay < 0) startDelay = (startDelay % (int)(abs(rotationPeriod))) + (int)(abs(rotationPeriod));
        esp_timer_stop(zeroHeadingTimer);
        int64_t startDelay = 1000000*(6.28 - state.angle * LSB2RAD)/state.angular_velocity;
        esp_timer_start_once(zeroHeadingTimer, startDelay);
    } else {
        // onStopCallback();
        // rotationPeriod = 0;
        // zeroCrossingTime = esp_timer_get_time();
    }
}

void Orientator::zeroHeadingCallback(void *args) {
    zeroCrossCallback();
}

void Orientator::stopZeroCrossCallback() {
    esp_timer_stop(zeroHeadingTimer);
}

void Orientator::setZeroCrossCallback(void (* callback)()) {
    zeroCrossCallback = callback;
}

void Orientator::setOnStopCallback(void (* callback)()) {
    onStopCallback = callback;
}

void Orientator::adjustAngle(float angle) {
    state.angle += angle;
}

void Orientator::adjustVelocity(float velocity) {
    state.angular_velocity += velocity;
}

void Orientator::adjustAccel(float accel) {
    state.angular_acceleration += accel;
}

// calculates the point that the robot is spinning around by finding the intersection point of both acceleration vectors
vec2<float> Orientator::getCenterOfRotation(vec3<float> XL_left, vec3<float> XL_right) {
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
float Orientator::getVelocity() {
    return state.angular_velocity > SPINNING_THRESHOLD ? state.angular_velocity : 0; // might change this behavior later
}

// returns the current heading in LSBs using linear approximation from last state
uint16_t Orientator::getHeading() {
    return state.angle + (RAD2LSB*(float)(esp_timer_get_time() - state.time) * state.angular_velocity * 0.000001f);
}

void Orientator::readConfig() {
    nvs_handle_t my_handle;
    nvs_open("config", NVS_READWRITE, &my_handle);
    size_t read_size = sizeof(vec3<float>);
    nvs_get_blob(my_handle, "XL1_offset", &accel_r->offset, &read_size);
    nvs_get_blob(my_handle, "XL2_offset", &accel_l->offset, &read_size);
    nvs_get_blob(my_handle, "mag_offset", &mag->offset, &read_size);
    ESP_LOGI("NVS", "Read offsets XL1: %0.2f, %0.2f, %0.2f, XL2: %0.2f, %0.2f, %0.2f, Mag: %0.2f, %0.2f, %0.2f", accel_r->offset.x, accel_r->offset.y, accel_r->offset.z, accel_l->offset.x, accel_l->offset.y, accel_l->offset.z, mag->offset.x, mag->offset.y, mag->offset.z);
    nvs_close(my_handle);
}

void Orientator::writeConfig() {
    nvs_handle_t my_handle;
    nvs_open("config", NVS_READWRITE, &my_handle);
    size_t read_size = sizeof(vec3<float>);
    nvs_set_blob(my_handle, "XL1_offset", &(accel_r->offset), read_size);
    nvs_set_blob(my_handle, "XL2_offset", &(accel_l->offset), read_size);
    nvs_set_blob(my_handle, "mag_offset", &(mag->offset), read_size);
    esp_err_t err = nvs_commit(my_handle);
    if (err != ESP_OK) ESP_LOGI("NVS", "err %s", esp_err_to_name(err));
    nvs_close(my_handle);
}

void Orientator::setup(Sensor* accel_r, Sensor* accel_l, Sensor* mag) {
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
        .varience_angle = 999, // we have no idea which direction we are facing when powered on
        .varience_velocity = 0,
        .varience_acceleration = 0,
        .time = (uint64_t)esp_timer_get_time(),
    };
}

void Orientator::enableAccelCalibration(bool enable) {
    accel_calibration = enable;
    // TODO: implement
}

void Orientator::enableMagCalibration(bool enable) {
    mag_calibration = enable;
    // TODO: implement
}

float Orientator::getAccelVelocity() {
    
    float left_velocity = sqrt(MathUtils::fastLength((vec2<int16_t>)accel_val_2)*LSB_TO_MPS2/(MathUtils::fastLength(accel_l->pos - CoR))); // radians per second
    float right_velocity = sqrt(MathUtils::fastLength((vec2<int16_t>)accel_val_1)*LSB_TO_MPS2/(MathUtils::fastLength(accel_r->pos - CoR))); // radians per second

    return (left_velocity + right_velocity)/2;
}

uint16_t Orientator::getMagHeading() {
    return RAD2LSB*atan2(vec_m.y, vec_m.x);
}