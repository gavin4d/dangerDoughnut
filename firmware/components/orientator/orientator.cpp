#include "orientator.h"
#include "H3LIS331DL.h"
#include "MMC5983MA.h"
#include "MathUtils.h"
#include "donutPhysics.h"
#include "driver/gpio.h"
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
    if (!accel_1->getXYZ(accel_val_1)) accel_val_1 = {0,0,0}; // zero if read fails
    if (!accel_2->getXYZ(accel_val_2)) accel_val_2 = {0,0,0}; // TODO: do something else if read fails
    if (!mag->getXYZ(vec_m)) vec_m = {0,0,0};
    accel_avg_1 = accel_avg_1 + ((vec3<float>)accel_val_1 - accel_avg_1)/ROLLING_AVERAGE_SIZE; // rolling average of accel
    accel_avg_2 = accel_avg_2 + ((vec3<float>)accel_val_2 - accel_avg_2)/ROLLING_AVERAGE_SIZE;    

    // process sensor values into usable data
    CoR = state.angular_velocity > SPINNING_THRESHOLD ? getCenterOfRotation((vec2<float>)accel_avg_2, (vec2<float>)accel_avg_1) : vec2<float>{0,0};
    accel_1_dist = MathUtils::fastLength(accel_1->pos - CoR);
    accel_2_dist = MathUtils::fastLength(accel_2->pos - CoR);
    measured_state.angle = getMagHeading();
    float raw_velocity = 0, angular_accel = 0;
    getAccelVelocity(angular_accel, raw_velocity);
    measured_state.angular_velocity = raw_velocity - getVelocityCalCurve(raw_velocity);
    measured_state.angular_acceleration = angular_accel;
    measured_state.varience_angle = mag_varience;
    measured_state.varience_velocity = SENSOR_XL_VARIENCE_SCALAR * 1/(abs(measured_state.angular_velocity)+0.001) * accel_varience;
    measured_state.varience_acceleration = accel_varience/pow(accel_1_dist,2);
    measured_state.time = 0; // fixes telometer crash

    uint16_t prev_angle = state.angle;
    // step physics and state
    physics->step(state);
    kalmanFilter::stateUpdate(state, measured_state);
    // ESP_LOGI("mag", "m %03x, %03x, %03x, time: %lld", vec_m.x, vec_m.y, vec_m.z, state.time);

    // self calibration
    setVelocityCalCurve(raw_velocity, ((float)((int16_t)(state.angle - prev_angle)))/state.dt);
    if (accel_calibration) {
        accel_1->offset = accel_1->offset + ((vec3<float>)((vec3<float>)accel_val_1 - vec3<float>{0,0,(int16_t)(1/LSB_TO_G)} )/ROLLING_AVERAGE_SIZE);
        accel_2->offset = accel_2->offset + ((vec3<float>)((vec3<float>)accel_val_2 - vec3<float>{0,0,(int16_t)(1/LSB_TO_G)} )/ROLLING_AVERAGE_SIZE);
    }
    if (mag_calibration && state.angular_velocity > SPINNING_THRESHOLD) {
        mag->offset = mag->offset + (vec3<float>)vec_m/(ROLLING_AVERAGE_SIZE);
    }

}

// calculates the point that the robot is spinning around by finding the intersection point of both acceleration vectors
vec2<float> Orientator::getCenterOfRotation(vec2<float> XL_2, vec2<float> XL_1) {
    XL_2 = MathUtils::rotate(-1*XL_2, accel_2->angle) + accel_2->pos;
    XL_1 = MathUtils::rotate(-1*XL_1, accel_1->angle) + accel_1->pos;
    float t = ((accel_2->pos.x - accel_1->pos.x)*XL_1.y)/(XL_1.x * XL_2.y - XL_2.x * XL_1.y);
    return XL_2*t + accel_2->pos;
}

// returns the current angular velocity in rad/s
float Orientator::getVelocity() {
    return state.angular_velocity;
}

// returns the current heading in LSBs using linear approximation from last state
uint16_t Orientator::getHeading() {
    return state.angle + (RAD2LSB*(float)(esp_timer_get_time() - state.time) * state.angular_velocity * 0.000001f) + angle_offset;
}

void Orientator::readConfig() {
    nvs_handle_t my_handle;
    nvs_open("config", NVS_READWRITE, &my_handle);
    size_t read_size = sizeof(vec3<float>);
    nvs_get_blob(my_handle, "XL1_offset", &accel_1->offset, &read_size);
    nvs_get_blob(my_handle, "XL2_offset", &accel_2->offset, &read_size);
    nvs_get_blob(my_handle, "mag_offset", &mag->offset, &read_size);
    read_size = sizeof(velocity_cal_curve);
    esp_err_t err = nvs_get_blob(my_handle, "vel_cal_curve", &velocity_cal_curve, &read_size);
    if (err != ESP_OK) ESP_LOGE("NVS", "err %s", esp_err_to_name(err));
    ESP_LOGI("NVS", "Read offsets XL1: %0.2f, %0.2f, %0.2f, XL2: %0.2f, %0.2f, %0.2f, Mag: %0.2f, %0.2f, %0.2f", accel_1->offset.x, accel_1->offset.y, accel_1->offset.z, accel_2->offset.x, accel_2->offset.y, accel_2->offset.z, mag->offset.x, mag->offset.y, mag->offset.z);
    ESP_LOGI("NVS", "Read %u bytes of velocity calibration curve:", read_size);
    for (int i = 0; i < CAL_CURVE_SIZE-1; i++) {
        printf("%0.3f, ", velocity_cal_curve[i]);
    }
    printf("%0.3f\n", velocity_cal_curve[CAL_CURVE_SIZE-1]);
    nvs_close(my_handle);
}

void Orientator::writeConfig() {
    nvs_handle_t my_handle;
    nvs_open("config", NVS_READWRITE, &my_handle);
    size_t read_size = sizeof(vec3<float>);
    nvs_set_blob(my_handle, "XL1_offset", &(accel_1->offset), read_size);
    nvs_set_blob(my_handle, "XL2_offset", &(accel_2->offset), read_size);
    nvs_set_blob(my_handle, "mag_offset", &(mag->offset), read_size);
    read_size = sizeof(velocity_cal_curve);
    esp_err_t err = nvs_set_blob(my_handle, "vel_cal_curve", &velocity_cal_curve, read_size);
    if (err != ESP_OK) ESP_LOGE("NVS", "err %s", esp_err_to_name(err));
    ESP_LOGI("NVS", "Wrote %u bytes of velocity calibration curve:", read_size);
    for (int i = 0; i < CAL_CURVE_SIZE-1; i++) {
        printf("%0.3f, ", velocity_cal_curve[i]);
    }
    printf("%0.3f\n", velocity_cal_curve[CAL_CURVE_SIZE-1]);
    err = nvs_commit(my_handle);
    if (err != ESP_OK) ESP_LOGE("NVS", "err %s", esp_err_to_name(err));
    nvs_close(my_handle);
}

void Orientator::setup(Sensor* accel_1, Sensor* accel_2, Sensor* mag) {
    this->accel_2 = accel_2;
    this->accel_1 = accel_1;
    this->mag = mag;

    state = system_state_t{
        .angle = 0,
        .angular_velocity = 0,
        .angular_acceleration = 0,
        .varience_angle = 999999, // we have no idea which direction we are facing when powered on
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

void Orientator::getAccelVelocity(float &angular_accel, float &velocity) {
    
    float normal_accel_1 = -MathUtils::dot(MathUtils::rotate((vec2<float>)accel_val_1, accel_1->angle), accel_1->pos - CoR)*LSB_TO_MPS2/(accel_1_dist); // radians per second
    float normal_accel_2 = -MathUtils::dot(MathUtils::rotate((vec2<float>)accel_val_2, accel_2->angle), accel_2->pos - CoR)*LSB_TO_MPS2/(accel_2_dist); // radians per second
    float tangent_accel_1 = -MathUtils::dot(MathUtils::rotate((vec2<float>)accel_val_1, accel_1->angle + MathUtils::angleFromDegrees(90)), accel_1->pos - CoR)*LSB_TO_MPS2/(accel_1_dist); // radians per second
    float tangent_accel_2 = -MathUtils::dot(MathUtils::rotate((vec2<float>)accel_val_2, accel_2->angle + MathUtils::angleFromDegrees(90)), accel_2->pos - CoR)*LSB_TO_MPS2/(accel_2_dist); // radians per second
    angular_accel = (tangent_accel_1 + tangent_accel_2)/(2 * accel_2_dist); // positive is accelerating clockwise
    velocity = (sqrt(abs(normal_accel_1/(accel_1_dist)) + sqrt(abs(normal_accel_2/(accel_2_dist))))/2);

    // float left_velocity = sqrt(MathUtils::fastLength((vec2<int16_t>)accel_val_2)*LSB_TO_MPS2/(accel_2_dist)); // radians per second
    // float right_velocity = sqrt(MathUtils::fastLength((vec2<int16_t>)accel_val_1)*LSB_TO_MPS2/(accel_1_dist); // radians per second

}

void Orientator::setVelocityCalCurve(float raw_velocity, float angle_derivative) {
    uint16_t cal_index = (uint16_t)(raw_velocity*CAL_CURVE_SIZE/MAX_VELOCITY);
    if (cal_index >= CAL_CURVE_SIZE) cal_index = CAL_CURVE_SIZE - 1;
    float error = (raw_velocity - (angle_derivative*LSB2RAD));
    if (error < SPINNING_THRESHOLD) // don't update if angle is stuck
        velocity_cal_curve[cal_index] += (error - velocity_cal_curve[cal_index])/ROLLING_AVERAGE_SIZE;
}

float Orientator::getVelocityCalCurve(float raw_velocity) {
    uint16_t cal_index = (uint16_t)(raw_velocity*CAL_CURVE_SIZE/MAX_VELOCITY);
    if (cal_index >= CAL_CURVE_SIZE) cal_index = CAL_CURVE_SIZE - 1;
    return velocity_cal_curve[cal_index];
}

uint16_t Orientator::getMagHeading() {
    static uint16_t last_output = 0;
    uint16_t output = RAD2LSB*atan2(vec_m.y, vec_m.x);
    if (output == last_output) { // estimate if magnitometer misbehaves 
        output += state.angular_velocity * state.dt * RAD2LSB;
        // printf("%d\n", output);
    }
    last_output = output;
    return output;
}