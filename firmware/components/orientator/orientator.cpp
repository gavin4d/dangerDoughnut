#include "orientator.h"
#include "H3LIS331DL.h"
#include "MMC5983MA.h"
#include "MathUtils.h"
#include "esp_timer.h"
#include <cmath>
#include <cstdint>
#include <iostream>

#define RESOLUTION 1000 // microseconds

vec2<float> XL_l_pos = {-0.015038, -0.03848};
angle XL_l_angle = MathUtils::angleFromDegrees(23.654);
vec2<float> XL_r_pos = {0.015038, -0.03848};
angle XL_r_angle = MathUtils::angleFromDegrees(66.347);

esp_timer_handle_t orientator::zeroHeadingTimer;
void (* orientator::zeroCrossCallback)() = nullptr;
void (* orientator::onStopCallback)() = nullptr;
double orientator::rotationPeriod = 0;

using namespace std;

orientator::orientator() {
}

orientator::~orientator() {
}

void orientator::initCallback(void *args) {
    if (rotationPeriod < 10 || rotationPeriod >= 500) return;
    zeroHeadingCallback(args);
    esp_timer_start_periodic(zeroHeadingTimer, rotationPeriod*RESOLUTION);
}

void orientator::zeroHeadingCallback(void *args) {
    if (zeroCrossCallback != nullptr) zeroCrossCallback();
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

double orientator::getXSign() {
    // return copysign(1, -xAccel);
    return 0;
}

double orientator::getXAccel() {
    // return -xAccel*LSB_TO_G;
    return 0;
}

double orientator::getYSign() {
    // return copysign(1, yAccel);
    return 0;
}

double orientator::getYAccel() {
    // return yAccel*LSB_TO_G;
    return 0;
}

double orientator::getZSign() {
    // return copysign(1, -zAccel);
    return 0;
}

double orientator::getZAccel() {
    // return -zAccel*LSB_TO_G;
    return 0;
}

void orientator::setOffset(double offset) {
    orientator::offset = offset-floor(offset);
}

double orientator::getOffset() {
    return offset;
}

void orientator::setAccelPos(double accelPos) {
    // orientator::accelPos[min((int)round(angularVelocity/ACCEL_POS_SPREAD),NUM_ACCEL_POS-1)] = accelPos;
}

void orientator::setAccelPos(double accelPos, int index) {
    orientator::accelPos[index] = accelPos;
}

void orientator::adjustAngle(double angle) {
    filter.adjustAngle(angle);
}

void orientator::adjustVelocity(double velocity) {
    filter.adjustVelocity(velocity);
}

void orientator::adjustAccel(double accel) {
    filter.adjustAccel(accel);
}

double orientator::getAccelPos() {
    // return accelPos[min((int)round(angularVelocity/ACCEL_POS_SPREAD), NUM_ACCEL_POS-1)];
    return 0;
}

double orientator::getAccelPos(int index) {
    return accelPos[index];
}

vec2<float> orientator::getCenterOfRotation(vec3<float> XL_left, vec3<float> XL_right) {
    vec2<float> XL_l, XL_r;
    XL_l.x = -XL_left.x; 
    XL_l.y = -XL_left.y; 
    XL_r.x = -XL_right.x; 
    XL_r.y = -XL_right.y; 
    XL_l = MathUtils::rotate(XL_l, XL_l_angle);
    XL_l = XL_l + XL_l_pos;
    XL_r = MathUtils::rotate(XL_r, XL_r_angle);
    XL_r = XL_r + XL_r_pos;
    float t = ((XL_l_pos.x - XL_r_pos.x)*XL_r.y)/(XL_r.x * XL_l.y - XL_l.x * XL_r.y);
    return XL_l*t + XL_l_pos;
}

double orientator::getPeriod() {
    return rotationPeriod;
}

double orientator::getVelocity() {
    if (angularVelocity > 13)
        return angularVelocity;
    else
        return 0;
}

// returns current heading
double orientator::getAngle() {
    if (rotationPeriod == 0) return 0;
    uint32_t oneRotationTime = rotationPeriod*RESOLUTION;
    uint64_t timeSinceZero = (esp_timer_get_time() - zeroCrossingTime + (int)(offset*oneRotationTime));
    return (double)(timeSinceZero % oneRotationTime)*2*M_PI/oneRotationTime;
}

// returns heading
double orientator::getAngle(uint64_t zeroCrossingTime) {
    if (rotationPeriod == 0) return 0;
    uint32_t oneRotationTime = rotationPeriod*RESOLUTION;
    uint64_t timeSinceZero = (esp_timer_get_time() - zeroCrossingTime + (int)(offset*oneRotationTime));
    return 2*M_PI*(double)(timeSinceZero % oneRotationTime)/oneRotationTime;
}

void orientator::setup(H3LIS331DL accel) {
    orientator::accel_l = accel;

    esp_timer_create_args_t new_timer;
    // new_timer.callback = &checkIRCallback;
    // new_timer.dispatch_method = ESP_TIMER_TASK;
    // esp_timer_create(&new_timer, &update_timer);
    // esp_timer_start_periodic(update_timer, RESOLUTION);

    new_timer.callback = &initCallback;
    new_timer.dispatch_method = ESP_TIMER_TASK;
    esp_timer_create(&new_timer, &initTimer);

    new_timer.callback = &zeroHeadingCallback;
    new_timer.dispatch_method = ESP_TIMER_TASK;
    esp_timer_create(&new_timer, &zeroHeadingTimer);

    systemState initialState = {
        .angle = 0,
        .angular_velocity = 0,
        .angular_acceleration = 0,
        .variance_A = 0.1,
        .variance_AV = 0,
        .variance_AA = 0,
        .time = (uint64_t)esp_timer_get_time(),
    };
    filter.setInitState(initialState);
}

void orientator::update() {
    const float velocityVariance = 0.003;
    const float headingVariance = 0.01;

    float measuredVelocity = 0;
    uint16_t measuredHeading = 0;

    getMagHeading(measuredHeading);
    getAccelVelocity(measuredVelocity);
    
    filter.makeMeasurement(systemState{
                           .angle=measuredHeading,
                           .angular_velocity=measuredVelocity,
                           .variance_A = headingVariance,
                           .variance_AV=velocityVariance,
                           .time = (uint64_t) esp_timer_get_time(),
                       });
    systemState currentState = filter.stateUpdate();
    // angularVelocity = max(min(currentState.angularVelocity, 1.5*VELOCITY_MAX), (double)0);
    //ESP_LOGI("Velocity", "measured: %lf, kalman: %lf", measuredVelocity, angularVelocity);

    esp_timer_stop(zeroHeadingTimer);
    if (angularVelocity > 13) {
        rotationPeriod = (double)(1000*2*M_PI)/angularVelocity;
        zeroCrossingTime = esp_timer_get_time() - (double)currentState.angle*LSB2ROT*rotationPeriod*RESOLUTION;
        int64_t startDelay = zeroCrossingTime - esp_timer_get_time() - (int)(offset*rotationPeriod*RESOLUTION);
        if (startDelay < 0) startDelay = (startDelay % (int)(abs(rotationPeriod*RESOLUTION))) + (int)(abs(rotationPeriod*RESOLUTION));
        esp_timer_start_once(initTimer, startDelay);
    } else {
        if (onStopCallback != nullptr)
            onStopCallback();
        rotationPeriod = 0;
        zeroCrossingTime = esp_timer_get_time();
    }

}

bool orientator::getAccelVelocity(float& accelVelocity) {
    vec2<int16_t> vec_l, vec_r;
    if (!accel_l.getXY(vec_l)) return false; // return false if read fails
    if (!accel_r.getXY(vec_r)) return false; // return false if read fails

    vec2<float> CoR = getCenterOfRotation(accel_l.getXYZ100Avg(), accel_r.getXYZ100Avg());
    
    // double normAccel = hypot(x,y);
    float left_velocity = sqrt(MathUtils::fastLength(vec_l)*LSB_TO_MPS2/(MathUtils::fastLength(XL_l_pos - CoR))); // radians per second
    float right_velocity = sqrt(MathUtils::fastLength(vec_r)*LSB_TO_MPS2/(MathUtils::fastLength(XL_r_pos - CoR))); // radians per second

    accelVelocity = (left_velocity + right_velocity)/2;
    // // report sensor error when maxing out the sensor
    // if (normAccel*LSB_TO_G > 280) return false;

    return true;
}

bool orientator::getMagHeading(uint16_t &heading) {

    vec2<int32_t> vec;
    if (!mag.getXY(vec)) return false;

    heading = RAD2LSB*atan2(vec.y, vec.x);
    
    return true;
}