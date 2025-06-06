#include "kalmanFilter.h"
#include "cmath"

kalmanFilter::kalmanFilter() {
}

void kalmanFilter::setInitState(systemState state) {
    estimateState = state;
}

void kalmanFilter::adjustAngle(float angle) {
    estimateState.angle += angle*RAD2LSB;
}

void kalmanFilter::adjustVelocity(float velocity) {
    estimateState.angular_velocity += velocity;
}

void kalmanFilter::adjustAccel(float accel) {
    estimateState.angular_acceleration += accel;
}

void kalmanFilter::makeMeasurement(float angle, float angular_velocity, float variance_A, float variance_AV) {
    measurmentState = {
        .angle = (uint16_t)(angle*RAD2LSB),
        .angular_velocity = angular_velocity,
        .variance_A = variance_A,
        .variance_AV = variance_AV,
        .time = (uint64_t) esp_timer_get_time(),
    };
}

void kalmanFilter::makeMeasurement(systemState state) {
    measurmentState = state;
}

systemState kalmanFilter::stateUpdate() {
    predict();
    update();
    return estimateState;
}

void kalmanFilter::predict() {
    delta_time = (float)(esp_timer_get_time() - estimateState.time)/1000000;
    estimateState.time = esp_timer_get_time();
    estimateState.angle += RAD2LSB*(delta_time*estimateState.angular_velocity + 0.5*pow(delta_time, 2)*estimateState.angular_acceleration);
    estimateState.angular_velocity += delta_time*estimateState.angular_acceleration;
    // acceleration is assumed to be constant so there is no point predicting it

    estimateState.variance_A += pow(delta_time, 2)*estimateState.variance_AV + 0.25*pow(delta_time, 4)*estimateState.variance_AA; // this is a estimation and is ignoring covariances
    estimateState.variance_AV += pow(delta_time, 2)*estimateState.variance_AA;
    estimateState.variance_AA += PROCESS_NOISE;
}

void kalmanFilter::update() {
    float K_A = estimateState.variance_A / (estimateState.variance_A + measurmentState.variance_A);
    float K_AV = estimateState.variance_AV / (estimateState.variance_AV + measurmentState.variance_AV);

    estimateState.angle += K_A*angleDiff(measurmentState.angle, estimateState.angle);
    estimateState.angular_acceleration += 0.1*K_AV*(measurmentState.angular_velocity - estimateState.angular_velocity)/delta_time;
    estimateState.angular_velocity += K_AV*(measurmentState.angular_velocity - estimateState.angular_velocity);

    estimateState.variance_A *= 1-K_A;
    estimateState.variance_AV *= 1-K_AV;
    estimateState.variance_AA *= pow(1-K_AV, 0.5); // honestly I have no idea what I'm doing here but it works
}

int16_t kalmanFilter::angleDiff(uint16_t minuend, uint16_t subtrahend) {
    int32_t output = minuend - subtrahend;
    if (output > 0x7fff)
        return output - 0xffff;
    if (output < -0x7fff)
        return output + 0xffff;
    return output;
}