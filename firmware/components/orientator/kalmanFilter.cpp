#include "kalmanFilter.h"
#include "cmath"

kalmanFilter::kalmanFilter() {
}

void kalmanFilter::setInitState(systemState state) {
    estimateState = state;
}

void kalmanFilter::adjustAngle(double angle) {
    estimateState.angle += angle*RAD2LSB;
}

void kalmanFilter::adjustVelocity(double velocity) {
    estimateState.angularVelocity += velocity;
}

void kalmanFilter::adjustAccel(double accel) {
    estimateState.angularAcceleration += accel;
}

void kalmanFilter::makeMeasurement(double angle, double angularVelocity, double variance_A, double variance_AV) {
    measurmentState = {
        .angle = (uint16_t)(angle*RAD2LSB),
        .angularVelocity = angularVelocity,
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
    deltaTime = (double)(esp_timer_get_time() - estimateState.time)/1000000;
    estimateState.time = esp_timer_get_time();
    estimateState.angle += RAD2LSB*(deltaTime*estimateState.angularVelocity + 0.5*pow(deltaTime, 2)*estimateState.angularAcceleration);
    estimateState.angularVelocity += deltaTime*estimateState.angularAcceleration;
    // acceleration is assumed to be constant so there is no point predicting it

    estimateState.variance_A += pow(deltaTime, 2)*estimateState.variance_AV + 0.25*pow(deltaTime, 4)*estimateState.variance_AA; // this is a estimation and is ignoring covariances
    estimateState.variance_AV += pow(deltaTime, 2)*estimateState.variance_AA;
    estimateState.variance_AA += PROCESS_NOISE;
}

void kalmanFilter::update() {
    float K_A = estimateState.variance_A / (estimateState.variance_A + measurmentState.variance_A);
    float K_AV = estimateState.variance_AV / (estimateState.variance_AV + measurmentState.variance_AV);

    estimateState.angle += K_A*angleDiff(measurmentState.angle, estimateState.angle);
    estimateState.angularAcceleration += 0.1*K_AV*(measurmentState.angularVelocity - estimateState.angularVelocity)/deltaTime;
    estimateState.angularVelocity += K_AV*(measurmentState.angularVelocity - estimateState.angularVelocity);

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