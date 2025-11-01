#include "kalmanFilter.h"

void kalmanFilter::stateUpdate(system_state_t& perdicted_state, const system_state_t& measured_state) {
    float K_A = perdicted_state.varience_angle / (perdicted_state.varience_angle + measured_state.varience_angle);
    float K_AV = perdicted_state.varience_velocity/ (perdicted_state.varience_velocity + measured_state.varience_velocity);

    perdicted_state.angle += K_A*angleDiff(measured_state.angle, perdicted_state.angle);
    perdicted_state.angular_acceleration += 100*K_AV*(measured_state.angular_velocity - perdicted_state.angular_velocity); //0.1*___/delta_time;
    perdicted_state.angular_velocity += K_AV*(measured_state.angular_velocity - perdicted_state.angular_velocity);

    perdicted_state.varience_angle *= 1-K_A;
    perdicted_state.varience_velocity*= 1-K_AV;
    perdicted_state.varience_acceleration *= pow(1-K_AV, 0.5); // honestly I have no idea what I'm doing here but it works
}

int16_t kalmanFilter::angleDiff(uint16_t minuend, uint16_t subtrahend) {
    int32_t output = minuend - subtrahend;
    if (output > 0x7fff)
        return output - 0xffff;
    if (output < -0x7fff)
        return output + 0xffff;
    return output;
}