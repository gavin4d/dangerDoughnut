#pragma once
#include <stdint.h>
#include "math.h"
#include "donutPhysics.h"

#define LSB2DEG 360/65536
#define DEG2LSB 65536/360
#define RAD2LSB 65536/6.28
#define LSB2RAD 6.28/65536
#define ROT2LSB 65536
#define LSB2ROT 1/65536

namespace kalmanFilter {
    /**
    * @brief Updates the perdicted state using the measured state to the most likely system state
    * @param perdicted_state current state of the system as perdicted by a physics model. Make sure to step the physics before running stateUpdate
    * @param measured_state current state of the system from as read from only sensor inputs. Make sure angle is in LSBs
    */
    void stateUpdate(system_state_t& perdicted_state, const system_state_t& measured_state);

    int16_t angleDiff(uint16_t minuend, uint16_t subtrahend);
};
