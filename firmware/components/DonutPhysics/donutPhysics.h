#ifndef DONUT_PHYSICS_H
#define DONUT_PHYSICS_H

#include <cstdint>
#include <stdint.h>
#include "math.h"
#define LSB2DEG 360/65536
#define DEG2LSB 65536/360
#define RAD2LSB 65536/6.28
#define LSB2RAD 6.28/65536
#define ROT2LSB 65536
#define LSB2ROT 1/65536

#define SLIP_THRESHOLD 20 // rad/s
#define STATE_BUF_SIZE 128 // number of previous states to compare against the new parameters

typedef enum {
  WHEEL_COSF, // coefficient between ground and wheels
  WHEEL_RATIO, // radius of wheel divided by distance from center to wheel
  MOTOR_KT_R, // constant affects stall torque. Includes winding resistance (kt/R)
  MOTOR_KI, // constant affects velocity (1/kv)
  LINEAR_DRAG, // coefficient drag that is linear with time
  SQUARE_DRAG, // coefficient drag that scales with the square of time
  MOI, // moment of intera of the whole bot
  PARAMETER_COUNT
} parameters;

typedef struct {
  char key[16];
  float learning_rate;
  float value;
  float min;
  float max;
  float gradient;
} parameter_t;

// State of the system
typedef struct {
  uint16_t angle; // angle in 16 bit LSBs (65,536 LSBs per rotation) (east = 0, north = 0x3fff, west = 0x7fff, south = 0xbfff)
  float angular_velocity; // angular velocity measurement in radians per second
  float angular_acceleration; // angular acceleration measurement in radians per second per second
  bool upright;
  float motor_torque;
  float motor_percentage;
  float wheel_velocity; 
  float battery_voltage;
  uint64_t time; // time of state
} system_state_t;

class DonutPhysics {

public:
  DonutPhysics();
  ~DonutPhysics();
  
  void init();
  void step(system_state_t &current_state, float delta_time);
  float calcMotorPercent(float angular_velocity);
  void calcGradient();
  void gradientDecent();
  float cost();

  bool isSlipping(system_state_t &current_state);

private:
  void loadValues();
  float error(system_state_t perdicted_state, system_state_t measured_state);
  void clipAdd(uint16_t param_id, float addend);
  parameter_t param_list[PARAMETER_COUNT];
  uint16_t state_buf_size = 0;
  system_state_t state_buf[STATE_BUF_SIZE];
 
};
#endif