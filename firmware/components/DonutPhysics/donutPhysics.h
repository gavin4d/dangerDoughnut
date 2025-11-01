#ifndef DONUT_PHYSICS_H
#define DONUT_PHYSICS_H

#include <cstdint>
#include <stdint.h>
#include "math.h"
#include "esp_timer.h"

#define LSB2DEG 360/65536
#define DEG2LSB 65536/360
#define RAD2LSB 65536/6.28
#define LSB2RAD 6.28/65536
#define ROT2LSB 65536
#define LSB2ROT 1/65536

// #define PROCESS_NOISE 5 // trust the process
#define SLIP_THRESHOLD 20 // rad/s
#define STATE_BUF_SIZE 128 // number of previous states to compare against the new parameters

typedef enum {
  WHEEL_COSF, // coefficient between ground and wheels
  WHEEL_RATIO, // radius of wheel divided by distance from center to wheel
  MOTOR_KT_R, // constant affects stall torque. Includes winding resistance (kt/R)
  MOTOR_KI, // constant affects velocity (1/kv)
  LINEAR_DRAG, // coefficient drag that is linear with velocity
  SQUARE_DRAG, // coefficient drag that scales with the square of velocity
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
struct system_state_t {
  uint16_t angle; // angle in 16 bit LSBs (65,536 LSBs per rotation) (east = 0, north = 0x3fff, west = 0x7fff, south = 0xbfff)
  float angular_velocity; // angular velocity measurement in radians per second
  float angular_acceleration; // angular acceleration measurement in radians per second per second
  float varience_angle; // varience estimate of the angle measurement
  float varience_velocity; // varience estimate of the angular velocity measurement
  float varience_acceleration; // varience estimate of the angular acceleration measurement
  bool upright;
  bool spining;
  float motor_torque;
  float motor_percentage;
  float wheel_velocity; 
  float battery_voltage;
  uint64_t time; // time of state
};

class DonutPhysics {

public:
  DonutPhysics();
  ~DonutPhysics();
  
  void init();
  void step(system_state_t &current_state, float delta_time);
  void step(system_state_t &current_state);
  void learn();
  float calcMotorPercent(float angular_velocity);
  void calcGradient();
  void gradientDecent();
  float cost();

  bool isSlipping(system_state_t &current_state);

  float process_noise = 5;

private:
  void stepPhysics(system_state_t &current_state, float delta_time);
  void loadValues();
  void writeValues();
  float error(system_state_t perdicted_state, system_state_t measured_state);
  void clipAdd(uint16_t param_id, float addend);
  parameter_t param_list[PARAMETER_COUNT];
  uint16_t state_buf_size = 0;
  system_state_t state_buf[STATE_BUF_SIZE];
 
};
#endif