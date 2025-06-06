#include "donutPhysics.h"

DonutPhysics::DonutPhysics() {
  param_list[WHEEL_COSF] = parameter_t{
    .key = "wheel_cosf",
    .learning_rate = 0,
    .value = 0.5,
    .min = 0,
    .max = 1,
    .gradient = 0,
  };
  param_list[WHEEL_RATIO] = parameter_t{
    .key = "wheel_ratio",
    .learning_rate = 0,
    .value = 0.23,
    .min = 0,
    .max = 1,
    .gradient = 0,
  };
  param_list[MOTOR_KT_R] = parameter_t{
    .key = "motor_kt",
    .learning_rate = 0,
    .value = 0.05,
    .min = 0,
    .max = 1,
    .gradient = 0,
  };
  param_list[MOTOR_KI] = parameter_t{
    .key = "motor_ki",
    .learning_rate = 0,
    .value = 0.008,
    .min = 0,
    .max = 1,
    .gradient = 0,
  };
  param_list[LINEAR_DRAG] = parameter_t{
    .key = "linear_drag",
    .learning_rate = 0,
    .value = 0,
    .min = 0,
    .max = 1,
    .gradient = 0,
  };
  param_list[SQUARE_DRAG] = parameter_t{
    .key = "square_drag",
    .learning_rate = 0,
    .value = 0.001,
    .min = 0,
    .max = 1,
    .gradient = 0,
  };
  param_list[MOI] = parameter_t{
    .key = "moi",
    .learning_rate = 0,
    .value = 0.00816242827499,
    .min = 0,
    .max = 1,
    .gradient = 0,
  };
}

void DonutPhysics::init() {
}

// TODO:
// torque reduction when slipping
// import real data for training

void DonutPhysics::step(system_state_t &current_state, float delta_time) {
  current_state.time += delta_time * 1000000;
  current_state.angle += RAD2LSB*(delta_time*current_state.angular_velocity + 0.5*pow(delta_time, 2)*current_state.angular_acceleration);
  current_state.angular_velocity += delta_time*current_state.angular_acceleration;
  const float input_voltage = current_state.battery_voltage * current_state.motor_percentage;
  const float back_emf = param_list[MOTOR_KI].value * current_state.angular_velocity / param_list[WHEEL_RATIO].value; // at no-load speed when this equals input voltage
  current_state.motor_torque = param_list[MOTOR_KT_R].value * (input_voltage - back_emf);
  const float drag = param_list[LINEAR_DRAG].value * current_state.angular_velocity + param_list[SQUARE_DRAG].value * pow(current_state.angular_velocity, 2);
  current_state.angular_acceleration = 2 * current_state.motor_torque / (param_list[MOI].value * param_list[WHEEL_RATIO].value) - drag;
}

float DonutPhysics::calcMotorPercent(float angular_velocity) {
  const float drag = param_list[LINEAR_DRAG].value * angular_velocity + param_list[SQUARE_DRAG].value * pow(angular_velocity, 2);
  return 0.5 * drag * param_list[MOI].value * param_list[WHEEL_RATIO].value / param_list[MOTOR_KT_R].value + param_list[MOTOR_KI].value * angular_velocity / param_list[WHEEL_RATIO].value;
}

bool DonutPhysics::isSlipping(system_state_t &current_state) {
  return abs(current_state.angular_velocity - current_state.wheel_velocity*param_list[WHEEL_RATIO].value) > SLIP_THRESHOLD;
}

void DonutPhysics::gradientDecent() {
  for (int param_id = 0; param_id < PARAMETER_COUNT; param_id++) {
    param_list[param_id].value -= param_list[param_id].learning_rate * param_list[param_id].gradient;  
  }
}

void DonutPhysics::calcGradient() {
  for (int param_id = 0; param_id < PARAMETER_COUNT; param_id++) {
    if (param_list[param_id].learning_rate == 0) continue; // no point calculating if we aren't going to use it
    float init_value = param_list[param_id].value;
    clipAdd(param_id, init_value*0.1);
    param_list[param_id].gradient = cost()/(init_value*0.1);
    param_list[param_id].value = init_value;
  }
}

float DonutPhysics::cost() {
  float total_square_error = 0;
  system_state_t test_state = state_buf[0]; // this is the state we will step forwards and compare with the measured state
  for (int i = 1; i < state_buf_size; i++) {
    step(test_state, (float)(state_buf[i].time - test_state.time)/1000000);
    total_square_error += pow(error(test_state, state_buf[i]), 2);
  }
  return total_square_error/state_buf_size;
}

float DonutPhysics::error(system_state_t perdicted_state, system_state_t measured_state) { // difference between two system states
  return measured_state.angular_velocity - perdicted_state.angular_velocity;
}

void DonutPhysics::clipAdd(uint16_t param_id, float addend) {
  param_list[param_id].value += addend;
  if (param_list[param_id].value > param_list[param_id].max) param_list[param_id].value = param_list[param_id].max;
  if (param_list[param_id].value < param_list[param_id].min) param_list[param_id].value = param_list[param_id].min;
}

void DonutPhysics::loadValues() {
  // for (int i = 0; i < PARAMETER_COUNT; i++) {
  //   param_list[i].value = preferences.getFloat(param_list[i].EEPROM_name, param_list[i].value);
  // }
}
