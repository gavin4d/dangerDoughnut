#include "donutPhysics.h"
// #include "freertos/idf_additions.h"
#include "nvs.h"
#include "esp_log.h"
// #include <semaphore.h>
#define GRAD_CALC_STEP 0.1
#define ANGLE_ERROR_WEIGHT 0.002
#define VELOCITY_ERROR_WEIGHT 0.198
#define ACCEL_ERROR_WEIGHT 0.8

using namespace std;

// extern "C" {
  
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
    .learning_rate = 0.0001,
    .value = 0.235,
    .min = 0.2,
    .max = 0.262,
    .gradient = 0,
  };
  param_list[MOTOR_KV] = parameter_t{ // rad/Vs or Nm/A
    .key = "motor_kv",
    .learning_rate = 0.001,
    .value = 1100*M_PI/30, // = 115.2 rad/Vs
    .min = 100,
    .max = 150,
    .gradient = 0,
  };
  param_list[MOTOR_R] = parameter_t{
    .key = "motor_r",
    .learning_rate = 0.001,
    .value = 0.0579, // ohm
    .min = 0.05,
    .max = 0.2,
    .gradient = 0,
  };
  param_list[LINEAR_DRAG] = parameter_t{
    .key = "linear_drag",
    .learning_rate = 0.1,
    .value = 0, // rad/s^2 s/rad  =  Hz
    .min = 0,
    .max = 100,
    .gradient = 0,
  };
  param_list[SQUARE_DRAG] = parameter_t{
    .key = "square_drag",
    .learning_rate = 0.1,
    .value = 0, // rad/s^2 / (rad/s)^2  =  1/rad
    .min = 0,
    .max = 100,
    .gradient = 0,
  };
  param_list[MOI] = parameter_t{
    .key = "moi",
    .learning_rate = 0.001,
    .value = 0.0043802836715, // without ring
    // .value = 0.00816242827499, // with ring
    .min = 0.004,
    .max = 0.009,
    .gradient = 0,
  };
}

void DonutPhysics::init() {
  loadValues();
}

// TODO:
// torque reduction when slipping
// import real data for training

void DonutPhysics::step(system_state_t &current_state) {
  current_state.dt = (float)(esp_timer_get_time() - current_state.time)/1000000;
  current_state.time = esp_timer_get_time();
  stepPhysics(current_state);
}

void DonutPhysics::step(system_state_t &current_state, float delta_time) {
  current_state.time += delta_time * 1000000;
  current_state.dt = delta_time;
  stepPhysics(current_state);
}

void DonutPhysics::learn() {
  state_buf_index = 0; // start collecting samples
  // TODO: sem take
  // TODO: sem wait
  calcGradient();
  gradientDecent();
}

void DonutPhysics::stepPhysics(system_state_t &current_state) {
  // collect training sample before step to not make a feedback loop
  if (state_buf_index < STATE_BUF_SIZE) {
    state_buf[state_buf_index] = current_state;
    state_buf_index++;
    if (state_buf_index == STATE_BUF_SIZE)
      ; // TODO: sem post
  }
  
  current_state.angle += RAD2LSB*(current_state.dt*current_state.angular_velocity + 0.5*pow(current_state.dt, 2)*current_state.angular_acceleration);
  current_state.angular_velocity += current_state.dt*current_state.angular_acceleration;
  const float input_voltage = current_state.battery_voltage * current_state.motor_percentage;
  const float back_emf =  current_state.angular_velocity / param_list[WHEEL_RATIO].value / param_list[MOTOR_KV].value; // at no-load speed when this equals input voltage
  current_state.motor_torque =  (input_voltage - back_emf) / (param_list[MOTOR_R].value * param_list[MOTOR_KV].value);
  const float drag = param_list[LINEAR_DRAG].value * current_state.angular_velocity + param_list[SQUARE_DRAG].value * pow(current_state.angular_velocity, 2);
  current_state.angular_acceleration = 2 * current_state.motor_torque / (param_list[MOI].value * param_list[WHEEL_RATIO].value) - drag;
  current_state.varience_angle += pow(current_state.dt * RAD2LSB, 2)*current_state.varience_velocity + 0.25*pow(current_state.dt, 4)*pow(RAD2LSB, 2)*current_state.varience_acceleration; // this is a estimation and is ignoring covariences
  current_state.varience_velocity += pow(current_state.dt, 2)*current_state.varience_acceleration;
  current_state.varience_acceleration += process_noise;
}

float DonutPhysics::calcMotorPercent(float angular_velocity) {
  const float drag = param_list[LINEAR_DRAG].value * angular_velocity + param_list[SQUARE_DRAG].value * pow(angular_velocity, 2);
  return 0.5 * drag * param_list[MOI].value * param_list[WHEEL_RATIO].value * param_list[MOTOR_KV].value * param_list[MOTOR_R].value + angular_velocity / (param_list[WHEEL_RATIO].value * param_list[MOTOR_KV].value);
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
    const float init_cost = cost();
    const float init_value = param_list[param_id].value;
    clipAdd(param_id, init_value*GRAD_CALC_STEP);
    param_list[param_id].gradient = (cost() - init_cost)/(init_value*GRAD_CALC_STEP);
    param_list[param_id].value = init_value; // return param back to its original value
  }
}

// total square error of state buffer with current physics params
float DonutPhysics::cost() {
  float total_square_error = 0;
  system_state_t test_state = state_buf[0]; // this is the state we will step forwards and compare with the measured state
  for (int i = 1; i < STATE_BUF_SIZE; i++) {
    test_state.motor_percentage = state_buf[i].motor_percentage; // update input of test_state
    test_state.dt = state_buf[i].dt;
    stepPhysics(test_state);
    total_square_error += pow(error(test_state, state_buf[i]), 2);
  }
  return total_square_error/STATE_BUF_SIZE;
}

float DonutPhysics::error(system_state_t perdicted_state, system_state_t measured_state) { // difference between two system states
  const float angle_error = measured_state.angle - perdicted_state.angle;
  const float velocity_error = measured_state.angular_velocity - perdicted_state.angular_velocity;
  const float accel_error = measured_state.angular_acceleration - perdicted_state.angular_acceleration;
  return ANGLE_ERROR_WEIGHT * angle_error + VELOCITY_ERROR_WEIGHT * velocity_error + ACCEL_ERROR_WEIGHT * accel_error;
}

void DonutPhysics::clipAdd(uint16_t param_id, float addend) {
  const float value = param_list[param_id].value + addend;
  if (value > param_list[param_id].max)
    param_list[param_id].value = param_list[param_id].max;
  else if (value < param_list[param_id].min)
    param_list[param_id].value = param_list[param_id].min;
  else
    param_list[param_id].value = value;
}

void DonutPhysics::loadValues() {
  nvs_handle_t my_handle;
  nvs_open("physics", NVS_READWRITE, &my_handle);
  for (int i = 0; i < PARAMETER_COUNT; i++) {
    size_t read_size = sizeof(param_list[i].value);
    nvs_get_blob(my_handle, param_list[i].key, &param_list[i].value, &read_size);
  }
  nvs_close(my_handle);
}

void DonutPhysics::writeValues() {
  nvs_handle_t my_handle;
  nvs_open("physics", NVS_READWRITE, &my_handle);
  for (int i = 0; i < PARAMETER_COUNT; i++) {
    nvs_set_blob(my_handle, param_list[i].key, &param_list[i].value, sizeof(param_list[i].value));
  }
  esp_err_t err = nvs_commit(my_handle);
  if (err != ESP_OK) ESP_LOGI("NVS", "err %s", esp_err_to_name(err));
  nvs_close(my_handle);
}
// }