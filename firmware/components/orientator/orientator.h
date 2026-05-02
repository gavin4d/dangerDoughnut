#ifndef ORIENTATOR_H
#define ORIENTATOR_H

#include "H3LIS331DL.h"
#include "esp_timer.h"
#include "sensor.h"
#include "donutPhysics.h"
#include "kalmanFilter.h"
#include "math.h"
#include "MathUtils.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <cstdint>

#define ROLLING_AVERAGE_SIZE 500
#define SPINNING_THRESHOLD 24 // rad/s
#define CAL_CURVE_SIZE 30
#define MAX_VELOCITY 366
const float SENSOR_XL_VARIENCE_SCALAR = 0.25 / 0.04131; // 0.25 from square root, sensor distace

class Orientator {

    public:
        vec2<float> CoR = {0,0};
        float accel_1_dist = 0.0413;
        float accel_2_dist = 0.0413;
        vec3<int16_t> vec_m = {0,0,0};
        vec3<float> accel_avg_1 = {0,0,0}, accel_avg_2 = {0,0,0};
        vec3<int16_t> accel_val_1 = {0,0,0}, accel_val_2 = {0,0,0};
        system_state_t state;
        system_state_t measured_state;
        float accel_varience = 21;
        float mag_varience = 200;
        uint16_t angle_offset = 0;

        Orientator();
        Orientator(DonutPhysics* physics);
        ~Orientator();

        void setup(Sensor* accel_1, Sensor* accel_2, Sensor* mag);
        void update();
        void readConfig();
        void writeConfig();
        uint16_t getHeading();
        float getVelocity();
        void setOffset(double offset);
        void enableAccelCalibration(bool enable); // keep the robot level and upright
        void enableMagCalibration(bool enable); // spin the robot
        vec2<float> getCenterOfRotation(vec2<float> XL_2, vec2<float> XL_1);

    private:
        DonutPhysics* physics;
        Sensor* accel_1;
        Sensor* accel_2;
        Sensor* mag;
        uint64_t zeroCrossingTime = 0; // time stamp of last zero crossing
        bool accel_calibration = false;
        bool mag_calibration = false;
        float velocity_cal_curve[CAL_CURVE_SIZE];

        void setVelocityCalCurve(float raw_velocity, float angle_derivative);
        float getVelocityCalCurve(float raw_velocity);
        void getAccelVelocity(float& accel, float& velocity); // rad/s
        uint16_t getMagHeading(); // returns heading in 16 bit LSBs (65,536 LSBs per rotation) (east = 0, north = 0x3fff, west = 0x7fff, south = 0xbfff)

};
#endif
