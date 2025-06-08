#ifndef ORIENTATOR_H
#define ORIENTATOR_H

#include "esp_timer.h"
#include "sensor.h"
#include "donutPhysics.h"
#include "kalmanFilter.h"
#include "math.h"
#include "MathUtils.h"
#include <cstdint>

#define ROLLING_AVERAGE_SIZE 500
#define SPINNING_THRESHOLD 13 // rad/s
#define SENSOR_MAG_VARIENCE state.angular_velocity > SPINNING_THRESHOLD ? 0.01f : 0.001f
#define SENSOR_XL_VARIENCE state.angular_velocity > SPINNING_THRESHOLD ? 0.003f : 0.1f

class orientator {

    public:
        vec2<float> CoR;
        vec3<int16_t> vec_m = {0,0,0};
        vec3<float> accel_avg_1, accel_avg_2;
        vec3<int16_t> accel_val_1, accel_val_2;
        system_state_t state;

        orientator();
        ~orientator();

        void setup(Sensor* accel_1, Sensor* accel_2, Sensor* mag);
        void update();
        uint16_t getHeading();
        float getVelocity();
        void setZeroCrossCallback(void(* callback)());
        void stopZeroCrossCallback();
        void setOnStopCallback(void(* callback)());
        void setOffset(double offset);
        void adjustAngle(float angle);
        void adjustVelocity(float velocity);
        void adjustAccel(float accel);
        void enableAccelCalibration(bool enable); // keep the robot level and upright
        void enableMagCalibration(bool enable); // spin the robot
        vec2<float> getCenterOfRotation(vec3<float> XL_l, vec3<float> XL_r);

    private:
        DonutPhysics* physics;
        Sensor* accel_r;
        Sensor* accel_l;
        Sensor* mag;
        uint64_t zeroCrossingTime = 0; // time stamp of last zero crossing

        esp_timer_handle_t update_timer;
        static esp_timer_handle_t zeroHeadingTimer;
        static void (* zeroCrossCallback)();
        static void (* onStopCallback)();
        static void zeroHeadingCallback(void *args);

        float getAccelVelocity(); // rad/s
        uint16_t getMagHeading(); // returns heading in 16 bit LSBs (65,536 LSBs per rotation) (east = 0, north = 0x3fff, west = 0x7fff, south = 0xbfff)
        static void emptyFunction() {};

};
#endif
