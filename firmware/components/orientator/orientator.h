#ifndef ORIENTATOR_H
#define ORIENTATOR_H

#include "esp_timer.h"
#include <H3LIS331DL.h>
#include "MMC5983MA.h"
#include <bitset>
#include "kalmanFilter.h"
#include "math.h"
#include "MathUtils.h"
#include "driver/gpio.h"

#define NUM_ACCEL_POS 10
#define ACCEL_POS_SPREAD (VELOCITY_MAX/(NUM_ACCEL_POS-1))

class orientator {

    public:
        orientator();
        ~orientator();

        void setup(H3LIS331DL accel);
        void update();
        //boolean updatePeriod();
        //boolean updateOrientation();
        double getAngle();
        double getXAccel();
        double getXSign();
        double getYAccel();
        double getYSign();
        double getZAccel();
        double getZSign();
        void setZeroCrossCallback(void(* callback)());
        void stopZeroCrossCallback();
        void setOnStopCallback(void(* callback)());
        void setOffset(double offset);
        void adjustAngle(double angle);
        void adjustVelocity(double velocity);
        void adjustAccel(double accel);
        double getOffset();
        void setAccelPos(double accelPos);
        void setAccelPos(double accelPos, int index);
        double getAccelPos();
        double getAccelPos(int index);
        static vec2<float> getCenterOfRotation(vec3<float> XL_l, vec3<float> XL_r);
        double getPeriod();
        double getVelocity();

    private:
        H3LIS331DL accel_r;
        H3LIS331DL accel_l;
        MMC5983MA mag;
        double accelPos[NUM_ACCEL_POS] = {0.030};
        double offset = 0;
        static double rotationPeriod; // milliseconds
        double angularVelocity; // radians per second
        uint64_t zeroCrossingTime = 0; // time stamp of last zero crossing
        double lastRotationPeriod = 0;

        kalmanFilter filter;
        esp_timer_handle_t update_timer;
        esp_timer_handle_t initTimer;
        static esp_timer_handle_t zeroHeadingTimer;
        static void (* zeroCrossCallback)();
        static void (* onStopCallback)();
        static void initCallback(void *args);
        static void zeroHeadingCallback(void *args);
        bool getAccelVelocity(float& accelVelocity);
        bool getMagHeading(uint16_t &heading);
        double getAngle(uint64_t period);

};
#endif
