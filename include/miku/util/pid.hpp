#pragma once

#include <vector>
#include "miku/util/lut.hpp"

struct PIDGains {
    float kP;
    float kI;
    float kD;

    PIDGains(float kP, float kI, float kD) : kP(kP), kI(kI), kD(kD) {}
    PIDGains() : kP(0.0), kI(0.0), kD(0.0) {}
};

struct FeedforwardGains {
    float kS;
    float kV;
    float kA;

    FeedforwardGains(float kS, float kV, float kA) : kS(kS), kV(kV), kA(kA) {}
    FeedforwardGains() : kS(0.0), kV(0.0), kA(0.0) {}
};

/**
 * @param Gains gains The PID gains
 * @param float windup_range The range at which to limit the integral term (default 0, no limit)
 * @param bool sign_flip_reset Whether to reset the integral term when the error changes sign (default true)
 * @param bool trapezoidal Whether to use trapezoidal integration for the integral term (default true)
*/
class PID {
    public:
        PID(PIDGains pid_gains, float windup_range = 0, bool sign_flip_reset = true, bool trapezoidal = true);
        PID();

        float update(float error);
        float update(float error, float derivative);

        void reset();

        PID& operator=(const PID& other);
    protected:
        // gains 
        PIDGains pid_gains;

        // optimizations
        const float windup_range;
        const bool sign_flip_reset;
        const bool trapezoidal;

        float integral = 0;
        float prevError = 0;
};