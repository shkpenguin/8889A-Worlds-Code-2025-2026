#include "miku/util/utils.hpp"
#include "miku/util/pid.hpp"
#include "api.h"

PID::PID(PIDGains pid_gains, float windup_range, bool sign_flip_reset, bool trapezoidal)
    : pid_gains(pid_gains),
      windup_range(windup_range),
      sign_flip_reset(sign_flip_reset),
      trapezoidal(trapezoidal) {};
PID::PID()
    : pid_gains(),
      windup_range(0),
      sign_flip_reset(true),
      trapezoidal(true) {};

float PID::update(const float error) {

    // calculate derivative
    const float derivative = error - prevError;

    // calculate integral
    if (trapezoidal) {
        integral += derivative / 2;
    }
    if (sign(error) != sign((prevError)) && sign_flip_reset) integral = 0;
    if (fabs(error) > windup_range && windup_range != 0) integral = 0;

    prevError = error;

    // calculate output
    return error * pid_gains.kP + integral * pid_gains.kI + derivative * pid_gains.kD;

}

float PID::update(const float error, const float derivative) {

    // calculate integral
    if (trapezoidal) {
        integral += derivative / 2;
    }
    if (sign(error) != sign((prevError)) && sign_flip_reset) integral = 0;
    if (fabs(error) > windup_range && windup_range != 0) integral = 0;

    prevError = error;

    // calculate output
    return error * pid_gains.kP + integral * pid_gains.kI + derivative * pid_gains.kD;

}

PID& PID::operator=(const PID& other) {
    if (this != &other) {
        pid_gains = other.pid_gains;
        integral = other.integral;
        prevError = other.prevError;
    }
    return *this;
}

void PID::reset() {
    integral = 0;
    prevError = 0;
}