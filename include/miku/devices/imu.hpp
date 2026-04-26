#pragma once

#include "miku/devices/controller.hpp"
#include "pros/imu.hpp"
#include "config.h"
#include "math.h"

extern miku::Controller master;

namespace miku {
    class Imu : public pros::Imu {
        using pros::Imu::Imu;
        
        public:
        void calibrate(int max_attempts = 2) {
            if(pros::Imu::is_installed()) {
                int attempts = 0;
                while(attempts < max_attempts) {
                    pros::Imu::reset();
                    while (pros::Imu::get_status() != pros::ImuStatus::error && pros::Imu::is_calibrating()) pros::delay(20);
                    if(!isnanf(pros::Imu::get_heading()) && !isinf(pros::Imu::get_heading())) break;
                    attempts++;
                }
                if(attempts == max_attempts) master.alert("imu calibration failed", 3000, "---");
            }
            else master.alert("imu disconnect", 3000, "---");
        }
    };
} // namespace miku