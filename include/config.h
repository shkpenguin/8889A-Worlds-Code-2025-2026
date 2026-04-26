#pragma once

#include "miku/devices/pneumatic.hpp"
#include "miku/devices/controller.hpp"
#include "miku/util/exit.hpp"

#define IMU_PORT 17

#define LEFT_MOTOR_PORTS {-11, -12, 13}
#define RIGHT_MOTOR_PORTS {-18, 19, 20}
#define DRIVE_MOTOR_CARTRIDGE pros::v5::MotorGears::blue
#define DRIVE_PID_GAINS PIDGains(8.0, 5.0, 40.0)

#define INTAKE_TOP_PORT -10
#define INTAKE_TOP_CARTRIDGE pros::v5::MotorGears::green
#define INTAKE_TOP_PID_GAINS PIDGains(20.0, 10.0, 0.0)
#define INTAKE_MIDDLE_PORT -9
#define INTAKE_MIDDLE_CARTRIDGE pros::v5::MotorGears::green
#define INTAKE_MIDDLE_PID_GAINS PIDGains(20.0, 5.0, 0.0)
#define INTAKE_BOTTOM_PORT 8
#define INTAKE_BOTTOM_CARTRIDGE pros::v5::MotorGears::blue
#define INTAKE_BOTTOM_PID_GAINS PIDGains(20.0, 5.0, 0.0)

#define LINEAR_PID_GAINS PIDGains(650.0, 0.0, 4500.0)
#define ANGULAR_PID_GAINS PIDGains(340.0, 0.0, 3100.0)

#define DRIVE_QUICK_EXIT PatienceExit(2, 1, false, 5.0)
#define DRIVE_SLOW_EXIT PatienceExit(3, 0.3, false, 5.0)
#define TURN_QUICK_EXIT PatienceExit(2, 1, false, 10.0)
#define TURN_SLOW_EXIT PatienceExit(5, 0.5, false, 10.0)

#define WHEEL_DIAMETER 3.25f
#define WHEEL_CIRC (WHEEL_DIAMETER * M_PI)
#define GEAR_RATIO 0.75f
#define TRACK_WIDTH 10.75f
#define MAX_RPM 660.0f
#define MAX_VEL 84.233953f
#define MAX_ANG_VEL 16.8467906f
#define DELTA_TIME 10
#define DEFAULT_AUTONOMOUS_BRAKE_MODE pros::E_MOTOR_BRAKE_BRAKE

#define USE_MCL 1

extern miku::Pneumatic loader_piston;
extern miku::Pneumatic lock_piston;
extern miku::Pneumatic middle_piston;
extern miku::Pneumatic descore_piston;

extern miku::Controller master;