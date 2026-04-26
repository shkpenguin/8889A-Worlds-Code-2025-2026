#pragma once

#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "miku/devices/motor.hpp"
#include "miku/util/geometry.hpp"
#include <memory>

enum class DriveMode {
    TANK = 0,
    ARCADE = 1,
    SINGLE_STICK_ARCADE = 2,
    FUNNY_TANK = 3
};

Point get_drive_position();
Point get_drive_velocity();
Point get_drive_torque();
Point get_drive_temperature();
compass_degrees get_imu_raw_heading();

void set_brake_mode(pros::motor_brake_mode_e mode);

void move_drive(float left_speed, float right_speed);
void move_drive(MotorCommand left_cmd, MotorCommand right_cmd);
void move_voltage(float left_speed, float right_speed);
void move_velocity(float left_velocity, float right_velocity);
void stop_drive();

void calibrate_drive();

void handle_joystick_inputs(DriveMode current_drive_mode);