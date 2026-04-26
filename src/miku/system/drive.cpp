#include "config.h"
#include "miku/system/drive.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "miku/devices/motor.hpp"
#include "miku/devices/imu.hpp"
#include "miku/util/geometry.hpp"
#include "miku/util/pid.hpp"
#include <memory>

LookupTable left_drive_lut({
    {-660, -12000}, {-611, -11000}, {-565, -10000}, {-513, -9000}, {-481, -8500},
    {-453, -8000}, {-425, -7500}, {-396, -7000}, {-367, -6500}, {-335, -6000}, 
    {-302, -5500}, {-273, -5000}, {-243, -4500}, {-215, -4000}, {-183, -3500}, 
    {-154, -3000}, {-115, -2500}, {-83, -2000}, {-58, -1500}, {-25, -1000},
    {0, -500}, {0, 0}, {0, 500}, 
    {28, 1000}, {56, 1500}, {86, 2000}, {114, 2500}, {151, 3000},
    {181, 3500}, {214, 4000}, {243, 4500}, {273, 5000}, {302, 5500},
    {339, 6000}, {366, 6500}, {397, 7000}, {422, 7500}, {450, 8000},
    {480, 8500}, {511, 9000}, {570, 10000}, {623, 11000}, {660, 12000}
});


LookupTable right_drive_lut({
    {-660, -12000}, {-606, -11000}, {-564, -10000}, {-510, -9000}, {-477, -8500},
    {-451, -8000}, {-421, -7500}, {-391, -7000}, {-366, -6500}, {-334, -6000}, 
    {-301, -5500}, {-272, -5000}, {-241, -4500}, {-213, -4000}, {-183, -3500}, 
    {-153, -3000}, {-115, -2500}, {-83, -2000}, {-58, -1500}, {-25, -1000},
    {0, -500}, {0, 0}, {0, 500}, 
    {28, 1000}, {56, 1500}, {86, 2000}, {114, 2500}, {150, 3000},
    {180, 3500}, {212, 4000}, {241, 4500}, {270, 5000}, {300, 5500},
    {335, 6000}, {365, 6500}, {395, 7000}, {418, 7500}, {448, 8000},
    {477, 8500}, {510, 9000}, {566, 10000}, {614, 11000}, {660, 12000}
});

miku::MotorGroup left_motors(LEFT_MOTOR_PORTS, DRIVE_MOTOR_CARTRIDGE, pros::v5::MotorUnits::degrees, left_drive_lut, DRIVE_PID_GAINS);
miku::MotorGroup right_motors(RIGHT_MOTOR_PORTS, DRIVE_MOTOR_CARTRIDGE, pros::v5::MotorUnits::degrees, right_drive_lut, DRIVE_PID_GAINS);

miku::Imu imu(IMU_PORT);

Point get_drive_position() {
    return {left_motors.get_average_position(), right_motors.get_average_position()};
}
Point get_drive_velocity() {
    return {left_motors.get_average_velocity(), right_motors.get_average_velocity()};
}
Point get_drive_torque() {
    return {left_motors.get_average_torque(), right_motors.get_average_torque()};
}
Point get_drive_temperature() {
    return {left_motors.get_highest_temperature(), right_motors.get_highest_temperature()};
}

void move_drive(float left_speed, float right_speed) {
    left_motors.move_voltage(left_speed);
    right_motors.move_voltage(right_speed);
}
void move_drive(MotorCommand left_cmd, MotorCommand right_cmd) {
    if(left_cmd.type == MotorCommandType::VOLTAGE) {
        left_motors.move_voltage(left_cmd.value);
    } else {
        left_motors.move_velocity(left_cmd.value);
    }
    if(right_cmd.type == MotorCommandType::VOLTAGE) {
        right_motors.move_voltage(right_cmd.value);
    } else {
        right_motors.move_velocity(right_cmd.value);
    }
}
void move_voltage(float left_speed, float right_speed) {
    left_motors.move_voltage(left_speed);
    right_motors.move_voltage(right_speed);
}
void move_velocity(float left_velocity, float right_velocity) {
    left_motors.move_velocity(left_velocity);
    right_motors.move_velocity(right_velocity);
}
void stop_drive() {
    left_motors.move_voltage(0);
    right_motors.move_voltage(0);
}
void set_brake_mode(pros::motor_brake_mode_e mode) {
    left_motors.set_brake_mode(mode);
    right_motors.set_brake_mode(mode);
}

compass_degrees get_imu_raw_heading() {
    return imu.get_rotation();
}

void calibrate_drive() {
    left_motors.tare_position();
    right_motors.tare_position();
    imu.calibrate();
}

int curve(int pos) {
    if(abs(pos) <= 10) return 0;
    // return 10502.7578057 * (std::exp(0.006 * abs(pos)) - 1.0) * (pos > 0 ? 1 : -1);
    return pos * (12000 / 127);
}

inline void funny_tank(int left_x, int left_y, int right_x, int right_y) {
    if(abs(left_x) > 50 && abs(right_x) > 50) {
        int sign = (left_x > 0 || right_x < 0) ? 1 : -1;
        left_x = (abs(left_x) - 50) * 127 / 77;
        right_x = (abs(right_x) - 50) * 127 / 77;
        float speed = (abs(left_x) + abs(right_x)) / 2.0 * sign;
        move_voltage(speed, speed);
    } else {
        move_voltage(curve(left_y), curve(right_y));
    }
}

void handle_joystick_inputs(DriveMode current_drive_mode) {
    int left_y = master.get_analog(ANALOG_LEFT_Y);
    int left_x = master.get_analog(ANALOG_LEFT_X);
    int right_y = master.get_analog(ANALOG_RIGHT_Y);
    int right_x = master.get_analog(ANALOG_RIGHT_X);

    switch (current_drive_mode) {
        case DriveMode::TANK:
            move_voltage(curve(left_y), curve(right_y));
            break;
        case DriveMode::ARCADE:
            move_voltage(curve(left_y) + curve(right_x), curve(left_y) - curve(right_x));
            break;
        case DriveMode::SINGLE_STICK_ARCADE:
            move_voltage(curve(left_y) + curve(left_x), curve(left_y) - curve(left_x));
            break;
        case DriveMode::FUNNY_TANK:
            funny_tank(left_x, left_y, right_x, right_y);
            break;
    }
}