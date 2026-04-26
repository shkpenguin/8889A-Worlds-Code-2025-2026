#pragma once

#include "miku/devices/motor.hpp"
#include "pros/rtos.hpp"
#include <functional>
#include <memory>
#include <deque>

void set_intake(const MotorCommand& top_cmd, const MotorCommand& middle_cmd, const MotorCommand& bottom_cmd);
void set_intake(float top_voltage, float middle_voltage, float bottom_voltage);
void set_intake_velocity(float top_vel, float middle_vel, float bottom_vel);

void set_intake(const MotorCommand& top_cmd, const MotorCommand& bottom_cmd);
void set_intake(float top_voltage, float bottom_voltage);
void set_intake_velocity(float top_vel, float bottom_vel);

void set_intake_top(const MotorCommand& cmd);
void set_intake_top(float voltage);
void set_intake_top_velocity(float velocity);

void set_intake_middle(const MotorCommand& cmd);
void set_intake_middle(float voltage);
void set_intake_middle_velocity(float velocity);

void set_intake_bottom(const MotorCommand& cmd);
void set_intake_bottom(float voltage);
void set_intake_bottom_velocity(float velocity);

float get_intake_top_velocity();
float get_intake_middle_velocity();
float get_intake_bottom_velocity();

int get_intake_top_temperature();
int get_intake_middle_temperature();
int get_intake_bottom_temperature();

void load_intake();

void score_intake();
void score_intake_slow();

void stop_intake();

void set_anti_jam(bool enabled);

void queue_command(const MotorCommand& top_cmd,
                    const MotorCommand& middle_cmd,
                    const MotorCommand& bottom_cmd,
                    uint32_t duration_ms = 0,
                    bool front = false);

void queue_command(std::function<void()> action,
                    uint32_t duration_ms = 0,
                    bool front = false);

void update_intake();