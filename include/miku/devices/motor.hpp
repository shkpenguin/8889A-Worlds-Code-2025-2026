#pragma once

#include "pros/motors.hpp"
#include "miku/util/pid.hpp"
#include "miku/util/lut.hpp"
#include <memory>

enum class MotorCommandType {
    VOLTAGE,
    VELOCITY
};

constexpr MotorCommandType mV  = MotorCommandType::VOLTAGE;
constexpr MotorCommandType rpm = MotorCommandType::VELOCITY;

struct MotorCommand {
    float value;
    MotorCommandType type;
    
    constexpr MotorCommand(float val = 0.0f, MotorCommandType t = MotorCommandType::VOLTAGE)
        : value(val), type(t) {}
};

inline constexpr MotorCommand operator-(MotorCommand cmd) {
    return MotorCommand(-cmd.value, cmd.type);
}   

using velocity = MotorCommand;
using voltage = MotorCommand;

inline constexpr MotorCommand operator"" mV(long double val) {
    return MotorCommand(static_cast<float>(val), mV);
}
inline constexpr MotorCommand operator"" mV(unsigned long long val) {
    return MotorCommand(static_cast<float>(val), mV);
}

inline constexpr MotorCommand operator"" rpm(long double val) {
    return MotorCommand(static_cast<float>(val), rpm);
}
inline constexpr MotorCommand operator"" rpm(unsigned long long val) {
    return MotorCommand(static_cast<float>(val), rpm);
}

namespace miku {

class AbstractMotor : public pros::Motor {
protected:
    int ticks_per_rev;

    float prev_ticks = 0;
    uint32_t last_time = 0;

    float sma_filter_size = 3;
    float median_filter_size = 1;
    float accel_filter_size = 1;
    std::vector<float> prev_raw_velocities;
    std::vector<float> prev_filtered_velocities;
    std::vector<float> prev_accels;
    float prev_estimated_velocity = 0;

public:
    AbstractMotor(std::int8_t port, pros::v5::MotorGears gearset = pros::v5::MotorGears::blue, 
                pros::v5::MotorUnits encoder_units = pros::v5::MotorUnits::degrees);
    virtual float get_filtered_velocity();
    virtual ~AbstractMotor() = default;
};

class MotorController {
protected:
    PID velocity_pid;
    LookupTable voltage_lut;
    float max_voltage = 12000;

    int last_commanded_voltage = 0;
    int last_commanded_velocity = 0;
public:
    MotorController(PIDGains pid_gains, LookupTable voltage_lookup_table);
    virtual ~MotorController() = default;
    virtual void move_velocity(float velocity) = 0;
    virtual float get_commanded_voltage() { return last_commanded_voltage; }
    virtual float get_commanded_velocity() { return last_commanded_velocity; }
};

class Motor : public AbstractMotor, public MotorController {

public:
Motor(std::int8_t port, pros::v5::MotorGears gearset = pros::v5::MotorGears::blue, 
      pros::v5::MotorUnits encoder_units = pros::v5::MotorUnits::degrees,
      LookupTable voltage_lookup_table = LookupTable(), PIDGains pid_gains = PIDGains());

void move_velocity(float velocity) override;

void move(int voltage) { pros::Motor::move(voltage); }
void move(MotorCommand cmd) {
    if(cmd.type == MotorCommandType::VOLTAGE) {
        move_voltage(cmd.value);
    } else {
        move_velocity(cmd.value);
    }
}
void move_voltage(int32_t voltage) { 
    pros::Motor::move_voltage(voltage);
    last_commanded_voltage = voltage;
}
float get_filtered_velocity() { return AbstractMotor::get_filtered_velocity(); }

};

class MotorGroup : public MotorController {

private:
std::vector<AbstractMotor*> motors;

public:
MotorGroup() = default;
MotorGroup(const std::initializer_list<std::int8_t> ports, 
           const pros::v5::MotorGears gearset = pros::v5::MotorGears::blue,
           const pros::v5::MotorUnits encoder_units = pros::v5::MotorUnits::degrees,
           LookupTable voltage_lookup_table = LookupTable(), PIDGains pid_gains = PIDGains());

void move_velocity(float velocity) override;

// Uses get_filtered_velocity() to return the average velocity of the group
float get_average_velocity();
float get_average_position();
float get_average_torque();

void move(int voltage);
void move(MotorCommand cmd) {
    if(cmd.type == MotorCommandType::VOLTAGE) {
        move_voltage(cmd.value);
    } else {
        move_velocity(cmd.value);
    }
}
void move_voltage(int32_t voltage);
void brake();
void tare_position(void) const;
void set_brake_mode(pros::motor_brake_mode_e mode);
int get_highest_temperature() const;
std::vector<int> get_temperature_all() const;

};

} // namespace miku