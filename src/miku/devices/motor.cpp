#include "main.h"
#include "miku/devices/motor.hpp"
#include "miku/util/utils.hpp"
#include <numeric>

miku::AbstractMotor::AbstractMotor(std::int8_t port, pros::v5::MotorGears gearset, pros::v5::MotorUnits encoder_units) 
    : pros::Motor(port, gearset, encoder_units) {
    if(gearset == pros::v5::MotorGears::red) {
        ticks_per_rev = 1800;
    } else if(gearset == pros::v5::MotorGears::green) {
        ticks_per_rev = 900;
    } else if(gearset == pros::v5::MotorGears::blue) {
        ticks_per_rev = 300;
    } else {
        std::cerr << "Invalid gearset for motor on port " << static_cast<int>(port) << std::endl;
    }
}

float miku::AbstractMotor::get_filtered_velocity() {

    uint32_t current_time;
    float current_ticks = this->get_raw_position(&current_time);

    float delta_ticks = current_ticks - prev_ticks;
    float delta_time = 5 * std::round((current_time - last_time) / 5.0); 

    float prev_raw_velocity = prev_raw_velocities.back();
    float prev_filtered_velocity = prev_filtered_velocities.back();

    if(delta_time <= 0) {
        return prev_raw_velocity;
    }

    float raw_velocity = (delta_ticks / ticks_per_rev) / (delta_time / 60000.0);
    if(std::fabs(raw_velocity) > 1000) {
        return prev_raw_velocity;
    }

    prev_raw_velocities.push_back(raw_velocity);
    if(prev_raw_velocities.size() > std::max(sma_filter_size, median_filter_size)) {
        prev_raw_velocities.erase(prev_raw_velocities.begin());
    }

    float sum = std::accumulate(prev_raw_velocities.begin(), prev_raw_velocities.end(), 0.0);
    float sma_velocity = sum / prev_raw_velocities.size();

    prev_filtered_velocities.push_back(sma_velocity);
    if(prev_filtered_velocities.size() > median_filter_size) {
        prev_filtered_velocities.erase(prev_filtered_velocities.begin());
    }

    float median_velocity = 0;
    if(prev_filtered_velocities.size() & 1) {
        median_velocity = prev_filtered_velocities[prev_filtered_velocities.size() / 2];
    } else {
        median_velocity = (prev_filtered_velocities[prev_filtered_velocities.size() / 2 - 1] + prev_filtered_velocities[prev_filtered_velocities.size() / 2]) / 2.0;
    }

    float accel = (sma_velocity - prev_filtered_velocity) / (delta_time / 1000.0);
    prev_accels.push_back(accel);
    if(prev_accels.size() > accel_filter_size) {
        prev_accels.erase(prev_accels.begin());
    }

    float filtered_accel;
    if(accel > 0) filtered_accel = *std::max_element(prev_accels.begin(), prev_accels.end());
    else filtered_accel = *std::min_element(prev_accels.begin(), prev_accels.end());
    filtered_accel = fabs(filtered_accel);

    float min_kA = 0.60;
    float max_kA = 1.00;
    float max_expected_accel = 12.0;

    if(fabs(filtered_accel) > max_expected_accel) filtered_accel *= max_expected_accel / fabs(filtered_accel);

    float kA = min_kA + (max_kA - min_kA) * (fabs(filtered_accel) / max_expected_accel);
    float ema_velocity = ema(median_velocity, prev_estimated_velocity, kA);

    prev_estimated_velocity = ema_velocity;
    prev_ticks = current_ticks;
    last_time = current_time;

    return ema_velocity;

}

miku::MotorController::MotorController(PIDGains pid_gains, LookupTable voltage_lookup_table) 
    : velocity_pid(pid_gains), voltage_lut(voltage_lookup_table) {}

miku::Motor::Motor(std::int8_t port, pros::v5::MotorGears gearset, 
      pros::v5::MotorUnits encoder_units,
      LookupTable voltage_lookup_table,
      PIDGains pid_gains) 
    : AbstractMotor(port, gearset, encoder_units),
      MotorController(pid_gains, voltage_lookup_table) {
        // add_system_port(port);
      }

void miku::Motor::move_velocity(float velocity) {
    float ff = voltage_lut.get_value(velocity);
    float error = velocity - get_filtered_velocity();
    float pid_output = velocity_pid.update(error);

    float total_voltage = ff + pid_output;
    total_voltage = clamp(total_voltage, -max_voltage, max_voltage);
    this->move_voltage(static_cast<int>(total_voltage));
    last_commanded_velocity = velocity;
}

miku::MotorGroup::MotorGroup(const std::initializer_list<std::int8_t> ports,
           const pros::v5::MotorGears gearset,
           const pros::v5::MotorUnits encoder_units,
           LookupTable voltage_lookup_table,
           PIDGains pid_gains) : MotorController(pid_gains, voltage_lookup_table) {
    for(auto port : ports) {
        motors.push_back(new AbstractMotor(port, gearset, encoder_units));
        // add_system_port(port);
    }
}

void miku::MotorGroup::move_velocity(float velocity) {
    float ff = voltage_lut.get_value(velocity);
    float error = velocity - get_average_velocity();
    float pid_output = velocity_pid.update(error);

    float total_voltage = ff + pid_output;
    clamp(total_voltage, -max_voltage, max_voltage);
    this->move_voltage(static_cast<int>(total_voltage));
    last_commanded_velocity = velocity;
}

void miku::MotorGroup::move(int voltage) {
    for(auto motor : motors) {
        motor->move(voltage);
    }
}

void miku::MotorGroup::move_voltage(int32_t voltage) {
    for(auto motor : motors) {
        motor->move_voltage(voltage);
    }
    last_commanded_voltage = voltage;
}

void miku::MotorGroup::brake() {
    for(auto motor : motors) {
        motor->brake();
    }
}

float miku::MotorGroup::get_average_velocity() {
    float sum = 0;
    for(auto motor : motors) {
        sum += motor->get_filtered_velocity();
    }
    return sum / motors.size();
}

float miku::MotorGroup::get_average_position() {
    float sum = 0;
    for(auto motor : motors) {
        sum += motor->get_position();
    }
    return sum / motors.size();
}

float miku::MotorGroup::get_average_torque() {
    float sum = 0;
    for(auto motor : motors) {
        sum += motor->get_torque();
    }
    return sum / motors.size();
}

int miku::MotorGroup::get_highest_temperature() const {
    int highest_temp = 0;
    for(auto motor : motors) {
        int temp = motor->get_temperature();
        if(temp > highest_temp) {
            highest_temp = temp;
        }
    }
    return highest_temp;
}
    
void miku::MotorGroup::tare_position(void) const {
    for(auto motor : motors) {
        motor->tare_position();
    }
}

void miku::MotorGroup::set_brake_mode(pros::motor_brake_mode_e mode) {
    for(auto motor : motors) {
        motor->set_brake_mode(mode);
    }
}

std::vector<int> miku::MotorGroup::get_temperature_all() const {
    std::vector<int> temperatures;
    for(auto motor : motors) {
        temperatures.push_back(motor->get_temperature());
    }
    return temperatures;
}