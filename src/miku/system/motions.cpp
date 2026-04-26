#include "config.h"
#include "miku/system/motions.hpp"
#include "miku/system/drive.hpp"
#include "miku/util/pid.hpp"
#include "miku/util/utils.hpp"
#include "miku/util/exit.hpp"
#include <cmath>

using namespace miku;

PIDGains drive_gains = LINEAR_PID_GAINS;
PIDGains turn_gains = ANGULAR_PID_GAINS;

MoveDistance::MoveDistance(float target, float timeout, MoveDistanceParams params)
    : target(target), timeout(timeout), params(params), use_point(false) {
    PIDGains gains = drive_gains;
    if(params.kP > 0) gains.kP = params.kP;
    if(params.kI > 0) gains.kI = params.kI;
    if(params.kD > 0) gains.kD = params.kD;
    drive_pid = PID(gains, true, true);
}

MoveDistance::MoveDistance(Point target_point, float timeout, MoveDistanceParams params)
    : target(0), timeout(timeout), params(params), use_point(true), target_point(target_point) {
    // distance will be computed in start() using current pose
    PIDGains gains = drive_gains;
    if(params.kP > 0) gains.kP = params.kP;
    if(params.kI > 0) gains.kI = params.kI;
    if(params.kD > 0) gains.kD = params.kD;
    drive_pid = PID(gains, true, true);
}

void MoveDistance::start() {
    done = false;
    if(use_point) {
        Point current = get_position();
        float dx = target_point.x - current.x;
        float dy = target_point.y - current.y;
        target = std::hypot(dx, dy);
        if(params.reverse) target = -target;
    }
    start_time = pros::millis();
    motors_start_pos = get_drive_position();
    
    timer.set(timeout);
    timer.reset();
    drive_pid.reset();
    drive_exit = params.quick_exit ? DRIVE_QUICK_EXIT : DRIVE_SLOW_EXIT;
    if (drive_exit.min_threshold > fabs(target) / 2.0f) drive_exit.min_threshold = static_cast<int>(std::ceil(fabs(target) / 2.0f));
    drive_exit.reset();
}

void MoveDistance::update() {
    if(timer.is_done()) {
        done = true;
        stop_drive();
        return;
    }

    Point motors_current_pos = get_drive_position();
    float start_avg_pos = (motors_start_pos.x + motors_start_pos.y) / 2.0f;
    float current_avg_pos = (motors_current_pos.x + motors_current_pos.y) / 2.0f;
    float dist_traveled_in = (current_avg_pos - start_avg_pos) * WHEEL_DIAMETER * GEAR_RATIO * M_PI / 360.0f;

    float error = (params.reverse ? -1 : 1) * (target - dist_traveled_in);

    // cutoff g
    if(params.cutoff > 0 && fabs(error) < params.cutoff) {
        done = true;
        stop_drive();
        return;
    }

    // finished if we've passed the target or are within half an inch
    if ((error < 0) != (target < 0)) {
        done = true;
        stop_drive();
        return;
    }

    drive_exit.update(fabs(error));

    // use PID on distance error to compute drive voltage
    float drive_out = drive_pid.update(error);
    float limit = params.max_volt_pct / 100.0f * 12000;
    drive_out = std::clamp(drive_out, -limit, limit);
    if(params.min_volt_pct > 0) {
        float minv = params.min_volt_pct / 100.0f * 12000;
        if(drive_out > 0 && drive_out < minv) drive_out = minv;
        if(drive_out < 0 && drive_out > -minv) drive_out = -minv;
    }
    move_voltage(drive_out, drive_out);

    if (drive_exit.get_exit()) {
        done = true;
        stop_drive();
        return;
    }
}

void MoveDistance::stop() {
    done = true;
    stop_drive();
}

bool MoveDistance::is_done() {
    return done;
}

MovePoint::MovePoint(Point target, float timeout, MovePointParams params)
    : target(target), timeout(timeout), params(params) {
    PIDGains mvt_drive_gains;
    mvt_drive_gains.kP = (params.drive_kP > 0) ? params.drive_kP : drive_gains.kP;
    mvt_drive_gains.kI = (params.drive_kI > 0) ? params.drive_kI : drive_gains.kI;
    mvt_drive_gains.kD = (params.drive_kD > 0) ? params.drive_kD : drive_gains.kD;
    drive_pid = PID(mvt_drive_gains, true, true);

    PIDGains mvt_turn_gains;
    mvt_turn_gains.kP = (params.turn_kP > 0) ? params.turn_kP : turn_gains.kP;
    mvt_turn_gains.kI = (params.turn_kI > 0) ? params.turn_kI : turn_gains.kI;
    mvt_turn_gains.kD = (params.turn_kD > 0) ? params.turn_kD : turn_gains.kD;
    turn_pid = PID(mvt_turn_gains, true, true);
}

void MovePoint::start() {
    standard_radians angle_to_target = miku::arctan(target.y - get_y(), target.x - get_x()).wrap();
    if(params.straight && fabs((angle_to_target - get_heading()).degrees()) > 5.0) {
        auto align_heading = new TurnPoint(target, 500, {.reverse = params.reverse, .cutoff = 5.0});
        queue_after_current(this);
        queue_after_current(align_heading);
        params.straight = false;
        done = true;
        return;
    }
    done = false;
    start_time = pros::millis();
    drive_pid.reset();
    turn_pid.reset();
    timer.set(timeout);
    timer.reset();

    Point current = get_position();
    drive_exit = params.quick_exit ? DRIVE_QUICK_EXIT : DRIVE_SLOW_EXIT;
    float target_distance = current.distance_to(target);
    if (drive_exit.min_threshold > target_distance / 2.0f) drive_exit.min_threshold = static_cast<int>(std::ceil(target_distance / 2.0f));
    drive_exit.reset();

    start_point = current; // record where the motion started

    float dx = target.x - current.x;
    float dy = target.y - current.y;
    start_side = sign(dx * cos(get_heading()) + dy * sin(get_heading()));
}

void MovePoint::update() {
    Point current = get_position();

    // Calculate angle error in compass frame (0 = +Y)
    float dx = target.x - current.x;
    float dy = target.y - current.y;
    compass_degrees angle_to_point = compass_degrees(miku::arctan(dy, dx));
    if (params.reverse) angle_to_point = (angle_to_point + 180.0f).wrap();

    float current_deg = compass_degrees(get_heading()).wrap();

    float turn_error = (angle_to_point - current_deg).wrap();
    // Distance error
    float drive_error = current.distance_to(target);
    bool close = fabs(drive_error) < 6.0;
    if (params.cutoff > 0 && drive_error < params.cutoff) {
        done = true;
        return;
    }

    standard_radians angle_error = (miku::arctan(dy, dx) - get_heading()).wrap();

    drive_exit.update(drive_error);
    float drive_out = std::clamp(drive_pid.update(drive_error), -params.drive_max_volt_pct / 100.0f * 12000, params.drive_max_volt_pct / 100.0f * 12000);

    // PID outputs
    float turn_out = std::clamp(turn_pid.update(turn_error), -params.turn_max_volt_pct / 100.0f * 12000, params.turn_max_volt_pct / 100.0f * 12000);
    float drive_scale = std::cos(angle_error);
    if(sign(drive_scale) != sign(std::pow(drive_scale, params.cos_scale))) drive_out *= -1.0 * std::pow(drive_scale, params.cos_scale);
    else drive_out *= std::pow(drive_scale, params.cos_scale);

    if(!close) drive_out = slew(drive_out, prev_drive_out, params.slew);
    prev_drive_out = drive_out;

    if(params.volt_exit_pct > 0) {
        float exit_threshold = params.volt_exit_pct / 100.0f * 12000;
        if(fabs(drive_out) < exit_threshold && fabs(drive_error) < params.volt_exit_range) {
            done = true;
            stop_drive();
            return;
        }
    }

    if(close) {
        int side = sign(dx * cos(get_heading()) + dy * sin(get_heading()));
        if(side != start_side) {
            done = true;
            stop_drive();
            return;
        }
        turn_out *= pow((drive_error / 6.0), 4);
    }

    if(params.min_volt_pct > 0) {
        if(drive_out > 0 && drive_out < fabs(params.min_volt_pct / 100.0f * 12000)) drive_out = params.min_volt_pct / 100.0f * 12000;
        if(drive_out < 0 && drive_out > -fabs(params.min_volt_pct / 100.0f * 12000)) drive_out = -params.min_volt_pct / 100.0f * 12000;
    }

    float left_out = drive_out + turn_out;
    float right_out = drive_out - turn_out;

    // Final motor outputs
    move_voltage(left_out, right_out);

    if (drive_exit.get_exit() || timer.is_done()) {
        done = true;
        stop_drive();
        return;
    }
}

void MovePoint::stop() {
    done = true;
    if(params.cutoff < 0) {
        stop_drive();
    }
}

bool MovePoint::is_done() {
    return done;
}

MoveTime::MoveTime(MotorCommand left_speed, MotorCommand right_speed, float duration)
        : left_speed(left_speed), right_speed(right_speed), duration(duration) {}

void MoveTime::start() {
    done = false;
    start_time = pros::millis();
    timer.set(duration);
    timer.reset();
    move_drive(left_speed, right_speed);
}

void MoveTime::update() {
    if (timer.is_done()) {
        done = true;
        stop_drive();
        return;
    }
    move_drive(left_speed, right_speed);
}

void MoveTime::stop() {
    done = true;
    stop_drive();
}

bool MoveTime::is_done() {
    return done;
}

/*
 * Polar motion controller designed in Control of Wheeled Mobile Robots: An Experimental Overview
 * by Alessandro De Luca, Giuseppe Oriolo, Marilena Vendittelli, section 6.3
*/
MovePose::MovePose(Point target, compass_degrees heading, float timeout, MovePoseParams params)
    : target(target), target_heading(standard_radians(heading)), timeout(timeout), params(params) {
    if(params.k1 < 0) k1 = 4.0;
    if(params.k2 < 0) k2 = 6.0;
    if(params.k3 < 0) k3 = 3.0;
    if(params.reverse) target_heading = (target_heading + M_PI).wrap();
    if(params.vel_exit_pct > 0) params.end_cutoff = 0;
}

MovePose::MovePose(Point target, float timeout, MovePoseParams params)
    : target(target), target_heading(0), timeout(timeout), params(params) {
    if(params.k1 < 0) k1 = 4.0;
    if(params.k2 < 0) k2 = 6.0;
    k3 = 0;
    if(params.reverse) target_heading = (target_heading + M_PI).wrap();
}

void MovePose::start() {
    standard_radians angle_to_target = miku::arctan(target.y - get_y(), target.x - get_x());
    if(params.straight && fabs((angle_to_target - get_heading().wrap()).degrees()) > 10.0deg) {
        auto align_heading = new TurnPoint(target, 500, {.reverse = params.reverse, .cutoff = 10.0});
        queue_after_current(this);
        queue_after_current(align_heading);
        params.straight = false;
        done = true;
        return;
    }
    done = false;
    start_time = pros::millis();
    start_point = get_position(); // record start location for `away()`/events
    timer.set(timeout);
    timer.reset();
}

void MovePose::update() {

    if(done) return;
    
    float drive_error = get_pose().distance_to(target);

    standard_radians robot_heading = get_heading().wrap();
    if (params.reverse) robot_heading = (robot_heading + M_PI).wrap();

    standard_radians angle_to_target = miku::arctan(target.y - get_y(), target.x - get_x());
    float gamma = (angle_to_target - robot_heading).wrap();
    float delta = (angle_to_target - target_heading).wrap();

    float v = k1 * drive_error * std::cos(gamma);
    float w;
    if (std::abs(gamma) <= 0.01) {
        w = k2 * gamma + k1 * std::cos(gamma) * (gamma + k3 * delta);
    } else {
        w = k2 * gamma + k1 * std::sin(gamma) * std::cos(gamma) / gamma * (gamma + k3 * delta);
    }

    v = std::clamp(v, -params.max_vel_pct / 100.0f * MAX_VEL, params.max_vel_pct / 100.0f * MAX_VEL);
    w = std::clamp(w, -params.max_vel_pct / 100.0f * MAX_ANG_VEL, params.max_vel_pct / 100.0f * MAX_ANG_VEL);

    if(params.min_vel_pct > 0 && v > 0 && v < fabs(params.min_vel_pct / 100.0f * MAX_VEL)) v = params.min_vel_pct / 100.0f * MAX_VEL;
    float r_vel = v + TRACK_WIDTH * w / 2;
    float l_vel = v - TRACK_WIDTH * w / 2;

    float l_rpm = (l_vel * 60) / (WHEEL_CIRC * GEAR_RATIO);
    float r_rpm = (r_vel * 60) / (WHEEL_CIRC * GEAR_RATIO);

    if(params.vel_exit_pct > 0) {
        if(fabs(v * 100 / MAX_VEL) < params.vel_exit_pct && drive_error < params.vel_exit_range) {
            done = true;
            return;
        }
    }

    if (params.reverse) {
        move_velocity(-r_rpm, -l_rpm);
    } else {
        move_velocity(l_rpm, r_rpm);
    }

    if(timer.is_done()) {
        done = true;
        stop_drive();
        return;
    }

    if((params.cutoff > 0 && drive_error < params.cutoff)) {
        done = true;
        stop_drive();
        return;
    }

    if(drive_error < params.end_cutoff) {
        MovePointParams new_params;
        new_params.reverse = params.reverse;
        new_params.quick_exit = params.quick_exit;
        auto settle_motion = new MovePoint(
            Point(target.x, target.y), 
            timer.get_time_left(),
            new_params
        );

        if(!done) queue_after_current(settle_motion);

        done = true;
        stop_drive();
        return;
    }
}

void MovePose::stop() {
    done = true;
    if(params.cutoff < 0) {
        stop_drive();
    }
}

bool MovePose::is_done() {
    return done;
}

TurnHeading::TurnHeading(compass_degrees target, float timeout, TurnParams params)
    : target(target), timeout(timeout), params(params) {
    PIDGains gains;
    gains.kP = (params.kP > 0) ? params.kP : turn_gains.kP;
    gains.kI = (params.kI > 0) ? params.kI : turn_gains.kI;
    gains.kD = (params.kD > 0) ? params.kD : turn_gains.kD;
    turn_pid = PID(gains);
}

void TurnHeading::start() {
    done = false;
    start_time = pros::millis();
    turn_pid.reset();
    timer.set(timeout);
    timer.reset();
    turn_exit = params.quick_exit ? TURN_QUICK_EXIT : TURN_SLOW_EXIT;
    float current_deg = compass_degrees(get_heading());
    float initial_angle = fabs((target - current_deg).wrap());
    if (turn_exit.min_threshold > initial_angle / 2.0f) turn_exit.min_threshold = static_cast<int>(std::ceil(initial_angle / 2.0f));
    turn_exit.reset();
}

void TurnHeading::update() {
    compass_degrees current_deg = compass_degrees(get_heading());  // convert to degrees
    float error = (target - current_deg).wrap();      // error in degrees

    turn_exit.update(error);
    if (timer.is_done() || turn_exit.get_exit()) {
        done = true;
        stop_drive();
        return;
    }

    if (params.cutoff > 0 && fabs(error) < params.cutoff) {
        done = true;
        return;
    }

    float output = turn_pid.update(error);
    output = std::clamp(output, -params.max_volt_pct / 100.0f * 12000, params.max_volt_pct / 100.0f * 12000);
    if(params.min_volt_pct > 0) {
        if(output > 0 && output < fabs(params.min_volt_pct / 100.0f * 12000)) output = params.min_volt_pct / 100.0f * 12000;
        if(output < 0 && output > -fabs(params.min_volt_pct / 100.0f * 12000)) output = -params.min_volt_pct / 100.0f * 12000;
    }

    move_voltage(output, -output);
}

void TurnHeading::stop() {
    done = true;
    if(params.cutoff < 0) {
        stop_drive();
    }
}

bool TurnHeading::is_done() {
    return done;
}

TurnPoint::TurnPoint(Point target, float timeout, TurnParams params)
    : target(target), timeout(timeout), params(params) {
    PIDGains gains;
    gains.kP = (params.kP > 0) ? params.kP : turn_gains.kP;
    gains.kI = (params.kI > 0) ? params.kI : turn_gains.kI;
    gains.kD = (params.kD > 0) ? params.kD : turn_gains.kD;
    turn_pid = PID(gains, true, true);
}

void TurnPoint::start() {
    done = false;
    original_target_deg = compass_degrees(miku::arctan(target.y - get_y(), target.x - get_x()));
    start_time = pros::millis();
    start_point = get_position(); // record start location for `away()`
    turn_pid.reset();
    timer.set(timeout);
    timer.reset();
    compass_degrees current_deg = compass_degrees(get_heading());
    compass_degrees target_deg = compass_degrees(miku::arctan(target.y - get_y(), target.x - get_x()));
    if(params.reverse) target_deg = (target_deg + 180.0f).wrap();
    turn_exit = params.quick_exit ? TURN_QUICK_EXIT : TURN_SLOW_EXIT;
    float initial_angle = fabs((target_deg - current_deg).wrap());
    if (turn_exit.min_threshold > initial_angle / 2.0f) turn_exit.min_threshold = static_cast<int>(std::ceil(initial_angle / 2.0f));
    turn_exit.reset();
}

void TurnPoint::update() {
    compass_degrees current_deg = compass_degrees(get_heading());  // convert to degrees
    compass_degrees target_deg = compass_degrees(miku::arctan(target.y - get_y(), target.x - get_x()));
    if(params.reverse) target_deg = (target_deg + 180.0f).wrap();
    compass_degrees error = (target_deg - current_deg).wrap();      // error in degrees

    turn_exit.update(error);
    if (timer.is_done() || turn_exit.get_exit()) {
        done = true;
        stop_drive();
        return;
    }

    if (params.cutoff > 0 && fabs(error) < params.cutoff) {
        done = true;
        return;
    }

    float output;
    if(fabs((target_deg - original_target_deg).wrap()) > 1.0) {
        output = turn_pid.update(error, 0);
        original_target_deg = target_deg;
    }
    else output = turn_pid.update(error);
    // output = turn_pid.update(error);
    output = std::clamp(output, -params.max_volt_pct / 100.0f * 12000, params.max_volt_pct / 100.0f * 12000);
    if(params.min_volt_pct > 0) {
        if(output > 0 && output < fabs(params.min_volt_pct / 100.0f * 12000)) output = params.min_volt_pct / 100.0f * 12000;
        if(output < 0 && output > -fabs(params.min_volt_pct / 100.0f * 12000)) output = -params.min_volt_pct / 100.0f * 12000;
    }

    move_voltage(output, -output);
}

void TurnPoint::stop() {
    done = true;
    if(params.cutoff < 0) {
        stop_drive();
    }
}

bool TurnPoint::is_done() {
    return done;
}

SwingHeading::SwingHeading(compass_degrees target, float timeout, SwingParams params)
    : target(target), timeout(timeout), params(params) {
    PIDGains gains;
    gains.kP = (params.kP > 0) ? params.kP : turn_gains.kP;
    gains.kI = (params.kI > 0) ? params.kI : turn_gains.kI;
    gains.kD = (params.kD > 0) ? params.kD : turn_gains.kD;
    turn_pid = PID(gains);
}

void SwingHeading::start() {
    done = false;
    start_time = pros::millis();
    turn_pid.reset();
    timer.set(timeout);
    timer.reset();
    turn_exit = params.quick_exit ? TURN_QUICK_EXIT : TURN_SLOW_EXIT;
    float current_deg = compass_degrees(get_heading());
    float initial_angle = fabs((target - current_deg).wrap());
    if (turn_exit.min_threshold > initial_angle / 2.0f) turn_exit.min_threshold = static_cast<int>(std::ceil(initial_angle / 2.0f));
    turn_exit.reset();
}

void SwingHeading::update() {
    compass_degrees current_deg = compass_degrees(get_heading());  // convert to degrees
    float error = (target - current_deg).wrap();      // error in degrees

    if (params.cutoff > 0 && fabs(error) < params.cutoff) {
        done = true;
        return;
    }

    float output = turn_pid.update(error);
    output = std::clamp(output, -params.max_volt_pct / 100.0f * 12000, params.max_volt_pct / 100.0f * 12000);
    if(params.min_volt_pct > 0) {
        if(output > 0 && output < fabs(params.min_volt_pct / 100.0f * 12000)) output = params.min_volt_pct / 100.0f * 12000;
        if(output < 0 && output > -fabs(params.min_volt_pct / 100.0f * 12000)) output = -params.min_volt_pct / 100.0f * 12000;
    }

    if(params.locked_side == Side::LEFT) {
        move_voltage(0, -output);
    } else {
        move_voltage(output, 0);
    }

    turn_exit.update(error);
    if (timer.is_done() || turn_exit.get_exit()) {
        done = true;
        return;
    }
}

void SwingHeading::stop() {
    done = true;
    if(params.cutoff < 0) {
        stop_drive();
    }
}

bool SwingHeading::is_done() {
    return done;
}

SwingPoint::SwingPoint(Point target, float timeout, SwingParams params)
    : target(target), timeout(timeout), params(params) {
    PIDGains gains;
    gains.kP = (params.kP > 0) ? params.kP : turn_gains.kP;
    gains.kI = (params.kI > 0) ? params.kI : turn_gains.kI;
    gains.kD = (params.kD > 0) ? params.kD : turn_gains.kD;
    turn_pid = PID(gains);
}

void SwingPoint::start() {
    done = false;
    start_time = pros::millis();
    start_point = get_position(); // record start location for `away()`
    turn_pid.reset();
    timer.set(timeout);
    timer.reset();
    compass_degrees current_deg = compass_degrees(get_heading());
    compass_degrees target_deg = compass_degrees(miku::arctan(target.y - get_y(), target.x - get_x()));
    if(params.reverse) target_deg = (target_deg + 180.0f).wrap();
    turn_exit = params.quick_exit ? TURN_QUICK_EXIT : TURN_SLOW_EXIT;
    float initial_angle = fabs((target_deg - current_deg).wrap());
    if (turn_exit.min_threshold > initial_angle / 2.0f) turn_exit.min_threshold = static_cast<int>(std::ceil(initial_angle / 2.0f));
    turn_exit.reset();
    prev_deg = compass_degrees(get_heading()).wrap();
}

void SwingPoint::update() {
    compass_degrees current_deg = compass_degrees(get_heading());  // convert to degrees
    compass_degrees target_deg = compass_degrees(miku::arctan(target.y - get_y(), target.x - get_x()));
    float error = (target_deg - current_deg).wrap();      // error in degrees

    if (params.cutoff > 0 && fabs(error) < params.cutoff) {
        done = true;
        return;
    }

    float output = turn_pid.update(error, -(current_deg - prev_deg).wrap());
    prev_deg = current_deg;
    output = std::clamp(output, -params.max_volt_pct / 100.0f * 12000, params.max_volt_pct / 100.0f * 12000);
    if(params.min_volt_pct > 0) {
        if(output > 0 && output < fabs(params.min_volt_pct / 100.0f * 12000)) output = params.min_volt_pct / 100.0f * 12000;
        if(output < 0 && output > -fabs(params.min_volt_pct / 100.0f * 12000)) output = -params.min_volt_pct / 100.0f * 12000;
    }

    if(params.locked_side == Side::LEFT) {
        move_voltage(0, -output);
    } else {
        move_voltage(output, 0);
    }

    turn_exit.update(error);
    if (timer.is_done() || turn_exit.get_exit()) {
        done = true;
        return;
    }
}

void SwingPoint::stop() {
    done = true;
    if(params.cutoff < 0) {
        stop_drive();
    }
}

bool SwingPoint::is_done() {
    return done;
}