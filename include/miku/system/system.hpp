#pragma once

#include <deque>
#include "miku/system/motions.hpp"
#include "pros/misc.h"
#include "miku/util/utils.hpp"
#include "miku/devices/controller.hpp"
#include "api.h"
#include <vector>

struct SystemWarning {
    std::function<bool()> condition;
    std::string message;
    bool done = true;
};

void queue_motion(MotionPrimitive* motion);
void queue_after_current(MotionPrimitive* motion);

void user_update_intake_teleop();
void user_update_drive_teleop();

void user_pre_auton();
void user_pre_teleop();

extern List<DriveMode> drive_modes;
extern List<std::function<void()>> display_modes;

struct Route {
    std::string name;
    Point start_position;
    compass_degrees start_heading;
    std::function<void()> queue;
    std::function<void()> setup;
    Route(std::string name, Point start_position, compass_degrees start_heading, std::function<void()> queue, std::function<void()> setup = nullptr)
        : name(name), start_position(start_position), start_heading(start_heading), queue(queue), setup(setup) {}
    Route() = default;
};
extern std::vector<Route> routes;
extern int selected_route;