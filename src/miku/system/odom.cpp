#include "miku/system/drive.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "miku/devices/motor.hpp"
#include "miku/util/geometry.hpp"
#include "miku/util/pid.hpp"
#include <memory>

Pose robot_pose;
pros::Mutex pose_mutex;

bool use_particle_filtering = true;

void set_use_particle_filtering(bool use_pf) {
    use_particle_filtering = use_pf;
}

Pose get_pose() { return robot_pose; };
Point get_position() { return Point(robot_pose.x, robot_pose.y); };
float get_x() { return robot_pose.x; };
float get_y() { return robot_pose.y; };
standard_radians get_heading() { return robot_pose.theta; };
void set_x(float x) {
    pose_mutex.take();
    robot_pose.x = x;
    pose_mutex.give();
};
void set_y(float y) {
    pose_mutex.take();
    robot_pose.y = y;
    pose_mutex.give();
};
void set_heading(AngleTemplate<> heading) {
    pose_mutex.take();
    robot_pose.theta = standard_radians(heading);
    pose_mutex.give();
}
void set_pose(Pose new_pose) { 
    pose_mutex.take();
    robot_pose = new_pose;
    pose_mutex.give();
};
void set_position(Point new_position) { 
    pose_mutex.take();
    robot_pose.x = new_position.x;
    robot_pose.y = new_position.y;
    pose_mutex.give();
};