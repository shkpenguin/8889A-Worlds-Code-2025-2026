#pragma once

#include "miku/util/geometry.hpp"

Pose get_pose();
Pose get_pose_delta();
Point get_position();
Point get_local_velocity();
float get_x();
float get_y();
standard_radians get_heading();
void set_x(float x);
void set_y(float y);
void set_heading(AngleTemplate<> heading);
void set_pose(Pose new_pose);
void set_position(Point new_position);
void reset(Pose new_pose);
void reset(Point new_position);

void distance_reset(Point estimate, float particle_spread = 0.0f);
void distance_reset(float particle_spread = 0.0f);

void set_max_distance_error(float error);
void set_min_odom_noise(float noise);
void set_max_sensor_stdev(float stdev);

void update_position();

void add_field_object(Shape* obj);