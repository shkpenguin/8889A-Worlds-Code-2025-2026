#pragma once
#include "pros/distance.hpp"
#include "miku/util/geometry.hpp"
#include <cmath>

// Sensor orientation is now stored as an angle (radians):
//  0 = forward (+Y), positive = CCW


namespace miku {
class Distance : public pros::Distance {
    float data = -1;
    bool valid = false;
    bool enabled = true;
    pros::Mutex mutex;
public:
    uint8_t id;
    float offset_x, offset_y;

    float orientation_angle = 0.0f;
    float cos_orient = 1.0f;
    float sin_orient = 0.0f;

    Distance(const uint8_t id, const std::uint8_t port, float offset_x, float offset_y, float orientation_angle)
        : pros::Distance(port), id(id), offset_x(offset_x), offset_y(offset_y) {
        set_orientation_angle(orientation_angle);
        // add_system_port(port); // Uncomment if system port management is needed
    }

    void set_orientation_angle(compass_degrees offset) {
        orientation_angle = offset.radians();
        cos_orient = std::cos(orientation_angle);
        sin_orient = std::sin(orientation_angle);
    }

    void set_enabled(bool enable) { enabled = enable; }
    void update_reading();
    bool get_enabled() { return enabled; }
    bool get_valid() { return valid; }
    float get_reading() { return data; }
};
}