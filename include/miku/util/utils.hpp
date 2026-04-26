#pragma once

#include <cmath>
#include "miku/util/geometry.hpp"

#define HALF_FIELD 71.0

inline float clamp(float value, float min, float max) {
    if(value < min) return min;
    if(value > max) return max;
    return value;
}

inline float sign(float value) {
    return (value > 0) - (value < 0);
}

inline float dist(float x1, float y1, float x2, float y2) {
    return std::hypot(x2 - x1, y2 - y1);
}

inline float ema(float current, float previous, float alpha) {
    return alpha * current + (1 - alpha) * previous;
}

inline int find_quadrant(Pose robot_pose) {
    if(robot_pose.x >= 0 && robot_pose.y >= 0) return 1;
    if(robot_pose.x < 0 && robot_pose.y >= 0) return 2;
    if(robot_pose.x < 0 && robot_pose.y < 0) return 3;
    return 4;
}

// bound value between 0 and mod-1
inline void increment_mod(int& value, int mod) {
    value = (value + 1) % mod;
}

inline void decrement_mod(int& value, int mod) {
    value = (value - 1 + mod) % mod;
}

namespace miku {
    inline standard_radians arctan(float y, float x) {
        return standard_radians(std::atan2(y, x));
    }
}

template<typename T>
class List {
    std::vector<T> values;
    int size;
    int index = 0;
    public:
    List(std::vector<T> vals) : values(vals), size(vals.size()) {}
    List(std::initializer_list<T> vals) : values(vals), size(vals.size()) {}
    T cycle_forward() {
        index = (index + 1) % size;
        return values[index];
    }
    T cycle_reverse() {
        index = (index - 1 + size) % size;
        return values[index];
    }
    T get_value() const {
        return values[index];
    }
};

inline float slew(float current, float previous, float max_change) {
    float change = current - previous;
    if (max_change == 0) return current;
    if (change > max_change) change = max_change;
    else if (change < -max_change) change = -max_change;
    return previous + change;
}