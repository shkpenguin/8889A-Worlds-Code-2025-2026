#pragma once

#include "pros/adi.hpp"

namespace miku {
class Pneumatic : public pros::adi::DigitalOut {
bool state;
public:
    Pneumatic(uint8_t port) : pros::adi::DigitalOut(port), state(false) {}
    void set_value(bool value) {
        state = value;
        pros::adi::DigitalOut::set_value(value);
    }
    void toggle() {
        state = !state;
        pros::adi::DigitalOut::set_value(state);
    }
    bool get_value() const {
        return state;
    }
};
} // namespace miku