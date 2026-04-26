#pragma once

#include "pros/optical.hpp"

struct Color {
    float min_hue;
    float max_hue;
};

#define TILE Color{40.0, 70.0} // Color of VEX field tiles
#define RED Color{0.0, 30.0} // Bright red VEX field element color
#define BLUE Color{180.0, 220.0} // Bright blue VEX field element color

namespace miku {
    class Optical : public pros::Optical {
        public:
        Optical(const std::uint8_t port) : pros::Optical(port) {
            // add_system_port(port);
        }
        bool get_color(const Color& color) {
            float hue = get_hue();
            return (hue >= color.min_hue && hue <= color.max_hue && get_proximity() > 100.0);
        }
        void initialize() {
            this->set_integration_time(5);
            this->set_led_pwm(100);
        }
    };
}