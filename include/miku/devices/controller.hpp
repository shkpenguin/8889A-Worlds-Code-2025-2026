#pragma once

#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include <functional>
#include <set>
#include <queue>
#include <string>

#define CONTROLLER_UPDATE_RATE_MS 100

namespace miku {

struct Alert {
    std::string message;
    std::string rumble;
    int time;
};

class Controller : public pros::Controller {
private:
    std::vector<Alert*> alert_queue;
    Alert* current_alert = nullptr;

    std::string rumble_queue;
    
    std::vector<std::function<std::string()>> display_functions = {
        nullptr,
        nullptr,
        nullptr
    };
    std::vector<std::function<std::string()>> display_alerts = {
        nullptr,
        nullptr,
        nullptr
    };
    std::vector<std::function<std::string()>>* current_display_function = &display_functions;

    std::set<int> saved_active_lines;
    std::set<int> active_lines;
    int interval_ms = 1000;
    bool rumble_on = false;
    int32_t prev_update_time = 0;
    int32_t prev_rumble_time = 0;
    int32_t prev_alert_time = 0;
    int display_update_count = 0;
public:
    using pros::Controller::Controller;

    void update_display();
    void display(std::uint8_t line, std::function<std::string()> func);
    void set_text(std::uint8_t line, std::string text);
    void remove(std::uint8_t line);
    void rumble(std::string pattern);
    void toggle_rumble();
    void set_rumble(bool on);
    void set_rumble_interval(int interval_ms);
    void alert(std::string message, int time, std::string rumble = "");

};

} // namespace miku