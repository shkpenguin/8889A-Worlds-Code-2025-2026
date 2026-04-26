#include "miku/devices/controller.hpp"
#include "miku/util/utils.hpp"
#include <stdexcept>

void miku::Controller::display(std::uint8_t line, std::function<std::string()> func) {
    if(line > 2) throw std::out_of_range("Line number out of range");
    display_functions[line] = func;
    active_lines.insert(line);
}

void miku::Controller::set_text(std::uint8_t line, std::string text) {
    if(line > 2) throw std::out_of_range("Line number out of range");
    std::function<std::string()> func = [text]() {
        return text;
    };
    display_functions[line] = func;
    active_lines.insert(line);
}

void miku::Controller::remove(std::uint8_t line) {
    if(line < 0 || line > 2) throw std::out_of_range("Line number out of range");
    display_functions[line] = nullptr;
    active_lines.erase(line);
}

void miku::Controller::rumble(std::string pattern) {
    rumble_queue += pattern;
}

void miku::Controller::toggle_rumble() {
    rumble_on = !rumble_on;
}

void miku::Controller::set_rumble(bool on) {
    rumble_on = on;
}

void miku::Controller::set_rumble_interval(int interval_ms) {
    this->interval_ms = interval_ms;
}

void miku::Controller::alert(std::string message, int time, std::string rumble) {
    Alert* alert = new Alert{message, rumble, time};
    alert_queue.push_back(alert);
}

// the controller can only process one command(set_text or rumble) at a time so we need to alternate
void miku::Controller::update_display() {

    // update times
    int32_t current_time = pros::millis();
    if(current_time - prev_update_time < CONTROLLER_UPDATE_RATE_MS) return;

    if(rumble_on && current_time - prev_rumble_time >= interval_ms) {
        prev_rumble_time = current_time;
        prev_update_time = current_time;
        pros::Controller::rumble("-");
        return;
    }
    else if(!rumble_queue.empty()) {
        char rumble_char = rumble_queue.front();
        rumble_queue.erase(0, 1);
        char rumble_str[2] = {rumble_char, '\0'};
        pros::Controller::rumble(rumble_str);
        return;
    }

    // alert handling
    if(!alert_queue.empty() && !current_alert) {
        current_alert = alert_queue.front();
        alert_queue.erase(alert_queue.begin());
        display_alerts[0] = [text = current_alert->message]() {
            return "                   ";
        };
        display_alerts[1] = [text = current_alert->message]() {
            return text;
        };
        display_alerts[2] = [text = current_alert->message]() {
            return "                   ";
        };
        saved_active_lines = active_lines;
        active_lines.insert({0, 1, 2});
        current_display_function = &display_alerts;
        rumble(current_alert->rumble);
        prev_alert_time = current_time;
    } else if(current_alert) {
        if(current_time - prev_alert_time >= current_alert->time) {
            delete current_alert;
            current_alert = nullptr;
            current_display_function = &display_functions;
            active_lines = saved_active_lines;
        }
    }

    prev_update_time = current_time;
    if(!active_lines.empty()) {
        increment_mod(display_update_count, active_lines.size());
        auto it = active_lines.begin();
        std::advance(it, display_update_count);
        int line = *it;
        if((*current_display_function)[line]) {
            std::string text = (*current_display_function)[line]() + "                ";
            pros::Controller::set_text(line, 0, text.c_str());
        }
    }

}