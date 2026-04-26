#include "main.h"
#include "miku/system/system.hpp"
#include "config.h"
#include "miku/devices/intake.hpp"
#include "miku/system/motions.hpp"
#include "miku/system/log.hpp"

pros::Task* setup_task = nullptr;
pros::Task* autonomous_system_task = nullptr; 
pros::Task* teleop_drive_task = nullptr; 
pros::Task* teleop_intake_task = nullptr;
std::deque<MotionPrimitive*> motion_queue;
pros::Mutex queue_mutex;
MotionPrimitive* current_motion = nullptr;

uint32_t curr_dt = 0;
uint32_t prev_ms = 0;

uint32_t system_start_time = 0;
bool autonomous_run = false;

void queue_motion(MotionPrimitive* motion) {
    queue_mutex.take();
    motion_queue.push_back(motion);
    queue_mutex.give();
}

void queue_after_current(MotionPrimitive* motion) {
    queue_mutex.take();
    if (current_motion == nullptr) {
        motion_queue.push_back(motion);
    } else {
        motion_queue.insert(motion_queue.begin(), motion);
    }
    queue_mutex.give();
}

inline void display_battery_time() {
    master.display(0, []() {
        return fmt::format("battery: {}%", pros::battery::get_capacity());
    });
    master.display(1, []() {
        int seconds = (pros::millis() - system_start_time) / 1000;
        int minutes = seconds / 60;
        seconds %= 60;

        std::string s = fmt::format("{:02}:{:02}", minutes, seconds);
        return fmt::format("time: {}", s);
    });
	master.display(2, []() {
        int temp = std::max(get_drive_temperature().x, get_drive_temperature().y);
        return "drive: " + std::to_string(temp) + "C";
    });
}

inline void display_pose() {
    master.display(0, []() {
        return fmt::format("queuelen: {}", motion_queue.size());
    });
    master.display(1, []() {
        return get_pose().to_string();
    });
}

inline void display_setup() {
    master.display(0, []() {
        return "ROUTE: " + routes[selected_route].name;
    });
    master.display(1, []() {
        return "heading: " + std::to_string(get_imu_raw_heading().wrap()) + "°";
    });
}

void system_loop_autonomous() {

    prev_ms = pros::millis();

    while (true) {

        uint32_t curr_ms = pros::millis();
        curr_dt = curr_ms - prev_ms;
        prev_ms = curr_ms;

        master.update_display();
        update_position();
        update_intake();

        if (current_motion == nullptr) {
            queue_mutex.take();
            if (!motion_queue.empty()) {
                current_motion = motion_queue.front();
                motion_queue.pop_front();
                queue_mutex.give();
                master.rumble(".");
                current_motion->start();
            }
            else {
                queue_mutex.give();
            }
        }

        // Update running motion
        if (current_motion) {
            // Trigger motion events first (so events like stop_when() take effect immediately)
            for (auto& e : current_motion->conditional_events) {
                if (!e.triggered && e.condition()) {
                    e.action();
                    e.triggered = true;
                }
            }

            if(!current_motion->sequential_events.empty()) {
                ConditionalEvent& e = current_motion->sequential_events.front();
                if (e.condition()) {
                    e.action();
                    current_motion->sequential_events.pop();
                }
            }

            if (current_motion->is_done()) {
                current_motion = nullptr;
                master.rumble(".");
            }
            else {
                current_motion->update();
            }
        }

        pros::Task::delay_until(&curr_ms, DELTA_TIME);
    }

}

void system_loop_intake_teleop() {
    
    while(true) {

        user_update_intake_teleop();
		update_intake();
        pros::delay(10);

    }

}

void system_loop_drive_teleop() {

    system_start_time = pros::millis();

    teleop_intake_task = new pros::Task([]() {
        set_anti_jam(false);
        system_loop_intake_teleop();
    });

    while (true) {

        uint32_t prev_time = pros::millis();

        master.update_display();
        // update_position();

        bool joysticks_active = 
            (abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) > 10) ||
            (abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) > 10) ||
            (abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) > 10) ||
            (abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) > 10);

        if(!motion_queue.empty() || current_motion != nullptr) {
            if(joysticks_active) {
                // Clear motion queue and stop current motion
                queue_mutex.take();
                motion_queue.clear();
                queue_mutex.give();
                current_motion = nullptr;
                master.rumble("-");
            }
        }

        if(joysticks_active && teleop_intake_task == nullptr) {
            teleop_intake_task = new pros::Task([]() {
                system_loop_intake_teleop();
            });
        }

        if(master.get_digital_new_press(DIGITAL_B) && pros::competition::is_competition_switch()) {
			drive_modes.cycle_forward();
			master.rumble("-");
		}

        if(master.get_digital_new_press(DIGITAL_X)) {
            display_modes.cycle_forward()();
            master.rumble("-");
        }

        if(motion_queue.empty() && current_motion == nullptr) {
            handle_joystick_inputs(drive_modes.get_value());
            user_update_drive_teleop();
        }

        else {
        
        if (current_motion == nullptr) {
            queue_mutex.take();
            if (!motion_queue.empty()) {
                current_motion = motion_queue.front();
                motion_queue.pop_front();
                queue_mutex.give();

                current_motion->start();
            }
            else {
                queue_mutex.give();
            }
        }

        // Update running motion
        if (current_motion) {
            // Trigger motion events first (so events like stop_when() take effect immediately)
            for (auto& e : current_motion->conditional_events) {
                if (!e.triggered && e.condition()) {
                    e.action();
                    e.triggered = true;
                }
            }

            if(!current_motion->sequential_events.empty()) {
                ConditionalEvent& e = current_motion->sequential_events.front();
                if (e.condition()) {
                    e.action();
                    current_motion->sequential_events.pop();
                }
            }

            if (current_motion->is_done()) {
                current_motion = nullptr;
                master.rumble(".");
            }
            else {
                current_motion->update();
            }
        }

        }

        pros::Task::delay_until(&prev_time, DELTA_TIME);
    }

}

void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        selected_route = (selected_route + 1) % routes.size();
    }
}

void initialize() {

	open_log("/usd/log.bin");

    master.clear();
    set_brake_mode(DEFAULT_AUTONOMOUS_BRAKE_MODE);

    reset(Pose(routes[selected_route].start_position, standard_radians(routes[selected_route].start_heading)));
    calibrate_drive();
    
    routes[selected_route].queue();
    pros::lcd::initialize();
    pros::lcd::register_btn1_cb(on_center_button);

    display_setup();
    setup_task = new pros::Task([]() {
        while(true) {
            master.update_display();
            pros::delay(50);
        }
    });

    if(routes[selected_route].setup != nullptr) {
        routes[selected_route].setup();
    }

}

void autonomous() {

    autonomous_run = true;
    if(isnanf(get_x()) || isnanf(get_y()) || isnanf(get_heading())) {
        reset(Pose(routes[selected_route].start_position, standard_radians(routes[selected_route].start_heading)));
        pros::delay(50);
    }

    if(setup_task != nullptr) {
        setup_task->remove();
    }

    display_pose();

    autonomous_system_task = new pros::Task([]() {
        pros::delay(10); // give the task time to start (idk if this does anything)
        while(motion_queue.size() <= 0) pros::delay(50);
        system_loop_autonomous();
    });

}

void disabled() {
    if(autonomous_run) {
        user_pre_teleop();
        autonomous_run = false;
    }
    else user_pre_auton();
}

void opcontrol() {

    display_battery_time();

    if(autonomous_system_task != nullptr) {
        autonomous_system_task->remove();
    }

    motion_queue.clear();
    current_motion = nullptr;

    set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    teleop_drive_task = new pros::Task([]() {
        pros::delay(10); // give the task time to start (idk if this does anything)
        system_loop_drive_teleop();
    });

}