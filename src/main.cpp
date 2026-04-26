#include "config.h"
#include "main.h"
#include "miku/devices/controller.hpp"
#include "miku/system/system.hpp"
#include "miku/devices/intake.hpp"
#include "miku/system/drive.hpp"
#include "miku/system/log.hpp"
#include "miku/system/odom.hpp"
#include "routes.hpp"
#include "miku/libs/gif-pros.hpp"

miku::Pneumatic middle_piston('B');
miku::Pneumatic loader_piston('D');
miku::Pneumatic lock_piston('C');
miku::Pneumatic descore_piston('A');

miku::Controller master(pros::E_CONTROLLER_MASTER);

int selected_route = 1;
std::vector<Route> routes = {
	Route("sawp", {17, -48}, -90, push_sawp),
	Route("left", {-17, -48}, 90, left_split),
	Route("right", {16, -49}, 15, right_rush),
	Route("skills", {0, 0}, 180, skills),
};

List<DriveMode> drive_modes = {
    DriveMode::TANK,
    DriveMode::ARCADE,
};

List<std::function<void()>> display_modes = {
	// []() { // show lut test voltage and intake velocity
	// 	master.display(0, []() {
	// 		return fmt::format("lut voltage: {}mV", lut_test_voltage);
	// 	});
	// }
};

auto control_pneumatics = []() {
    bool shift = master.get_digital(DIGITAL_L1);

    if(master.get_digital_new_press(DIGITAL_A)) middle_piston.toggle();
    // if(master.get_digital_new_press(DIGITAL_RIGHT)) loader_piston.toggle();

    if(shift) {
        if(master.get_digital_new_press(DIGITAL_R1)) loader_piston.toggle();
    } else {
        if(master.get_digital_new_press(DIGITAL_R1)) {
            lock_piston.toggle();
            master.set_rumble(lock_piston.get_value());
        }
        
        if(master.get_digital_new_press(DIGITAL_L2)) descore_piston.toggle();
    }
};

void user_pre_auton() {

}

void user_pre_teleop() {
	// lexlib::helpers::gif::Gif gif("alysa2.gif", nullptr);
}

void user_update_intake_teleop() {

	control_pneumatics();

	if(master.get_digital_new_press(DIGITAL_LEFT)) {
		queue_command(0mV, 0mV, -12000mV, 200);
		queue_command(-8000mV, -8000mV, 0, 200);
		queue_command([]() { lock_piston.set_value(true); });
		queue_command({-180, rpm}, {150, rpm}, {12000, mV}, 500);
		queue_command({-150, rpm}, {120, rpm}, {12000, mV}, 800);
	}
	else if(master.get_digital(DIGITAL_LEFT)) {
		set_intake_top_velocity(-100);
		set_intake_middle_velocity(80);
		set_intake_bottom(12000);
	}

	// shift-R2 triggers manual reverse/unload action; keep existing logic
	if(master.get_digital_new_press(DIGITAL_RIGHT)) { 
		queue_command({0, rpm}, {0, rpm}, {-400, rpm}, 500);
	}
	else if(master.get_digital(DIGITAL_RIGHT)) {
		set_intake_bottom_velocity(-180); 
		set_intake_middle_velocity(-300); 
		set_intake_top(0);
	}

	// if(master.get_digital(DIGITAL_DOWN)) {
		// intake.set_intake_top(-12000);
		// intake.set_intake_middle_velocity(-150);
		// intake.set_intake_bottom_velocity(-150);
	// }

	bool shift = master.get_digital(DIGITAL_L1);

	if(shift) {

		// shift-R2 triggers manual reverse/unload action; keep existing logic
		if(master.get_digital(DIGITAL_R2) && !master.get_digital(DIGITAL_L2)) {
			set_intake(-12000, -12000);
		}

		// else if(master.get_digital_new_press(DIGITAL_L2)) intake.queue_command(-12000, -12000, -12000, 200);
		// else if(master.get_digital(DIGITAL_L2)) {
		//     intake.set_intake_top(-8000);
		//     intake.set_intake_middle(8000);
		//     intake.set_intake_bottom(12000);
		// }

		if(master.get_digital_new_press(DIGITAL_L2)) {
			queue_command({-7000, mV}, {-7000, mV}, {-12000, mV}, 200);
			queue_command([]() { lock_piston.set_value(true); });
		}
		else if(master.get_digital(DIGITAL_L2)) {
			set_intake_top(-8000);
			set_intake_middle(7000);
			set_intake_bottom(12000);
		}

	} else {

		if(master.get_digital(DIGITAL_R2)) {
			if(lock_piston.get_value() == false) {
				// enter loading mode; resets torque stop inside
				load_intake();
			} else {
				set_intake(12000, 12000);
			}
		}

	} // shift

	if(!master.get_digital(DIGITAL_R2) && !master.get_digital(DIGITAL_L2) && !master.get_digital(DIGITAL_LEFT) && !master.get_digital(DIGITAL_RIGHT)) {
		stop_intake();
	}

}


void user_update_drive_teleop() {

	if(master.get_digital_new_press(DIGITAL_Y)) flush_logs();

}