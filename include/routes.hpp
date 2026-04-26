#pragma once
#include <string>
#include <functional>
#include "miku/util/geometry.hpp"
#include "miku/system/motions.hpp"
#include "miku/devices/intake.hpp"
#include "config.h"

inline void sawp() {

    // wait(100)
    //     .start([]() { load_intake(); })
    //     .queue();
    // move_time(4000, 12000, 200).queue();
    // turn_point({-48, -48}, 300, {.reverse = true}).queue();

    add_field_object(new Polygon({{12, -54}, {12, -72}, {30, -72}, {30, -54}}));
    
    move_pose({-48, -48}, 90, 1200, {.reverse = true})
        .start([]() { load_intake(); })
        .queue();
    turn_point({-48, -71}, 750)
        .start([]() { loader_piston.set_value(true); })
        .queue();
    move_distance(15, 300, {.min_volt_pct = 40}).queue();
    move_time(6000, 6000, 500).queue();
    move_pose({-48, -26}, 180, 1000, {.reverse = true, .straight = true, .max_vel_pct = 50})
        .within(6.0, []() { score_intake(); })
        .queue();
    move_time(-3000, -3000, 300)
        .start([]() { score_intake(); })
        .queue();
    turn_point({-24, -24}, 750, {.cutoff = 5.0, .min_volt_pct = 20})
        .start([]() { loader_piston.set_value(false); load_intake(); })
        .queue();

    move_pose({-24, -24}, 1000, {.cutoff = 8.0, .min_vel_pct = 20}).queue();

    move_pose({24, -24}, 1600, {.quick_exit = false, .end_cutoff = 12.0})
        .within(24.0, []() { set_intake_middle(0); })
        .within(8.0, []() { loader_piston.set_value(true); })
        .queue();

    turn_heading(-45, 800)
        .start([]() { loader_piston.set_value(false); })
        .queue();
    move_pose({10, -10}, -45, 1000, {.straight = true, .max_vel_pct = 40})
        .start([]() { stop_intake(); middle_piston.set_value(true); })
        .within(8.0, []() { set_intake_top(0); set_intake_middle(-8000); set_intake_bottom_velocity(-500); })
        .queue();

    turn_point({4, -4}, 300)
        .start([]() { set_intake_top(0); set_intake_middle(-8000); set_intake_bottom_velocity(-500);})
        .queue();
    wait(300).queue();

    move_pose({24, -24}, 500, {.reverse = true, .cutoff = 6.0}).queue();
    move_pose({48, -48}, 1200, {.reverse = true, .quick_exit = false, .end_cutoff = 12.0})
        .start([]() { load_intake(); middle_piston.set_value(false); })
        .queue();
    
    turn_point({48, -71}, 750)
        .start([]() { loader_piston.set_value(true); })
        .queue();
    move_distance(15, 300, {.min_volt_pct = 40}).queue();
    move_time(6000, 6000, 500).queue();

    move_pose({48, -26}, 180, 1000, {.reverse = true, .straight = true, .max_vel_pct = 50})
        .within(6.0, []() { score_intake(); })
        .queue();
    move_time(-3000, -3000, 500)
        .start([]() { score_intake(); loader_piston.set_value(false); })
        .queue();

    turn_heading(120, 500, {.cutoff = 5.0, .min_volt_pct = 20})
        .start([]() { load_intake(); })
        .queue();
    move_pose({61, -36}, 400).queue();
    move_pose({58, -20}, 180, 300, {.reverse = true, .straight = true, .cutoff = 3.0, .k3 = 0.5})
        .start([]() { descore_piston.set_value(false); })
        .queue();
    move_time(-8000, -8000, 500)
        .end([]() { return get_y() > -16.0; })
        .queue();
    move_time(6000, 6000, 50).queue();
    
}

inline void push_sawp() {
    wait(100)
        .start([]() { load_intake(); })
        .queue();
    move_time(12000, 4000, 200).queue();
    turn_point({48, -48}, 300, {.reverse = true, .quick_exit = true}).queue();

    // add_field_object(new Polygon({{12, -54}, {12, -72}, {30, -72}, {30, -54}}));
    
    move_pose({48, -48}, -90, 1200, {.reverse = true, .quick_exit = true})
        .start([]() { load_intake(); })
        .within(8.0, []() { loader_piston.set_value(true); })
        .queue();
    // turn_heading(180, 500);
    move_point({48, -64}, 500, {.quick_exit = true, .straight = true, .cutoff = 6.0})
        .start([]() { stop_intake(); })
        .elapsed(100, []() { load_intake(); })
        .queue();
    // move_distance(15, 300, {.min_volt_pct = 40}).queue();
    move_time(5000, 5000, 400).queue();
    move_pose({48, -26}, 180, 1000, {.reverse = true, .quick_exit = true, .straight = true, .max_vel_pct = 50})
        .within(2.0, []() { score_intake(); })
        .queue();
    move_time(-3000, -3000, 500)
        .start([]() { score_intake(); })
        .queue();
    turn_heading(-90, 750, {.cutoff = 5.0, .min_volt_pct = 20})
        .start([]() { loader_piston.set_value(false); stop_intake(); })
        .elapsed(100, []() { load_intake(); })
        .queue();

    move_point({24, -24}, 1000, {.cutoff = 5.0}).queue();

    move_point({-12, -24}, 1000, {.cutoff = 6.0, .min_volt_pct = 30})
        .queue();

    move_point({-24, -24}, 800, {.quick_exit = true, .drive_max_volt_pct = 30})
        // .within(8.0, []() { loader_piston.set_value(true); })
        .queue();

    move_pose({-48, -36}, -100, 1000, {.straight = true}).queue();
    // turn_point({-48, -26}, 500, {.reverse = true, .quick_exit = true}).queue();
    // turn_heading(180, 750, { .cutoff = 5.0 });
    move_pose({-48, -26}, 180, 750, {.reverse = true, .straight = true, .max_vel_pct = 50})
        .within(2.0, []() { score_intake(); })
        .queue();
    move_time(-3000, -3000, 500)
        .start([]() { score_intake(); loader_piston.set_value(true); })
        .queue();
    move_point({-48, -64}, 800, {.quick_exit = true, .straight = true, .drive_max_volt_pct = 65})
        .start([]() { set_intake(-8000mV, -8000mV, -12000mV); })
        .elapsed(200, []() { load_intake(); })
        .queue();
    // move_distance(15, 300, {.min_volt_pct = 40}).queue();
    move_time(5000, 5000, 400).queue();
    move_distance(-8, 500).queue();
    move_pose({-9, -9}, -135, 2000, {.reverse = true, .straight = true})
        .within(15.0, []() { 
            queue_command({-8000, mV}, {-8000, mV}, {-12000, mV}, 300);
			queue_command([]() { lock_piston.set_value(true); });
			set_intake_top(-8000);
			set_intake_middle(6000);
			set_intake_bottom(12000); })
        .queue();
    wait(500).start([]() { loader_piston.set_value(false); }).queue();
    
}

inline void left_split() {
    move_pose({-48, -48}, 90, 1200, {.reverse = true, .quick_exit = true})
        .start([]() { load_intake(); })
        .within(8.0, []() { loader_piston.set_value(true); })
        .queue();
    move_point({-48, -64}, 500, {.quick_exit = true, .straight = true, .cutoff = 6.0})
        .start([]() { stop_intake(); })
        .elapsed(100, []() { load_intake(); })
        .queue();
    move_time(5000, 5000, 500).queue();
    move_pose({-48, -26}, 180, 1000, {.reverse = true, .straight = true, .max_vel_pct = 50})
        .within(2.0, []() { score_intake(); })
        .queue();
    move_time(-3000, -3000, 400)
        .start([]() { score_intake(); })
        .queue();
    turn_point({-24, -24}, 750, {.cutoff = 5.0, .min_volt_pct = 20})
        .start([]() { loader_piston.set_value(false); stop_intake(); })
        .elapsed(100, []() { load_intake(); })
        .queue();
    move_pose({-24, -24}, 1000)
        .within(8.0, []() { loader_piston.set_value(true); })
        .queue();
    move_point({-36, -8}, 1200, {.straight = true})
        .start([]() { loader_piston.set_value(false); })
        .queue();
    turn_heading(-90, 500).queue();
    move_distance(10, 1000)
        .elapsed(400, []() { loader_piston.set_value(true); })
        .queue();
    wait(300)
        .start([]() { loader_piston.set_value(true); })
        .queue();
    move_point({-18, -18}, 750, {.reverse = true, .drive_max_volt_pct = 50}).queue();
    turn_heading(-135, 500, {.reverse = true}).queue();
    move_pose({-8, -8}, -135, 2000, {.reverse = true, .straight = true})
        .within(15.0, []() {
            queue_command({-12000, mV}, {-12000, mV}, {-12000, mV}, 300);
			queue_command([]() { lock_piston.set_value(true); });
			set_intake_top(-8000);
			set_intake_middle(6000);
			set_intake_bottom(12000); })
        .queue();
    wait(500).start([]() { loader_piston.set_value(false); }).queue();

}

inline void right_rush() {
    move_pose({24, -24}, 1000)
        .within(10.0, []() { loader_piston.set_value(true); load_intake(); })
        .queue();
    // move_distance(22, 800, {.min_volt_pct = 30})
    //     .within(10.0, []() { loader_piston.set_value(true); load_intake(); })
    //     .queue();
    turn_point({42, -29}, 500, {.cutoff = 10.0}).queue();
    move_pose({42, -29}, 500, {.reverse = true, .cutoff = 3.0, .min_vel_pct = 30}).queue();
    move_time(-12000, -4000, 800)
        .start([]() { loader_piston.set_value(false);})
        .queue();
    move_time(-6000, -6000, 300)
        .start([]() { score_intake(); })
        .queue();
    turn_heading(120, 500, {.cutoff = 5.0, .min_volt_pct = 20})
        .start([]() { load_intake(); })
        .queue();
    move_pose({61, -36}, 300).queue();
    move_pose({58, -20}, 180, 300, {.reverse = true, .straight = true, .cutoff = 3.0, .k3 = 0.5})
        .start([]() { descore_piston.set_value(false); })
        .queue();
    move_time(-8000, -8000, 500)
        .end([]() { return get_y() > -16.0; })
        .queue();
    move_time(6000, 6000, 50).queue();
    
}

inline void pre_skills() { descore_piston.set_value(true); }

inline void skills() {

    auto shimmy = []() {
        move_time(5000, 5000, 400).queue();
        wait(400).queue();
        move_time(-4000, -4000, 50).queue();
        move_time(4000, 4000, 200).queue();
    };

    auto park_clear = []() {
        move_time(6000, 6000, 500)
            .start([]() { load_intake(); })
            .elapsed(200, []() { loader_piston.set_value(true); })
            .queue();
        move_time(4000, 4000, 1000).queue();   
        move_time(6000, 6000, 750)
            .start([]() { loader_piston.set_value(false); })
            .queue();
        move_time(4000, 4000, 750).queue();
        move_time(-4000, -4000, 200).queue();
        move_time(4000, 4000, 750).queue();
        move_time(-6000, -6000, 800).queue();
    };

    park_clear();

    turn_heading(180, 500)
        .start([]() { distance_reset({0, -40}); set_anti_jam(true); })
        .queue();
    turn_point({18, -30}, 1000).queue();
    move_pose({18, -30}, 1500, {.max_vel_pct = 50}).queue();
    turn_point({18, -18}, 1000)
        .start([]() { queue_command(0, 0, -6000, 100); })
        .queue();
    move_pose({18, -18}, 1000).queue();
    turn_point({10, -10}, 1000).queue();
    move_pose({10, -10}, -45, 1000)
        .start([]() { stop_intake(); middle_piston.set_value(true); })
        .queue();

    turn_point({4, -4}, 500).queue();
    wait(500)
        .start([]() { set_intake_bottom_velocity(-400); set_intake_middle(0); set_intake_top(0); })
        .queue();
    wait(2500)
        .start([]() { set_intake_bottom_velocity(-180); set_intake_middle_velocity(-300); set_intake_top(0); })
        .queue();
    
    move_time(-2000, -2000, 200).queue();
    
    move_point({24, -27}, 1000, {.reverse = true}).queue();
    turn_point({-24, -26}, 500)
        .start([]() { load_intake(); middle_piston.set_value(false); })
        .queue();

    move_pose({-24, -26}, 1000, {.quick_exit = true}).queue();
    move_point({-48, -48}, 1200, {.straight = true, .drive_max_volt_pct = 70})
        .within(12.0, []() { loader_piston.set_value(true); })
        .queue();

    move_point({-48, -60}, 1000, {.quick_exit = true, .straight = true})
        .start([]() { load_intake(); loader_piston.set_value(true); })
        .queue();
    shimmy();

    move_pose({-60, -24}, 180, 1500, {.reverse = true, .min_settle_volt_pct = 30}).queue();
    move_point({-60, 30}, 2000, {.reverse = true, .drive_max_volt_pct = 75, .min_volt_pct = 30})
        .start([]() { loader_piston.set_value(false); })
        .queue();
    move_point({-48, 36}, 1000, {.reverse = true}).queue();
    turn_heading(0, 1000).queue();
    move_pose({-48, 26}, 0, 600, {.reverse = true, .straight = true, .max_vel_pct = 50})
        .within(3.0, []() { score_intake(); })
        .queue();
    move_time(-3000, -3000, 300)
        .start([]() { score_intake(); loader_piston.set_value(true); })
        .queue();
    wait(1000).queue();

    move_point({-48, 60}, 1000, {.quick_exit = true, .straight = true, .drive_max_volt_pct = 65})
        .start([]() { load_intake(); })
        .queue();
    shimmy();
    move_point({-48, 26}, 1500, {.reverse = true})
        .within(3.0, []() { score_intake(); })
        .queue();
    move_time(-3000, -3000, 500)
        .start([]() { score_intake(); })
        .queue();
    wait(500)
        .start([]() { score_intake_slow(); })
        .queue();
    move_distance(8, 500)
        .start([]() { loader_piston.set_value(false); load_intake(); })
        .queue();
    turn_point({-30, 30}, 1000).queue();
    move_point({-30, 30}, 1000).queue();
    wait(300).queue();
    
    move_pose({0, 36}, 2000, {.straight = true, .max_vel_pct = 50})
        .elapsed(200, []() { set_intake_bottom(-3000); })
        .queue();
    // turn_point({0, 71}, 1000).queue();
    // move_distance(8, 1000).queue();
    move_pose({0, 48}, 0, 1000, {.straight = true}).queue();
    
    park_clear();
    
    turn_heading(0, 500)
        .start([]() { distance_reset({0, 40}); set_anti_jam(true); })
        .queue();
    turn_point({14, 14}, 1000, {.reverse = true}).queue();
    move_point({14, 14}, 2000, {.reverse = true, .drive_max_volt_pct = 50})
        .within(6.0, []() { 
            queue_command(0mV, 0mV, -12000mV, 200);
            queue_command(-8000mV, -8000mV, 0, 200);
            queue_command([]() { lock_piston.set_value(true); });
        })
        .queue();
    move_pose({9, 9}, 45, 1000, {.reverse = true, .straight = true, .max_vel_pct = 50})
        .start([]() {
            queue_command({-180, rpm}, {150, rpm}, {12000, mV}, 500);
            queue_command({-150, rpm}, {120, rpm}, {12000, mV}, 800);
        })
        .queue();
    wait(2500)
        .start([]() {
            set_intake_top_velocity(-120);
            set_intake_middle_velocity(100);
            set_intake_bottom(12000);
        })
        .queue();
    
    move_point({48, 48}, 2000, {.drive_max_volt_pct = 80})
        .start([]() { load_intake(); })
        .within(12.0, []() { loader_piston.set_value(true); })
        .queue();
    move_point({48, 60}, 1000, {.quick_exit = true, .straight = true}).queue();
    shimmy();

    move_pose({60, 24}, 0, 1500, {.reverse = true, .min_settle_volt_pct = 30}).queue();
    move_point({60, -30}, 2000, {.reverse = true, .drive_max_volt_pct = 75, .min_volt_pct = 30})
        .start([]() { loader_piston.set_value(false); })
        .queue();
    move_point({48, -36}, 1000, {.reverse = true}).queue();
    turn_heading(180, 1000).queue();
    move_point({48, -26}, 1500, {.reverse = true})
        .within(3.0, []() { score_intake(); })
        .queue();
    move_time(-3000, -3000, 300)
        .start([]() { score_intake(); loader_piston.set_value(true); })
        .queue();
    wait(1000).queue();

    move_point({48, -60}, 1000, {.quick_exit = true, .straight = true, .drive_max_volt_pct = 75})
        .start([]() { load_intake(); })
        .queue();
    shimmy();
        
    move_point({48, -26}, 1800, {.reverse = true})
        .within(3.0, []() { score_intake(); })
        .queue();
    move_time(-3000, -3000, 500)
        .start([]() { score_intake(); })
        .queue();
    wait(500)
        .start([]() { score_intake_slow(); })
        .queue();

    move_pose({0, -40}, 2000) 
        .start([]() { loader_piston.set_value(false); })
        .elapsed(200, []() { load_intake(); })
        .queue();
    turn_point({0, -71}, 1000).queue();
    
    move_time(7000, 7000, 2000).queue();

    // */

}