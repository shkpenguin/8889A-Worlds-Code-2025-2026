#include "miku/devices/intake.hpp"
#include "miku/devices/controller.hpp"
#include "config.h"

LookupTable intake_top_lut({
    {-281, -8000}, {-278, -7500}, {-262, -7000}, {-244, -6500}, {-226, -6000}, 
    {-202, -5500}, {-183, -5000}, {-167, -4500}, {-149, -4000}, {-129, -3500}, 
    {-110, -3000}, {-91, -2500}, {-71, -2000}, {-52, -1500}, {-32, -1000},
    {-16, -500}, {0, 0}, {14, 500}, 
    {32, 1000}, {50, 1500}, {67, 2000}, {81, 2500}, {103, 3000},
    {121, 3500}, {141, 4000}, {158, 4500}, {174, 5000}, {194, 5500},
    {216, 6000}, {233, 6500}, {250, 7000}, {268, 7500}, {272, 8000}
});
LookupTable intake_middle_lut({
    {-283, -8000}, {-279, -7500}, {-262, -7000}, {-245, -6500}, {-226, -6000},
    {-202, -5500}, {-182, -5000}, {-167, -4500}, {-149, -4000}, {-129, -3500},
    {-110, -3000}, {-90, -2500}, {-71, -2000}, {-51, -1500}, {-31, -1000},
    {-16, -500}, {0, 0}, {15, 500},
    {33, 1000}, {51, 1500}, {69, 2000}, {83, 2500}, {105, 3000},
    {123, 3500}, {143, 4000}, {159, 4500}, {176, 5000},{195, 5500},
    {218, 6000}, {236, 6500}, {252, 7000}, {272, 7500}, {277, 8000}
});
LookupTable intake_bottom_lut({
    {-669, -12000}, {-612, -11000}, {-559, -10000}, {-503, -9000}, {-465, -8500},
    {-438, -8000}, {-413, -7500}, {-381, -7000}, {-354, -6500}, {-327, -6000}, 
    {-289, -5500}, {-259, -5000}, {-230, -4500}, {-199, -4000}, {-170, -3500}, 
    {-143, -3000}, {-106, -2500}, {-78, -2000}, {-59, -1500}, {-29, -1000},
    {0, -500}, {0, 0}, {0, 500}, 
    {23, 1000}, {55, 1500}, {76, 2000}, {103, 2500}, {135, 3000},
    {167, 3500}, {195, 4000}, {226, 4500}, {251, 5000}, {283, 5500},
    {320, 6000}, {348, 6500}, {339, 7000}, {368, 7500}, {397, 8000},
    {429, 8500}, {463, 9000}, {517, 10000}, {578, 11000}, {652, 12000}
});

miku::Motor intake_top(INTAKE_TOP_PORT, INTAKE_TOP_CARTRIDGE, pros::v5::MotorUnits::degrees, intake_top_lut, INTAKE_TOP_PID_GAINS);
miku::Motor intake_middle(INTAKE_MIDDLE_PORT, INTAKE_MIDDLE_CARTRIDGE, pros::v5::MotorUnits::degrees, intake_middle_lut, INTAKE_MIDDLE_PID_GAINS);
miku::Motor intake_bottom(INTAKE_BOTTOM_PORT, INTAKE_BOTTOM_CARTRIDGE, pros::v5::MotorUnits::degrees, intake_bottom_lut, INTAKE_BOTTOM_PID_GAINS);

bool anti_jam_enabled = true;

uint32_t jam_detect_time = 0;
uint32_t last_unjam_end_time = 0;

bool torque_stop = false;
bool loading_mode = false;

uint32_t load_start_time = 0;
uint32_t torque_high_start = 0;

struct QueueEntry {
    MotorCommand top;
    MotorCommand middle;
    MotorCommand bottom;
    std::function<void()> action;
    bool has_action = false;
    bool has_command = false;
    bool auto_clear = false;
    uint32_t duration_ms = 0;     // 0 => run until removed
    uint32_t start_time = 0;      // will be set when entry starts executing
    bool is_unjam = false;       // convenience flag for anti-jam entries
};

std::deque<QueueEntry> command_queue;
pros::Mutex intake_queue_mutex; 

static constexpr float LOW_VELOCITY_THRESHOLD = 10.0f; // rpm
static constexpr int JAM_THRESHOLD_MS = 100;         // ms
static constexpr int UNJAM_DURATION_MS = 50;        // ms
static constexpr int UNJAM_VOLTAGE = -12000;         // mV
static constexpr uint32_t UNJAM_COOLDOWN_MS = 2000;

MotorCommand top_command;
MotorCommand middle_command;
MotorCommand bottom_command;

void execute_intake_command(miku::Motor& motor, const MotorCommand& cmd) {
    if (cmd.type == MotorCommandType::VOLTAGE) {
        motor.move_voltage(cmd.value);
    } else {
        motor.move_velocity(cmd.value);
    }
}

void move_intake() {
    if (!torque_stop) {
        execute_intake_command(intake_top, top_command);
    } else {
        intake_top.move_voltage(0);
    }
    execute_intake_command(intake_middle, middle_command);
    execute_intake_command(intake_bottom, bottom_command);
}

bool intake_jammed() {
    bool low_velocity = intake_bottom.get_filtered_velocity() < LOW_VELOCITY_THRESHOLD;
    bool commanded_forward = bottom_command.value > 0;
    return low_velocity && commanded_forward;
}

void queue_command(const MotorCommand& top_cmd,
                            const MotorCommand& middle_cmd,
                            const MotorCommand& bottom_cmd,
                            uint32_t duration_ms,
                            bool front) {
    QueueEntry e;
    e.top = top_cmd;
    e.middle = middle_cmd;
    e.bottom = bottom_cmd;
    e.has_command = true;
    e.duration_ms = duration_ms;
    e.start_time = 0;
    // heuristically mark standard unjam entries (internal use)
    e.is_unjam = (duration_ms == UNJAM_DURATION_MS &&
                  bottom_cmd.type == MotorCommandType::VOLTAGE &&
                  bottom_cmd.value == UNJAM_VOLTAGE);

    intake_queue_mutex.take();
    if (front) command_queue.push_front(e);
    else command_queue.push_back(e);
    intake_queue_mutex.give();
}

void queue_command(std::function<void()> action,
                            uint32_t duration_ms,
                            bool front) {
    QueueEntry e;
    e.action = std::move(action);
    e.has_action = true;
    e.duration_ms = duration_ms;
    e.auto_clear = (duration_ms == 0);
    e.start_time = 0;

    intake_queue_mutex.take();
    if (front) command_queue.push_front(e);
    else command_queue.push_back(e);
    intake_queue_mutex.give();
}

void handle_jam_detection(uint32_t now) {
    if (intake_jammed()) {
        if (jam_detect_time == 0) {
            jam_detect_time = now;
        }

        if (now - jam_detect_time > JAM_THRESHOLD_MS) {
            if (now - last_unjam_end_time >= UNJAM_COOLDOWN_MS) {
                MotorCommand unjam_cmd(UNJAM_VOLTAGE, MotorCommandType::VOLTAGE);
                queue_command(0, 0, unjam_cmd, UNJAM_DURATION_MS);
                jam_detect_time = 0;
            } else {
                move_intake();
            }
        } else {
            move_intake();
        }
    } else {
        jam_detect_time = 0;
        move_intake();
    }
}

void set_intake(const MotorCommand& top_cmd, const MotorCommand& middle_cmd, const MotorCommand& bottom_cmd) {
    torque_stop = false;
    loading_mode = false;
    load_start_time = 0;
    top_command = top_cmd;
    middle_command = middle_cmd;
    bottom_command = bottom_cmd;
}

// Backwards compatible two-arg overload
void set_intake(const MotorCommand& top_cmd, const MotorCommand& bottom_cmd) {
    set_intake(top_cmd, MotorCommand(0.0f, MotorCommandType::VOLTAGE), bottom_cmd);
}

void set_intake(float top_voltage, float middle_voltage, float bottom_voltage) {
    // call three-arg setter so flags are cleared
    set_intake(MotorCommand(top_voltage, MotorCommandType::VOLTAGE),
        MotorCommand(middle_voltage, MotorCommandType::VOLTAGE),
        MotorCommand(bottom_voltage, MotorCommandType::VOLTAGE));
}

// Backwards compatible two-arg overload
void set_intake(float top_voltage, float bottom_voltage) {
    set_intake(top_voltage, top_voltage, bottom_voltage);
}

void set_intake_velocity(float top_vel, float middle_vel, float bottom_vel) {
    top_command = MotorCommand(top_vel, MotorCommandType::VELOCITY);
    middle_command = MotorCommand(middle_vel, MotorCommandType::VELOCITY);
    bottom_command = MotorCommand(bottom_vel, MotorCommandType::VELOCITY);
}

// Backwards compatible two-arg overload
void set_intake_velocity(float top_vel, float bottom_vel) {
    set_intake_velocity(top_vel, top_vel, bottom_vel);
}

void set_intake_top(const MotorCommand& cmd) {
    torque_stop = false;
    loading_mode = false;
    load_start_time = 0;
    top_command = cmd;
}

void set_intake_top(float voltage) {
    torque_stop = false;
    loading_mode = false;
    load_start_time = 0;
    top_command = MotorCommand(voltage, mV);
}

void set_intake_top_velocity(float velocity) {
    torque_stop = false;
    loading_mode = false;
    load_start_time = 0;
    top_command = MotorCommand(velocity, rpm);
}

void set_intake_middle(const MotorCommand& cmd) {
    torque_stop = false;
    loading_mode = false;
    load_start_time = 0;
    middle_command = cmd;
}

void set_intake_middle(float voltage) {
    torque_stop = false;
    loading_mode = false;
    load_start_time = 0;
    middle_command = MotorCommand(voltage, mV);
}

void set_intake_middle_velocity(float velocity) {
    torque_stop = false;
    loading_mode = false;
    load_start_time = 0;
    middle_command = MotorCommand(velocity, rpm);
}

void set_intake_bottom(const MotorCommand& cmd) {
    torque_stop = false;
    loading_mode = false;
    load_start_time = 0;
    bottom_command = cmd;
}

void set_intake_bottom(float voltage) {
    torque_stop = false;
    loading_mode = false;
    load_start_time = 0;
    bottom_command = MotorCommand(voltage, mV);
}

void set_intake_bottom_velocity(float velocity) {
    torque_stop = false;
    loading_mode = false;
    load_start_time = 0;
    bottom_command = MotorCommand(velocity, rpm);
}

void load_intake() {
    lock_piston.set_value(false);
    if (!loading_mode) {
        load_start_time = pros::millis();
    }
    loading_mode = true;
    // assign commands directly (set() would clear torque_stop)
    top_command = MotorCommand(3000.0f, MotorCommandType::VOLTAGE);
    middle_command = MotorCommand(3000.0f, MotorCommandType::VOLTAGE);
    bottom_command = MotorCommand(12000.0f, MotorCommandType::VOLTAGE);
}

void score_intake() {
    // exiting load state
    loading_mode = false;
    load_start_time = 0;
    lock_piston.set_value(true);
    set_intake(12000.0f, 12000.0f, 12000.0f);
}

void score_intake_slow() {
    // exiting load state
    loading_mode = false;
    load_start_time = 0;
    lock_piston.set_value(true);
    set_intake_bottom(12000.0f);
    set_intake_middle_velocity(200);
    set_intake_top_velocity(200);
}

void stop_intake() {
    loading_mode = false;
    load_start_time = 0;
    top_command = MotorCommand(0.0f, MotorCommandType::VOLTAGE);
    middle_command = MotorCommand(0.0f, MotorCommandType::VOLTAGE);
    bottom_command = MotorCommand(0.0f, MotorCommandType::VOLTAGE);
}

void set_anti_jam(bool enabled) {
    anti_jam_enabled = enabled;
    if (!enabled) {
        // clear any transient jam state
        jam_detect_time = 0;
    }
}

float get_intake_top_velocity() {
    return intake_top.get_filtered_velocity();
}

float get_intake_middle_velocity() {
    return intake_middle.get_filtered_velocity();
}

float get_intake_bottom_velocity() {
    return intake_bottom.get_filtered_velocity();
}

int get_intake_top_temperature() {
    return intake_top.get_temperature();
}

int get_intake_middle_temperature() {
    return intake_middle.get_temperature();
}

int get_intake_bottom_temperature() {
    return intake_bottom.get_temperature();
}

void update_intake() {
    uint32_t now = pros::millis();

    if (loading_mode && load_start_time != 0 && now - load_start_time >= 1000) {
        // only monitor torque after intake has been running for a second
        double tq = intake_top.get_torque();
        // if we're above the threshold start/continue the high-torque timer
        if (tq > 0.25) {
            if (torque_high_start == 0) {
                torque_high_start = now;
            } else if (now - torque_high_start >= 5) {
                torque_stop = true;
                intake_top.move_voltage(0);
                master.rumble("-");
            }
        } else {
            // torque dropped below threshold, reset timer
            torque_high_start = 0;
        }
        // do NOT return early – we still want the queue/anti-jam logic to
        // update middle & bottom motors and possibly push new commands.
    }

    // 1) If there are queued commands, execute the active entry (highest
    // priority = front of deque). Queued commands override default commands
    // until they complete or are cancelled.
    intake_queue_mutex.take();
    if (!command_queue.empty()) {
        QueueEntry &active = command_queue.front();
        if (active.start_time == 0) active.start_time = now;

        if (active.has_action) {
            active.action();
        }

        if (active.has_command) {
            if(torque_stop) intake_top.move_voltage(0);
            else execute_intake_command(intake_top, active.top);
            execute_intake_command(intake_middle, active.middle);
            execute_intake_command(intake_bottom, active.bottom);
        }

        bool should_pop = false;
        bool was_unjam = active.is_unjam;
        if (active.has_action && !active.has_command && active.auto_clear) {
            should_pop = true;
        } else if (active.duration_ms > 0 && (now - active.start_time >= active.duration_ms)) {
            should_pop = true;
        }

        if (should_pop) {
            command_queue.pop_front();
            intake_queue_mutex.give();

            if (was_unjam) {
                // start unjam cooldown so we don't immediately re-enter
                last_unjam_end_time = now;
            }
            return; // we've already executed the queued command this tick
        }

        intake_queue_mutex.give();
        return;
    }
    intake_queue_mutex.give();

    // 2) No queued commands — fall back to anti-jam detection or normal
    // command behavior.
    if (!anti_jam_enabled) {
        move_intake();
        return;
    }

    // anti-jam flow may push an unjam entry into the queue; jam detection
    // will call move_normal() when no action is required
    handle_jam_detection(now);

}