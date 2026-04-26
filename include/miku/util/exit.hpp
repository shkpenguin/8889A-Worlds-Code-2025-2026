#pragma once
#include "pros/rtos.hpp"
#include "miku/util/geometry.hpp"

/**
 * @param range The acceptable range of error
 * @param time The time duration to check for the exit condition(ms)
 */
struct RangeExit {
    float range;
    int time;
    int start_time = -1;
    bool done = false;

    RangeExit(const float range, const int time) : range(range), time(time) {}

    bool get_exit() { return done; }
    bool update(const float input) {
        const int curr_time = pros::millis();
        if (std::fabs(input) > range) start_time = -1;
        else if (start_time == -1) start_time = curr_time;
        else if (curr_time >= start_time + time) done = true;
        return done;
    };
    void reset() { done = false; start_time = -1; }
};

struct PatienceExit {
    float stored_value = 0;          // last reference value
    float absolute_min_delta = 0;    // minimum improvement to reset patience
    int patience = 0;                 // current patience count
    int max_patience = 0;             // max patience before exit
    bool positive_improvement = true; // whether improvement means increasing or decreasing value
    int min_threshold = 0;            // minimum absolute value to consider for patience
    bool done = false;

    /* @param max_patience The number of consecutive non-improving measurements before exit
     * @param min_delta The minimum change in value considered an improvement (absolute value)
     * @param positive_improvement If true, improvement means value increases; if false, improvement means value decreases
     * @param min_threshold Minimum absolute value to consider for patience (values above this are ignored)
    */
    PatienceExit(int max_patience, float min_delta, bool positive_improvement = true, int min_threshold = 0)
        : max_patience(max_patience),
          absolute_min_delta(std::abs(min_delta)),
          positive_improvement(positive_improvement),
          min_threshold(min_threshold) {
        reset();
    }

    // reset state
    void reset() {
        patience = 0;
        done = false;
        if (positive_improvement) {
            stored_value = -1e9;
        } else {
            stored_value = 1e9;
        }
    }

    // call every loop with current measurement
    void update(float value) {
        if (done) return;
        if(fabs(value) > min_threshold) return;

        float delta = value - stored_value;
        bool improved = (positive_improvement && delta > absolute_min_delta)
                        || (!positive_improvement && delta < -absolute_min_delta);

        if (improved) {
            patience = 0;
            stored_value = value;
        } else {
            patience++;
        }

        if (patience >= max_patience) done = true;
    }

    bool get_exit() const { return done; }

};