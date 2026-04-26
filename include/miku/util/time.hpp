#pragma once

#include "api.h"

class Timer {
    public:
        Timer(int time);
        Timer() = default;
        int get_time_set() const;
        int get_time_left();
        int get_time_passed();
        bool is_done();
        bool is_paused() const;
        void set(int time);
        void reset();
        void pause();
        void resume();
        void wait_until_done();
    private:
        void update();

        int m_period;
        int m_lastTime = 0;
        int m_timeWaited = 0;
        bool m_paused = false;
};

class Stopwatch {
    public:
        Stopwatch();
        int get_time_passed();
        void reset();
    private:
        int m_startTime = pros::millis();
};