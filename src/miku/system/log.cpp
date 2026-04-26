#include "miku/system/log.hpp"
#include "pros/rtos.hpp"
#include <fstream>
#include <vector>
#include <cstring>

std::ofstream log_file;
std::vector<uint8_t> log_buffer;

std::string filename = "/usd/log.bin";

uint8_t* log_ptr = nullptr;
uint8_t* log_end = nullptr;

pros::Mutex log_mutex;

static uint32_t last_flush_time = 0;
static uint32_t flush_interval_ms = 500;

inline void write_u8(uint8_t v) {
    *log_ptr++ = v;
}

inline void write_i8(int8_t v) {
    *log_ptr++ = (uint8_t)v;
}

inline void write_i16(int16_t v) {
    std::memcpy(log_ptr, &v, 2);
    log_ptr += 2;
}

inline void write_u16(uint16_t v) {
    std::memcpy(log_ptr, &v, 2);
    log_ptr += 2;
}

inline void write_i32(int32_t v) {
    std::memcpy(log_ptr, &v, 4);
    log_ptr += 4;
}

inline void write_u32(uint32_t v) {
    std::memcpy(log_ptr, &v, 4);
    log_ptr += 4;
}

inline void write_f(float v) {
    std::memcpy(log_ptr, &v, 4);
    log_ptr += 4;
}

inline void write_d(double v) {
    std::memcpy(log_ptr, &v, 8);
    log_ptr += 8;
}

void open_log(const std::string& filename) {

    log_mutex.take();

    log_file.open(filename, std::ios::binary);
    if(!log_file.is_open()) {
        std::cerr << "Failed to open log file: " << filename << std::endl;
        log_mutex.give();
        return;
    }
    std::cout << filename << " opened" << std::endl;

    log_buffer.resize(1024 * 512); 
    log_ptr = log_buffer.data();
    log_end = log_buffer.data() + log_buffer.size();

    log_mutex.give();
}

void flush_logs() {
    std::cout << "enter flush" << std::endl;
    if (!log_mutex.take(100)) {
        std::cout << "failed to get mutex" << std::endl;
        return;
    }
    size_t size = log_ptr - log_buffer.data();
    std::cout << "writing" << std::endl;
    if (size > 0) {
        log_file.write((char*)log_buffer.data(), size);
        std::cout << "flushing" << std::endl;
        log_file.flush();
    }
    std::cout << "logs flushed" << std::endl;

    log_ptr = log_buffer.data();
    last_flush_time = pros::millis();

    log_mutex.give();
}

void write_data(const Template& temp,
                        const std::vector<std::vector<double>>& values) {
    log_mutex.take();

    if (log_ptr == nullptr || log_end == nullptr) {
        log_mutex.give();
        return;
    }

    if (log_end - log_ptr < temp.size) {
        size_t size = log_ptr - log_buffer.data();

        if (size > 0) {
            log_file.write((char*)log_buffer.data(), size);
        }

        log_ptr = log_buffer.data();
    }

    // template id
    *log_ptr++ = temp.id;

    size_t value_index = 0;

    for (const auto& field : temp.format) {
        for (int r = 0; r < field.repeat; r++) {
            const auto& row = values[value_index++];

            size_t row_index = 0;

            for (auto type : field.types) {
                double v = row[row_index++];

                switch (type) {
                    case INT8:   write_i8((int8_t)v); break;
                    case UINT8:  write_u8((uint8_t)v); break;
                    case INT16:  write_i16((int16_t)v); break;
                    case UINT16: write_u16((uint16_t)v); break;
                    case INT32:
                    case INT:    write_i32((int32_t)v); break;
                    case UINT32: write_u32((uint32_t)v); break;
                    case FLOAT:
                    case FLOAT32: write_f((float)v); break;
                    case DOUBLE:
                    case FLOAT64: write_d((double)v); break;
                }
            }
        }
    }

    log_mutex.give();

}