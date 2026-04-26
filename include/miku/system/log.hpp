#pragma once

#include <mutex>
#include <iostream>
#include <iomanip>
#include <chrono>

#include <vector>
#include <cstdint>
#include <cstring>
#include <fstream>

enum DataType {
    INT = 1,
    FLOAT = 2,
    DOUBLE = 3,

    INT8 = 4,
    INT16 = 5,
    INT32 = 6,
    UINT8 = 7,
    UINT16 = 8,
    UINT32 = 9,
    FLOAT32 = 10,
    FLOAT64 = 11
};

struct Field {
    std::vector<DataType> types;
    int repeat;
};

class Template {
public:
    Template(int id, std::vector<Field> format) : id(id), format(std::move(format)) {
        size = 1; // start with 1 byte for template ID
        for(const auto& field : this->format) {
            int type_size = 0;
            for(auto type : field.types) {
                switch (type) {
                    case INT8:
                    case UINT8:  type_size += 1; break;
                    case INT16:
                    case UINT16: type_size += 2; break;
                    case INT32:
                    case UINT32:
                    case FLOAT32: type_size += 4; break;
                    case FLOAT64:
                    case DOUBLE: type_size += 8; break;
                    case INT:
                    case FLOAT: type_size += sizeof(double); break;
                }
            }
            size += type_size * field.repeat;
        }
    }
    std::vector<Field> format;
    int id;
    int size;


};

void open_log(const std::string& filename);
void flush_logs();
void set_flush_interval(uint32_t ms);
void write_data(const Template& temp, const std::vector<std::vector<double>>& values);