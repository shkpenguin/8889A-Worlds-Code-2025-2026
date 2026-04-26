#pragma once

#include <vector>

class LookupTable {
    private:
        std::vector<std::pair<float, float>> table;
    public:
        LookupTable() {};
        LookupTable(std::vector<std::pair<float, float>> table) : table(table) {};
        LookupTable(std::initializer_list<std::pair<float, float>> table) : table(table) {};
        float get_value(float key);
};

void tune_lut_drive();
void tune_lut_intake();