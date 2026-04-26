#include "miku/util/lut.hpp"
#include <algorithm>
#include <numeric>

float LookupTable::get_value(float key) {
    auto it = std::lower_bound(
        table.begin(), table.end(), key,
        [](const std::pair<float, float>& pair, float value) {
            return pair.first < value;
        }
    );

    if (it == table.begin())
        return it->second;

    if (it == table.end())
        return table.back().second;

    // linearly interpolate between the two surrounding points
    auto [x1, y1] = *(it - 1);
    auto [x2, y2] = *it;
    float slope = (y2 - y1) / (x2 - x1);
    return y1 + slope * (key - x1);

}