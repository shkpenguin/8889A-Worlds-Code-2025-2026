#include "miku/devices/distance.hpp"

void miku::Distance::update_reading() {

    mutex.take();

    if(!enabled) valid = false;

    bool visible = this->get_object_size() > 30 || this->get_distance() < 100;

    bool within_range = this->get_distance() < 2000;

    if(visible && within_range) {
        data = this->get_distance() / 25.4;
        valid = true;
    } else {
        data = -1;
        valid = false;
    }
    
    mutex.give();
    
}