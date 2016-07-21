#include "Earth.h"

Earth::Earth() : World(), mss() {};

void Earth::init() {
    if (!earth) earth = std::unique_ptr<Earth>(new Earth());
}

Earth &Earth::get_instance() {
    return *earth;
}

void Earth::advance(std::size_t &iteration_counter, long long int ms_per_frame) { mss.move(); }

void Earth::draw() { mss.draw(); }
