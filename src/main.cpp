#include "World.h"
#include "Engine.h"
#include <regex>
#include <unordered_map>
#include "Earth.h"

float DT = 0.001;

/*
class BlankWorld : public ga::World {

    void advance(std::size_t &iteration_counter, long long int ms_per_frame) {};

    void draw() {};

    static std::unique_ptr<BlankWorld> blank_world;
};
*/

std::unique_ptr<Earth> Earth::earth = nullptr;

int main(int argc, char *argv[]) {
    ga::Engine::init("Springs2D");
    Earth::init();
    ga::Engine::get_instance().set_world(&Earth::get_instance());
    ga::Engine::get_instance().set_mouse_button_callback(&Earth::mouse_button_callback);
    ga::Engine::get_instance().run();
    return 0;
}
