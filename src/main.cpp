#include "World.h"
#include "Engine.h"
#include <regex>
#include <unordered_map>
#include "MSNWorld.h"

std::unique_ptr<MSNWorld> MSNWorld::msn2d_world = nullptr;

int main(int argc, char *argv[]) {
    ga::Engine::init("Mass-Spring Network", 0, 0, 800, 600);
    MSNWorld::init();
    ga::Engine::get_instance().set_world(&MSNWorld::get_instance());
    ga::Engine::get_instance().set_mouse_button_callback(&MSNWorld::mouse_button_callback);
    ga::Engine::get_instance().set_key_callback(&MSNWorld::keyboard_callback);
    ga::Engine::get_instance().run();
    return 0;
}
