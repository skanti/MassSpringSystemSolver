#include "World.h"
#include "Engine.h"
#include "MSNWorld.h"

std::unique_ptr<MSNWorld> MSNWorld::msn2d_world = nullptr;

int main(int argc, char *argv[]) {
    ga::Engine::init("Mass-Spring Network", 0, 0, 1280, 720);
    MSNWorld::init();
    ga::Engine::get_instance().set_world(&MSNWorld::get_instance());
    ga::Engine::get_instance().set_mouse_button_callback(&MSNWorld::mouse_button_callback);
    ga::Engine::get_instance().set_key_callback(&MSNWorld::keyboard_callback);
    ga::Engine::get_instance().run();

    // MSNWorld::init();
    // for (std::size_t i = 0; i < 60*60; i++) {
    // 	std::cout << i << std::endl;
    // 	MSNWorld::get_instance().advance(i, i);
    // }
    
    return 0;
}
