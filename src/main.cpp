#include "World.h"
#include "Engine.h"
#include <regex>
#include <unordered_map>
#include "MSNWorld.h"

std::unique_ptr<MSNWorld> MSNWorld::msn2d_world = nullptr;

int main(int argc, char *argv[]) {
    // ga::Engine::init("Mass-Spring Network", 0, 0, 1280, 720);
    // MSNWorld::init();
    // ga::Engine::get_instance().set_world(&MSNWorld::get_instance());
    // ga::Engine::get_instance().set_mouse_button_callback(&MSNWorld::mouse_button_callback);
    // ga::Engine::get_instance().set_key_callback(&MSNWorld::keyboard_callback);
    // ga::Engine::get_instance().run();

    for (std::size_t i = 0; i < 1; i++) {
        MSNWorld::init();
        std::cout << "i_traj: " << (i + 1) << std::endl;
        for (std::size_t j = 0; j < 20*60; j++) {
            // std::cout << (j + 1) <<  "\r";
            std::cout << (j + 1) <<  std::endl;
        	MSNWorld::get_instance().advance(j, i);
            fflush(stdout);
        }
        std::cout << std::endl;
        MSNWorld::kill();
    }
    
    return 0;
}
