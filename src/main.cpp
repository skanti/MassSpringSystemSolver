#include "World.h"
#include "Engine.h"
#include <regex>
#include <unordered_map>
#include "MSN2DWorld.h"

std::unique_ptr<MSN2DWorld> MSN2DWorld::msn2d_world = nullptr;

int main(int argc, char *argv[]) {
    ga::Engine::init("Mass-Spring Network", 0, 0, 800, 600);
    MSN2DWorld::init();
    ga::Engine::get_instance().set_world(&MSN2DWorld::get_instance());
    ga::Engine::get_instance().set_mouse_button_callback(&MSN2DWorld::mouse_button_callback);
    ga::Engine::get_instance().run();
    return 0;
}
