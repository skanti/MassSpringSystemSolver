#include "Engine.h"
#include "MSNWorld.h"


int main(int argc, char *argv[]) {
    gf::Engine::init("Mass-Spring Network", 0, 0, 1280, 720);
   	MSNWorld msn_world;
	gf::Engine::get_instance().set_world(&msn_world);
    gf::Engine::get_instance().run();

    return 0;
}
