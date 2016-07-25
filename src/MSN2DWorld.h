#ifndef SPRINGS2D_EARTH_H
#define SPRINGS2D_EARTH_H

#include "Visualization.h"
#include "AlignedAllocator.h"
#include "Nodes.h"
#include "Springs.h"
#include "World.h"
#include <memory>
#include "ImageManager.h"

class MSN2DWorld : public ga::World {
public:
    MSN2DWorld();

    static void init();

    static MSN2DWorld &get_instance();

    void draw();

    static void mouse_button_callback(GLFWwindow *window, int button, int action, int mods);

    void load_file(std::string file);

    void init_shape();

    void init_instances();

    void init_drawable();

    void advance(std::size_t &iteration_counter, long long int ms_per_frame);

    void springs_attach_detach();

    void gather_for_rendering();

    void spawn_floating_nodes(double pxi, double pyi);

private:
    Nodes nodes;
    Springs springs;
    ga::VAOMassSpring vao;
    static std::unique_ptr<MSN2DWorld> msn2d_world;
};


#endif //SPRINGS2D_EARTH_H
