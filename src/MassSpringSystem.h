#ifndef SPRINGS2D_MASSSPRINGSYSTEM_H
#define SPRINGS2D_MASSSPRINGSYSTEM_H

#include "Engine.h"
#include "Drawable.h"
#include "Shape.h"
#include "VertexObject.h"
#include "Nodes.h"
#include "Springs.h"
#include <utility>
#include <unordered_map>

class MassSpringSystem : public ga::Drawable {
public:
    MassSpringSystem();

    void load_file(std::string file);

    void init_shape();

    void init_instances();

    void init_drawable();

    void draw();

    void move();

    void attach_detach();

    void gather();

    void spawn_floating_nodes(double pxi, double pyi);

private:
    Nodes nodes;
    Springs springs;
    ga::VAOMassSpring vao;
};


#endif //SPRINGS2D_MASSSPRINGSYSTEM_H
