#ifndef SPRINGS2D_EARTH_H
#define SPRINGS2D_EARTH_H

#include "MassSpringSystem.h"
#include "World.h"
#include <memory>
#include "ImageManager.h"

class Earth : public ga::World {
public:
    Earth();

    static void init();

    static Earth &get_instance();

    void advance(std::size_t &iteration_counter, long long int ms_per_frame);

    void draw();

    static void mouse_button_callback(GLFWwindow *window, int button, int action, int mods);

    static std::unique_ptr<Earth> earth;
    MassSpringSystem mss;
};


#endif //SPRINGS2D_EARTH_H
