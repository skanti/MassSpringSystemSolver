#ifndef LB_ENGINE_H
#define LB_ENGINE_H

#include "WindowManager.h"
#include "Visualization.h"
#include "Types.h"
#include "World.h"
#include <cmath>
#include <chrono>
#include <boost/algorithm/minmax_element.hpp>

namespace lb {

    unsigned int MODE = 0;
    bool RUNNING = true;

    class Engine {
    public:

        Engine()
                : windowManager(),
                  world(0) {
            windowManager.set_keycallback(&key_callback);
            Visualization::init();
        }

        static void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods) {
            if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
                glfwSetWindowShouldClose(WindowManager::MainWindow, 1);
            else if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)
                RUNNING = !RUNNING;
            else if (key == GLFW_KEY_M && action == GLFW_PRESS)
                return;
        }

        static Engine &get_instance() {
            return *engine;
        }

        static void init() {
            if (!engine) engine = std::unique_ptr<Engine>(new Engine());
        }

        void run() {
            timer_type clock;
            unsigned int iteration_counter = 0;
            time_point t0 = clock.now();
            std::chrono::milliseconds ms_per_frame;
            while (WindowManager::window_alive()) {
                // process pending events
                WindowManager::poll_events();
                // clean screen
                Visualization::clear_screen();
                // advance mechanics
                advance(iteration_counter, ms_per_frame.count());
                // render screen
                render();
                // do timing
                ms_per_frame = std::chrono::duration_cast<std::chrono::milliseconds>(clock.now() - t0);
                t0 = clock.now();
                // draw on screen
                WindowManager::swap_window_buffer();
            }
            // clean up and exit
            WindowManager::terminate_window();
        }

        void render() {
            std::string running = RUNNING == 0 ? "off" : "on";
            world->draw_all();
        }

        void advance(unsigned int &iteration_counter, long long int ms_per_frame) {
            if (RUNNING) {
                world->advance_all(iteration_counter, ms_per_frame);
                iteration_counter++;
            }
        }

        void add_world(World *w) {
            if (world == 0) world = w;
        }

    private:
        WindowManager windowManager;
        World *world;

        static std::unique_ptr<Engine> engine;
    };

    std::unique_ptr<Engine> Engine::engine;
} // lb

#endif //LB_ENGINE_H
