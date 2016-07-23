#include "Earth.h"

Earth::Earth() : World(), mss() {};

void Earth::init() {
    if (!earth) earth = std::unique_ptr<Earth>(new Earth());
}

Earth &Earth::get_instance() {
    return *earth;
}

void Earth::advance(std::size_t &iteration_counter, long long int ms_per_frame) { mss.move(); }

void Earth::draw() { mss.draw(); }

void Earth::mouse_button_callback(GLFWwindow *window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        double p_x_target, p_y_target;
        ga::Engine::get_instance().get_cursor_position(&p_x_target, &p_y_target);
        p_x_target = p_x_target * 2.0 / WINDOW_WIDTH - 1.0;
        p_y_target = (-p_y_target * 2.0 / WINDOW_HEIGTH + 1.0) * WINDOW_ASPECTRATIO_INV;
        glm::mat4 z = glm::scale(glm::mat4(1), glm::vec3(0.3, 0.3, 1.0));
        glm::mat4 t = glm::translate(glm::mat4(1), glm::vec3(0, 0.3, 0));
        glm::vec4 p = glm::inverse(z) * t * glm::vec4(p_x_target, p_y_target, 0, 1);
        get_instance().mss.spawn_floating_nodes(p[0], p[1]);
    }
}