#ifndef LB_WINDOWMANAGER_H
#define LB_WINDOWMANAGER_H

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>

namespace lb {

    class WindowManager {
    public :
        WindowManager() {
            init_glfw();
        }

        static void init_glfw() {
            if (!glfwInit())
                throw std::runtime_error("glfwInit failed");

            // open a window with GLFW
            glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
            glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
            glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
            glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
            glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
            glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);
            MainWindow = glfwCreateWindow(800, 600, "Some OpenGL scratches...", NULL, NULL);
            if (!MainWindow)
                throw std::runtime_error("glfwOpenWindow failed. Can your hardware handle OpenGL 4.1?");

            // GLFW settings
            glfwMakeContextCurrent(MainWindow);
            // initialise GLEW
            glewExperimental = GL_TRUE; //stops glew crashing on OSX :-/
            if (glewInit() != GLEW_OK)
                throw std::runtime_error("glewInit failed\n");

            // print out some info about the graphics drivers
            std::cout << "OpenGL version: " << glGetString(GL_VERSION) << std::endl;
            std::cout << "GLSL version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl;
            std::cout << "Vendor: " << glGetString(GL_VENDOR) << std::endl;
            std::cout << "WindowManager: " << glGetString(GL_RENDERER) << std::endl;

            // make sure OpenGL version 4.1 API is available
            if (!GLEW_VERSION_4_1)
                throw std::runtime_error("OpenGL 4.1 API is not available.");

            //glEnable(GL_DEBUG_OUTPUT);
            //glDebugMessageCallback( debugCallback, NULL );
            //glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, NULL, GL_TRUE);
        }


        static void debugCallback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar * message, void * param) {

            // Convert GLenum parameters to strings

            printf("%s:%s[%s](%d): %s\n", "source", "type", "hard", id, message);

        }

        static void set_keycallback (void (key_callback) (GLFWwindow *, int , int , int , int ) ) {
            // set keycall back
            glfwSetKeyCallback(MainWindow, key_callback);
        }

        static int window_alive() {
            return !glfwWindowShouldClose(MainWindow);
        }

        static void poll_events() {
            glfwPollEvents();
        }

        static void swap_window_buffer() {
            glfwSwapBuffers(WindowManager::MainWindow);
        }

        static void terminate_window() {
            // clean up and exit
            glfwTerminate();
        }


    public:

        static GLFWwindow *MainWindow;
    };

    GLFWwindow *WindowManager::MainWindow = NULL;
}
#endif