#pragma once 

#include <random>
#include <memory>
#include "Visualization.h"
#include "Nodes.h"
#include "Springs.h"
#include "World.h"
#include "ImageManager.h"
#include "Common.h"

class MSNWorld : public ga::World {
public:
    MSNWorld();

    static void init();

    static MSNWorld &get_instance();

    void draw();

    static void mouse_button_callback(GLFWwindow *window, int button, int action, int mods);

    static void keyboard_callback(GLFWwindow *window, int key, int scancode, int action, int mods);

    void load_file(std::string file);

    void init_shape();

    void init_shader();

    void init_instances();

    void init_drawable();

    void advance(std::size_t &iteration_counter, long long int ms_per_frame);

    void gather_for_rendering();

    void spawn_nodes(float px, float py);

    void delete_nodes(float px, float py);

private:
    std::mt19937 mt;
    std::normal_distribution<double> dist_normal;
    int is_floating;
    double zoom;
    ga::GLSLProgram mass_spring_program;
    Eigen::SimplicialLDLT<SparseMatrix<double>> chol;
    Nodes<double> nodes;
    Springs<double> springs;
    Vector<int> T0;
    Vector<int> T1;
    SparseMatrix<double> A;
    SparseMatrix<double> J;
    SparseMatrixCSR<double> J1;
    SparseMatrix<double> Q;
    SparseMatrix<double> Qinv;
    SparseMatrix<double> M;
    Vector<double> f_gravity;
    Vector<double> fx_langevin;
    Vector<double> fy_langevin;
    Vector<double> fz_langevin;
    ga::VAOMassSpring3D vao;
    static std::unique_ptr<MSNWorld> msn2d_world;
};
