#pragma once 

#include <random>
#include <memory>
#include <unordered_map>
#include "Visualization.h"
#include "Nodes.h"
#include "Springs.h"
#include "World.h"
#include "ImageManager.h"
#include "Common.h"

class MSNWorld {
public:
    MSNWorld();

    static void init();

    static void kill();

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

    void create_springs(int i_node);

    void spawn_nodes(float px, float py);

    void delete_nodes(float px, float py);

private:
    std::string filename;
    int i_counter;
    int flag;
    double timer0;
    std::mt19937 mt;
    std::normal_distribution<double> dist_normal;
    std::uniform_int_distribution<int> dist_int_uniform;
    int is_floating;
    double zoom;
    ga::GLSLProgram mass_spring_program;
    Eigen::SimplicialLDLT<SparseMatrix<double>> chol;
    Nodes<double> nodes;
    Springs<double> springs;
    SparseMatrix<double> A;
    SparseMatrix<double> J;
    SparseMatrixCSR<double> J1;
    std::unordered_map<int64_t, int32_t> L;
    std::unordered_map<int64_t, int32_t> L1;
    SparseMatrix<double> Q;
    SparseMatrix<double> Qinv;
    SparseMatrix<double> M;
    SparseMatrix<double> K;
    SparseMatrix<double> MH1;
    SparseMatrix<double> H;
    Vector<double> G;
    Vector<double> f_gravity;
    Vector<double> fx_langevin;
    Vector<double> fy_langevin;
    Vector<double> fz_langevin;
    Vector<double> px_tmp;
    Vector<double> py_tmp;
    ga::VAOMassSpring3D vao;
    static std::unique_ptr<MSNWorld> msn2d_world;
};
