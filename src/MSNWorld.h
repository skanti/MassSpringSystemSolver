#pragma once 

#include "Common.h"
#include <random>
#include <memory>
#include <unordered_map>
#include "gf/Visualization.h"
#include "Nodes.h"
#include "Springs.h"
#include "gf/World.h"

class MSNWorld : public gf::World {
public:

    void init();

	void term();

    void draw();

    void keyboard_callback(GLFWwindow *window, int key, int scancode, int action, int mods);

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
    int i_counter;
    int flag;
    std::mt19937 mt;
    std::normal_distribution<double> dist_normal;
    std::uniform_int_distribution<int> dist_int_uniform;
    int is_floating;
    double zoom;
    gf::GLSLProgram mass_spring_program;
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
    Matrix<double> f_gravity;
    Matrix<double> f_langevin;
    Vector3<double> p0_tmp;
    Vector3<double> p1_tmp;
    gf::VAOMassSpring3D vao;
};
