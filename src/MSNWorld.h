#pragma once 

#include <random>
#include <memory>
#include <unordered_map>
#include "Nodes.h"
#include "Springs.h"
#include "Common.h"
#include <string>

class MSNWorld {
public:
    MSNWorld();

    static void init();
	
    static void kill();

    static MSNWorld &get_instance();

	void init_filename_and_seed_and_method(std::string optimization_method, int n_iteration_optimization, double sigma_langevin, int i_run);

    void advance(std::size_t &iteration_counter, int n_iteration_optimization, double sigma_langevin);


private:
    int optimization_method1;
	std::string filename, filename2;
    int i_counter;
    int flag;
    double timer0;
    std::mt19937 mt;
    std::normal_distribution<double> dist_normal;
    std::uniform_int_distribution<int> dist_int_uniform;
    int is_floating;
    double zoom;
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
    static std::unique_ptr<MSNWorld> msn2d_world;
};
