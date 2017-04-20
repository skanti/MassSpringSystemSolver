#include "MSNWorld.h"
#include "MeshLoader.h"
#include "MathKernels.h"
#include "Timer.h"
#include "Common.h"
#include <iostream>
#include <fstream>
#include <string>

const double dt = 0.2;
#define N_MAX_NODES 5000
#define N_MAX_SPRINGS 20000
#define N_CLOTH 50

MSNWorld::MSNWorld() {
    flag = 1;
    is_floating = 0;
    i_counter = 0;
    zoom = 0.7f;
    timer0 = 0;

    // -> reserve memory
    nodes.reserve(N_MAX_NODES);
    springs.reserve(N_MAX_SPRINGS);
    A.reserve(2*N_MAX_SPRINGS);
    A.resize(N_MAX_NODES, N_MAX_SPRINGS);
    J.reserve(2*N_MAX_SPRINGS);
    J.resize(N_MAX_NODES, N_MAX_SPRINGS);
    J1.reserve(2*N_MAX_SPRINGS);
    J1.resize(N_MAX_NODES, N_MAX_SPRINGS);
    int h1 = -2;
    std::generate(&J.outerIndexPtr()[0], &J.outerIndexPtr()[0] + N_MAX_SPRINGS + 1, [&h1]{return h1 += 2;});
    Q.reserve(N_MAX_NODES + 2*N_MAX_SPRINGS);
    Q.resize(N_MAX_NODES, N_MAX_NODES);
    M.reserve(N_MAX_NODES);
    M.resize(N_MAX_NODES, N_MAX_NODES);
    K.reserve(N_MAX_SPRINGS);
    K.resize(N_MAX_SPRINGS, N_MAX_SPRINGS);
    MH1.reserve(3*N_MAX_NODES);
    MH1.resize(3*N_MAX_NODES, 3*N_MAX_NODES);
    H.reserve(9*(N_MAX_NODES + 2*N_MAX_SPRINGS));
    H.resize(3*N_MAX_NODES, 3*N_MAX_NODES);
    G.resize(3*N_MAX_NODES);
    px_tmp.resize(2);
    py_tmp.resize(2);
    // <-

    // -> load and set mesh
    create_cloth<double>(nodes, springs, A, N_CLOTH);
    normalize_and_recenter_nodes<double>(nodes);
    springs.set_as_equilibrium(nodes.px, nodes.py, nodes.pz, A);
    // <-

    MH1.setIdentity();
    M.setIdentity();

    px_tmp << nodes.px[0], nodes.px[N_CLOTH - 1];
    py_tmp << nodes.py[0], nodes.py[N_CLOTH - 1];

    K.setIdentity();
    std::fill(&K.valuePtr()[0], &K.valuePtr()[0] + N_MAX_SPRINGS, 50);

    // mt.seed(time(0));
    // mt.seed(999);

    f_gravity = Eigen::MatrixXd::Constant(N_MAX_NODES, 1, -0.001f);
    fx_langevin = Eigen::MatrixXd::Constant(N_MAX_NODES, 1, 0);
    fy_langevin = Eigen::MatrixXd::Constant(N_MAX_NODES, 1, 0);
    fz_langevin = Eigen::MatrixXd::Constant(N_MAX_NODES, 1, 0);


    J.leftCols(springs.n_size) = A.leftCols(springs.n_size);
    Q.leftCols(nodes.n_size) = M.block(0, 0, nodes.n_size, nodes.n_size) 
    + (SparseMatrix<double>)(dt*dt*J.block(0, 0, nodes.n_size, springs.n_size)*K.block(0, 0, springs.n_size, springs.n_size)*J.block(0, 0, nodes.n_size, springs.n_size).transpose());
            
    chol.compute(Q.block(0,0, nodes.n_size, nodes.n_size));

    // -> set up hessian CSC container
    H.outerIndexPtr()[0] = 0;
    for (int i = 0, l = 0; i < nodes.n_size; i++) {
        H.outerIndexPtr()[3*i + 1] = 3*(Q.outerIndexPtr()[i + 1] - Q.outerIndexPtr()[i]) + H.outerIndexPtr()[3*i];
        H.outerIndexPtr()[3*i + 2] = 6*(Q.outerIndexPtr()[i + 1] - Q.outerIndexPtr()[i]) + H.outerIndexPtr()[3*i];
        H.outerIndexPtr()[3*i + 3] = 9*(Q.outerIndexPtr()[i + 1] - Q.outerIndexPtr()[i]) + H.outerIndexPtr()[3*i];

        for (int k = 0; k < 3; k++) {
            for (int j = Q.outerIndexPtr()[i]; j < Q.outerIndexPtr()[i + 1]; j++) {
                H.valuePtr()[l + 0] = 1;
                H.valuePtr()[l + 1] = 1;
                H.valuePtr()[l + 2] = 1;
                H.innerIndexPtr()[l + 0] = 3*Q.innerIndexPtr()[j] + 0;
                H.innerIndexPtr()[l + 1] = 3*Q.innerIndexPtr()[j] + 1;
                H.innerIndexPtr()[l + 2] = 3*Q.innerIndexPtr()[j] + 2;
                l += 3;
            }
        }
    }
	std::fill(&H.outerIndexPtr()[3*nodes.n_size + 1], &H.outerIndexPtr()[3*N_MAX_NODES + 1], H.outerIndexPtr()[3*nodes.n_size]);
	// <-    
    
    // -> create spring hashmap
    for (int i = 0, k = 0; i < nodes.n_size; i++) {
        for (int j = Q.outerIndexPtr()[i], m = 0; j < Q.outerIndexPtr()[i + 1]; j++, m++) {
            int64_t a = i;
            int64_t b = Q.innerIndexPtr()[j];
            int64_t c = (a << 32) + b;

            if(L1.find(c) == L1.end()) 
                L1.insert({c, m});

            if (b > a) 
                L.insert({c, k++});
        }
    }
    // <- 
    
    
}

void MSNWorld::init() {
    if (!msn2d_world) msn2d_world = std::unique_ptr<MSNWorld>(new MSNWorld());
}

void MSNWorld::kill() {
    msn2d_world.reset();
}

MSNWorld &MSNWorld::get_instance() {
    return *msn2d_world;
}


void MSNWorld::init_filename_and_seed_and_method(std::string optimization_method, int n_iteration_optimization, double sigma_langevin, int i_run) {
	mt.seed(time(0) + i_run);
	
	if (optimization_method == "pd")
	    optimization_method1 = 1;
	else if (optimization_method == "newton")
	    optimization_method1 = 2;
	else
	    exit(0);
	
	char buf[12];
	std::sprintf(buf, "%1.8f", sigma_langevin);
	std::string filename0  = "/cluster/scratch/armena/" + optimization_method + std::to_string(n_iteration_optimization) + "-langevin" + std::string(buf);
	filename  = filename0 + "-traj-irun" + std::to_string(i_run) + ".dat";
	filename2 = filename0 + "-timing-irun" + std::to_string(i_run) + ".dat";
}

void MSNWorld::advance(std::size_t &iteration_counter, int n_iteration_optimization, double sigma_langevin) {

    double h2 = dt*dt;

    std::generate(&fx_langevin[0], &fx_langevin[0] + nodes.n_size, [&](){return sigma_langevin*dist_normal(mt);});
    std::generate(&fy_langevin[0], &fy_langevin[0] + nodes.n_size, [&](){return sigma_langevin*dist_normal(mt);});
    std::generate(&fz_langevin[0], &fz_langevin[0] + nodes.n_size, [&](){return sigma_langevin*dist_normal(mt);});

    nodes.qx.block(0, 0, nodes.n_size, 1) = nodes.px.block(0, 0, nodes.n_size, 1) + nodes.vx.block(0, 0, nodes.n_size, 1)*dt*(1.0f - dt*0.1f); 
    nodes.qy.block(0, 0, nodes.n_size, 1) = nodes.py.block(0, 0, nodes.n_size, 1) + nodes.vy.block(0, 0, nodes.n_size, 1)*dt*(1.0f - dt*0.1f);
    nodes.qz.block(0, 0, nodes.n_size, 1) = nodes.pz.block(0, 0, nodes.n_size, 1) + nodes.vz.block(0, 0, nodes.n_size, 1)*dt*(1.0f - dt*0.1f);

    
    nodes.vx.block(0, 0, nodes.n_size, 1) = nodes.px.block(0, 0, nodes.n_size, 1);
    nodes.vy.block(0, 0, nodes.n_size, 1) = nodes.py.block(0, 0, nodes.n_size, 1);
    nodes.vz.block(0, 0, nodes.n_size, 1) = nodes.pz.block(0, 0, nodes.n_size, 1);
    
    nodes.px.block(0, 0, nodes.n_size, 1) = nodes.qx.block(0, 0, nodes.n_size, 1); 
    nodes.py.block(0, 0, nodes.n_size, 1) = nodes.qy.block(0, 0, nodes.n_size, 1);
    nodes.pz.block(0, 0, nodes.n_size, 1) = nodes.qz.block(0, 0, nodes.n_size, 1);
	
    Timer::start();
    switch(optimization_method1){

    case 1:

        for (int i = 0; i < n_iteration_optimization; i++) {
            springs.dx_rhs.block(0, 0, springs.n_size, 1) = (nodes.px.block(0, 0, nodes.n_size, 1).transpose()*J.block(0, 0, nodes.n_size, springs.n_size)*K.block(0, 0, springs.n_size, springs.n_size)).transpose();
            springs.dy_rhs.block(0, 0, springs.n_size, 1) = (nodes.py.block(0, 0, nodes.n_size, 1).transpose()*J.block(0, 0, nodes.n_size, springs.n_size)*K.block(0, 0, springs.n_size, springs.n_size)).transpose();
            springs.dz_rhs.block(0, 0, springs.n_size, 1) = (nodes.pz.block(0, 0, nodes.n_size, 1).transpose()*J.block(0, 0, nodes.n_size, springs.n_size)*K.block(0, 0, springs.n_size, springs.n_size)).transpose();


            for (int j = 0; j < springs.n_size; j++) {
                springs.d_rhs[j] = std::sqrt(springs.dx_rhs[j]*springs.dx_rhs[j] + springs.dy_rhs[j]*springs.dy_rhs[j] + springs.dz_rhs[j]*springs.dz_rhs[j]);
                springs.dx[j] = springs.d[j]*springs.dx_rhs[j]/springs.d_rhs[j];
                springs.dy[j] = springs.d[j]*springs.dy_rhs[j]/springs.d_rhs[j];
                springs.dz[j] = springs.d[j]*springs.dz_rhs[j]/springs.d_rhs[j];
            }

            nodes.px_rhs.block(0, 0, nodes.n_size, 1) = M.block(0, 0, nodes.n_size, nodes.n_size)*nodes.qx.block(0, 0, nodes.n_size, 1) + h2*(J.block(0, 0, nodes.n_size, springs.n_size)
                *K.block(0, 0, springs.n_size, springs.n_size)*springs.dx.block(0, 0, springs.n_size, 1) + fx_langevin.block(0, 0, nodes.n_size, 1));
            nodes.py_rhs.block(0, 0, nodes.n_size, 1) = M.block(0, 0, nodes.n_size, nodes.n_size)*nodes.qy.block(0, 0, nodes.n_size, 1) + h2*(J.block(0, 0, nodes.n_size, springs.n_size)
                *K.block(0, 0, springs.n_size, springs.n_size)*springs.dy.block(0, 0, springs.n_size, 1) + fy_langevin.block(0, 0, nodes.n_size, 1) + f_gravity.block(0, 0, nodes.n_size, 1));
            nodes.pz_rhs.block(0, 0, nodes.n_size, 1) = M.block(0, 0, nodes.n_size, nodes.n_size)*nodes.qz.block(0, 0, nodes.n_size, 1) + h2*(J.block(0, 0, nodes.n_size, springs.n_size)
                *K.block(0, 0, springs.n_size, springs.n_size)*springs.dz.block(0, 0, springs.n_size, 1) + fz_langevin.block(0, 0, nodes.n_size, 1));


            nodes.px.block(0, 0, nodes.n_size, 1) = chol.solve(nodes.px_rhs.block(0, 0, nodes.n_size, 1));
            nodes.py.block(0, 0, nodes.n_size, 1) = chol.solve(nodes.py_rhs.block(0, 0, nodes.n_size, 1));
            nodes.pz.block(0, 0, nodes.n_size, 1) = chol.solve(nodes.pz_rhs.block(0, 0, nodes.n_size, 1));
        }
        break;

    case 2:
        
        for (int l = 0; l < n_iteration_optimization; l++) {
            std::fill(&G[0], &G[0] + 3*nodes.n_size, 0);
            for (int i = 0; i < springs.n_size; i++) {
                int a = J.innerIndexPtr()[i*2 + 0];
                int b = J.innerIndexPtr()[i*2 + 1];
                double dx = nodes.px[a] - nodes.px[b];
                double dy = nodes.py[a] - nodes.py[b];
                double dz = nodes.pz[a] - nodes.pz[b];
                double d = std::sqrt(dx*dx + dy*dy + dz*dz);
                G[3*a + 0] += K.valuePtr()[i]*(d - springs.d[i])*dx/d;
                G[3*a + 1] += K.valuePtr()[i]*(d - springs.d[i])*dy/d;
                G[3*a + 2] += K.valuePtr()[i]*(d - springs.d[i])*dz/d;

                G[3*b + 0] -= K.valuePtr()[i]*(d - springs.d[i])*dx/d;
                G[3*b + 1] -= K.valuePtr()[i]*(d - springs.d[i])*dy/d;
                G[3*b + 2] -= K.valuePtr()[i]*(d - springs.d[i])*dz/d;
            }
            
            for (int i = 0; i < nodes.n_size; i++) {
                G[3*i + 0] = M.valuePtr()[i]*(nodes.px[i] - nodes.qx[i]) + h2*(G[3*i + 0] + fx_langevin[i]);
                G[3*i + 1] = M.valuePtr()[i]*(nodes.py[i] - nodes.qy[i]) + h2*(G[3*i + 1] + fy_langevin[i] - f_gravity[i]);
                G[3*i + 2] = M.valuePtr()[i]*(nodes.pz[i] - nodes.qz[i]) + h2*(G[3*i + 2] + fz_langevin[i]);
            }
            // std::cout << G.block(0, 0, 3*nodes.n_size, 1).squaredNorm() << std::endl;

            if (G.block(0, 0, 3*nodes.n_size, 1).lpNorm<Eigen::Infinity>() < 1e-16) break;

            std::fill(&H.valuePtr()[0], &H.valuePtr()[0] + 9*(nodes.n_size + 2*springs.n_size), 0);

            for (int i = 0, k = 0; i < nodes.n_size; i++) {
                for (int j = Q.outerIndexPtr()[i]; j < Q.outerIndexPtr()[i + 1]; j++, k++) {
                    int64_t a0 = i;
                    int64_t b0 = Q.innerIndexPtr()[j];
                    if (b0 > a0) {
                        int64_t c0 = (a0 << 32) + b0;
                        int64_t c1 = (b0 << 32) + a0;
                        int64_t c2 = (a0 << 32) + a0;
                        int64_t c3 = (b0 << 32) + b0;
                        int32_t s = L[c0];

                        double dx = nodes.px[a0] - nodes.px[b0];
                        double dy = nodes.py[a0] - nodes.py[b0];
                        double dz = nodes.pz[a0] - nodes.pz[b0];
                        double d = std::sqrt(dx*dx + dy*dy + dz*dz);

                        Eigen::Vector3d pab(dx, dy, dz);

                        Eigen::Matrix3d H1 = K.valuePtr()[s]*(Eigen::Matrix3d::Identity() - springs.d[s]/d*(Eigen::Matrix3d::Identity() - pab*pab.transpose()/(d*d)));

                        int32_t a1 = H.outerIndexPtr()[a0*3 + 0];
                        int32_t a2 = H.outerIndexPtr()[a0*3 + 1];
                        int32_t a3 = H.outerIndexPtr()[a0*3 + 2];

                        int32_t b1 = H.outerIndexPtr()[b0*3 + 0];
                        int32_t b2 = H.outerIndexPtr()[b0*3 + 1];
                        int32_t b3 = H.outerIndexPtr()[b0*3 + 2];

                        int32_t a4 = L1[c2];
                        int32_t b4 = L1[c3];
                        int32_t ab4 = L1[c0];
                        int32_t ba4 = L1[c1];
                    
                #define HADD(r0, r1, r2, c0, o0) { H.valuePtr()[r0 + 3*c0 + 0] += o0*H1(0,0);\
                    H.valuePtr()[r0 + 3*c0 + 1] += o0*H1(1,0);\
                    H.valuePtr()[r0 + 3*c0 + 2] += o0*H1(2,0);\
                    H.valuePtr()[r1 + 3*c0 + 0] += o0*H1(0,1);\
                    H.valuePtr()[r1 + 3*c0 + 1] += o0*H1(1,1);\
                    H.valuePtr()[r1 + 3*c0 + 2] += o0*H1(2,1);\
                    H.valuePtr()[r2 + 3*c0 + 0] += o0*H1(0,2);\
                    H.valuePtr()[r2 + 3*c0 + 1] += o0*H1(1,2);\
                    H.valuePtr()[r2 + 3*c0 + 2] += o0*H1(2,2);};


                        HADD(a1, a2, a3, a4, 1.0);
                        HADD(a1, a2, a3, ab4, -1.0);
                        HADD(b1, b2, b3, ba4, -1.0);
                        HADD(b1, b2, b3, b4, 1.0);

                    }
                }
            }
			
            H.leftCols(3*nodes.n_size) = MH1.leftCols(3*nodes.n_size) + h2*H.leftCols(3*nodes.n_size);            
            //  H.leftCols(3*nodes.n_size) = H.leftCols(3*nodes.n_size);
            
            Eigen::SimplicialLDLT<SparseMatrix<double>> ldlt_solver;
            
            // ldlt_solver.analyzePattern(H.block(0, 0, 3*nodes.n_size, 3*nodes.n_size));
            // ldlt_solver.factorize(H.block(0, 0, 3*nodes.n_size, 3*nodes.n_size));
            // double regularization = 1e-6;
            // while (ldlt_solver.info() != Eigen::Success) {
            //     std::cout << "regularized" << std::endl;
            //     regularization *= 10;
            //     H.leftCols(3*nodes.n_size) = H.leftCols(3*nodes.n_size) + regularization*MH1.leftCols(3*nodes.n_size);
            //     ldlt_solver.factorize(H.block(0, 0, 3*nodes.n_size, 3*nodes.n_size));
            // }
            
            ldlt_solver.compute(H.block(0, 0, 3*nodes.n_size, 3*nodes.n_size));

            Vector<double> desc = ldlt_solver.solve(G.block(0, 0, 3*nodes.n_size, 1));

            for (int i = 0; i < nodes.n_size; i++) {
                nodes.px[i] = nodes.px[i] - desc[3*i + 0];
                nodes.py[i] = nodes.py[i] - desc[3*i + 1];
                nodes.pz[i] = nodes.pz[i] - desc[3*i + 2];
            }

            // nodes.pz[0] = 0.2*std::sin(iteration_counter*5e-3);
            // nodes.pz[N_CLOTH - 1] = 0.2*std::sin(iteration_counter*5e-3);   
        }
        break;

	default:
		exit(0);
		break;
    }

    Timer::stop();

    // -> fixed constraints
    nodes.px[0] = px_tmp[0];
    nodes.py[0] = py_tmp[0];
    nodes.pz[0] = 0.4*std::sin(iteration_counter*1e-2);
    nodes.px[N_CLOTH - 1] = px_tmp[1];
    nodes.py[N_CLOTH - 1] = py_tmp[1];
    nodes.pz[N_CLOTH - 1] = 0.4*std::sin(iteration_counter*1e-2);
    // <-

    nodes.vx.block(0, 0, nodes.n_size, 1) = (nodes.px.block(0, 0, nodes.n_size, 1) - nodes.vx.block(0, 0, nodes.n_size, 1))/dt;
    nodes.vy.block(0, 0, nodes.n_size, 1) = (nodes.py.block(0, 0, nodes.n_size, 1) - nodes.vy.block(0, 0, nodes.n_size, 1))/dt;
    nodes.vz.block(0, 0, nodes.n_size, 1) = (nodes.pz.block(0, 0, nodes.n_size, 1) - nodes.vz.block(0, 0, nodes.n_size, 1))/dt;

    if (iteration_counter % 8 == 0) {
        // -> trajectory
        std::ofstream file;
        file.open (filename, std::ios::app);
        for (int i = 0; i < nodes.n_size; i++)
            file << nodes.px[i] << " " << nodes.py[i] << " " << nodes.pz[i] << std::endl;
        file << std::endl;
        file.close();
        // <-

        // -> timing
        file.open (filename2, std::ios::app);
        file << Timer::get_timing() << std::endl;
        file.close();
        // <-
    }

    // J1.topRows(nodes.n_size) = J.leftCols(springs.n_size);    
    // gather_for_rendering();
}


