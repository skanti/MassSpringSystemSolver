#include "MSNWorld.h"
#include "Engine.h"
#include "MeshLoader.h"
#include <eigen3/Eigen/Sparse>
#include "MathKernels.h"
#include "Timer.h"
#include <iostream>
#include <fstream>

const double dt = 0.3;
#define N_MAX_NODES 5000
#define N_MAX_SPRINGS 30000
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
    H.reserve(3*(N_MAX_NODES + 2*N_MAX_SPRINGS));
    H.resize(3*N_MAX_NODES, 3*N_MAX_NODES);
    G.resize(3*N_MAX_NODES);
    px_tmp.resize(2);
    py_tmp.resize(2);
    // <-

    // -> load and set mesh
    //load_mesh_ply2<float>("/dtome/amon/grive/development/MassSpringNetwork/data/canstick.ply2", nodes, springs);
    create_cloth<double>(nodes, springs, A, N_CLOTH);
    normalize_and_recenter_nodes<double>(nodes);
    springs.set_as_equilibrium(nodes.px, nodes.py, nodes.pz, A);
    // <-

    MH1.setIdentity();
    // MH1.valuePtr()[0] = 1e6;
    // MH1.valuePtr()[1] = 1e6;
    // MH1.valuePtr()[2] = 1e6;
    // MH1.valuePtr()[3*(N_CLOTH - 1) + 0] = 1e6;
    // MH1.valuePtr()[3*(N_CLOTH - 1) + 1] = 1e6;
    // MH1.valuePtr()[3*(N_CLOTH - 1) + 2] = 1e6;

    M.setIdentity();
    // M.valuePtr()[0] = 1e6;
    // M.valuePtr()[N_CLOTH - 1] = 1e6;

    px_tmp << nodes.px[0], nodes.px[N_CLOTH - 1];
    py_tmp << nodes.py[0], nodes.py[N_CLOTH - 1];

    K.setIdentity();
    std::fill(&K.valuePtr()[0], &K.valuePtr()[0] + N_MAX_SPRINGS, 50);

    mt.seed(time(0));
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
    // <-    


    // Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> H11 = H.block(0, 0, 3*nodes.n_size, 3*nodes.n_size);
    // std::cout << H11 << std::endl;
    // std::cout << "*********" << std::endl;
    // exit(0);

    // for (int i = 0; i < 3*nodes.n_size + 1; i++) {
    //     std::cout << H.outerIndexPtr()[i] << " ";
    // }
    // std::cout << std::endl;
    
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
    
    auto does_file_exist = [](std::string filename) { std::ifstream infile(filename); return infile.good();};
    for (int i = 0; i < (1 << 20); i++) {
        filename = std::string("newton10-traj") + std::to_string(i) + std::string(".dat");
        if (!does_file_exist(filename)) break;   
    }

    // init_shader();
    // init_shape();
    // init_instances();
    // init_drawable();
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


void MSNWorld::init_shape() {
    glm::mat4 trans_mat = glm::translate(glm::mat4(1), glm::vec3(0, 0, 0));
    vao.model_matrix = trans_mat*glm::scale(glm::mat4(1), glm::vec3(zoom));
    vao.shape = ga::Shape::make_sphere(0.01f);
}

void MSNWorld::init_shader() {
    std::string dir = "/home/amon/grive/development/MassSpringNetwork/src/glsl";
    ga::Visualization::load_shaders(mass_spring_program, dir, "MassSpringVS3D.glsl", "MassSpringFS3D.glsl", "", "", "");
}

void MSNWorld::init_instances() {
    vao.n_instances = N_MAX_NODES;
    vao.p_xyz.resize(3*N_MAX_NODES);
    vao.color.resize(3*N_MAX_NODES);

    vao.n_springs = N_MAX_SPRINGS;
    vao.s_ab.resize(2*N_MAX_SPRINGS);
}

void MSNWorld::init_drawable() {
    glUseProgram(mass_spring_program.id);
    glGenVertexArrays(1, &vao.vao_id);
    glBindVertexArray(vao.vao_id);

    glUniform1f(mass_spring_program.uniform("l_base"), vao.shape.l_base);

    glUniformMatrix4fv(mass_spring_program.uniform("ModelMatrix"), 1, GL_FALSE, &vao.model_matrix[0][0]);
    glUniformMatrix4fv(mass_spring_program.uniform("ViewMatrix"), 1, GL_FALSE, &ga::Visualization::view_window[0][0]);
    glUniformMatrix4fv(mass_spring_program.uniform("ProjectionMatrix"), 1, GL_FALSE, &ga::Visualization::projection_window[0][0]);
    float nodes_color_alpha = 1.0;
    glUniform1f(mass_spring_program.uniform("nodes_color_alpha"), nodes_color_alpha);
    float springs_color[] = {0.8f, 0.2f, 0.2f, 0.8f};
    glUniform4fv(mass_spring_program.uniform("springs_color"), 1, springs_color);

    // -> vertices
    glGenBuffers(1, &vao.vbo_shape_id);
    glBindBuffer(GL_ARRAY_BUFFER, vao.vbo_shape_id);
    glBufferData(GL_ARRAY_BUFFER, 3*vao.shape.n_vertices*sizeof(GLfloat), vao.shape.vertices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glGenBuffers(1, &vao.vbo_shape_index_id);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vao.vbo_shape_index_id);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 3*vao.shape.n_segments*sizeof(GLuint), vao.shape.elements.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    // <-

    // -> node instances
    glGenBuffers(1, &vao.vbo_instance_id0);
    glBindBuffer(GL_ARRAY_BUFFER, vao.vbo_instance_id0);
    glBufferData(GL_ARRAY_BUFFER, 3*nodes.n_size*sizeof(GLfloat), vao.p_xyz.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    // <-

      // -> node instances color
    glGenBuffers(1, &vao.vbo_instance_id1);
    glBindBuffer(GL_ARRAY_BUFFER, vao.vbo_instance_id1);
    glBufferData(GL_ARRAY_BUFFER, 3*nodes.n_size*sizeof(GLfloat), vao.color.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(2);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    // <-

    // -> springs
    glGenBuffers(1, &vao.vbo_springs_id);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vao.vbo_springs_id);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 2*springs.n_size*sizeof(GLuint), vao.s_ab.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    // <-

    // -> culling options
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    // <-
};

void MSNWorld::draw() {
    
    glUseProgram(mass_spring_program.id);
    glBindVertexArray(vao.vao_id);

    // -> draw nodes
    glUniform1i(mass_spring_program.uniform("mode"), 0);
    glVertexAttribDivisor(1, 1);
    glVertexAttribDivisor(2, 1);
    glUniformMatrix4fv(mass_spring_program.uniform("ModelMatrix"), 1, GL_FALSE, &vao.model_matrix[0][0]);

    glBindBuffer(GL_ARRAY_BUFFER, vao.vbo_instance_id0);
    glBufferData(GL_ARRAY_BUFFER, 3*nodes.n_size*sizeof(GLfloat), vao.p_xyz.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, vao.vbo_instance_id1);
    glBufferData(GL_ARRAY_BUFFER, 3*nodes.n_size*sizeof(GLfloat), vao.color.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vao.vbo_shape_index_id);
    glDrawElementsInstanced(GL_TRIANGLES, 3*vao.shape.n_segments, GL_UNSIGNED_INT, 0, nodes.n_size);
    // <-


    // -> draw springs
    glVertexAttribDivisor(1, 0);
    glUniform1i(mass_spring_program.uniform("mode"), 1);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vao.vbo_springs_id);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 2*springs.n_size*sizeof(GLuint), vao.s_ab.data(), GL_DYNAMIC_DRAW);
    glDrawElements(GL_LINES, 2*springs.n_size, GL_UNSIGNED_INT, 0);
    // <-
    
};

void MSNWorld::gather_for_rendering() {

    for (int i = 0; i < N_CLOTH; i++) {
        for (int j = 0; j < N_CLOTH; j++) {
            int k = i*N_CLOTH + j;
            vao.p_xyz[k*3 + 0] = (float)nodes.px[k];
            vao.p_xyz[k*3 + 1] = (float)nodes.py[k];
            vao.p_xyz[k*3 + 2] = (float)nodes.pz[k];
            vao.color[k*3 + 0] = i%2 ? 0.2 : 0.9;
            vao.color[k*3 + 1] = i%2 ? 0.2 : 0.9;
            vao.color[k*3 + 2] = 0.9;
        }

    }

   for (int j = 0; j < springs.n_size; j++) {
        int pi0 = J.outerIndexPtr()[j];
        int a = J.innerIndexPtr()[pi0];
        int b = J.innerIndexPtr()[pi0 + 1];
        vao.s_ab[j*2] = a;
        vao.s_ab[j*2 + 1] = b;
    } 
};


void MSNWorld::advance(std::size_t &iteration_counter, long long int ms_per_frame) {
    if (is_floating) {
        // double t = iteration_counter/100.0;
        glm::vec3 axis(0, 1.0f, 0);
        glm::mat4 rot = glm::rotate(ms_per_frame/1000.0f, axis);
        vao.model_matrix = rot*vao.model_matrix;
    }
    
    double h2 = dt*dt;

    // -> fixed constraints
    nodes.px[0] = px_tmp[0];
    nodes.py[0] = py_tmp[0];
    nodes.pz[0] = 0.5*std::sin(iteration_counter*1e-2);
    nodes.px[N_CLOTH - 1] = px_tmp[1];
    nodes.py[N_CLOTH - 1] = py_tmp[1];
    nodes.pz[N_CLOTH - 1] = 0.5*std::sin(iteration_counter*1e-2);
    // <-

    // std::generate(&fx_langevin[0], &fx_langevin[0] + nodes.n_size, [&](){return 0.01*dist_normal(mt);});
    // std::generate(&fy_langevin[0], &fy_langevin[0] + nodes.n_size, [&](){return 0.01*dist_normal(mt);});
    // std::generate(&fz_langevin[0], &fz_langevin[0] + nodes.n_size, [&](){return 0.01*dist_normal(mt);});

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
    if (0) {

        for (int i = 0; i < 5; i++) {
            springs.dx_rhs.block(0, 0, springs.n_size, 1) = (nodes.px.block(0, 0, nodes.n_size, 1).transpose()*K.block(0, 0, springs.n_size, springs.n_size)*J.block(0, 0, nodes.n_size, springs.n_size)).transpose();
            springs.dy_rhs.block(0, 0, springs.n_size, 1) = (nodes.py.block(0, 0, nodes.n_size, 1).transpose()*K.block(0, 0, springs.n_size, springs.n_size)*J.block(0, 0, nodes.n_size, springs.n_size)).transpose();
            springs.dz_rhs.block(0, 0, springs.n_size, 1) = (nodes.pz.block(0, 0, nodes.n_size, 1).transpose()*K.block(0, 0, springs.n_size, springs.n_size)*J.block(0, 0, nodes.n_size, springs.n_size)).transpose();

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

            // nodes.pz[0] = 0.2*std::sin(iteration_counter*5e-3);
            // nodes.pz[N_CLOTH - 1] = 0.2*std::sin(iteration_counter*5e-3);    
        }

    } else {
        
        for (int l = 0; l < 10; l++) {
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
            // H.leftCols(3*nodes.n_size) = H.leftCols(3*nodes.n_size);
            
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
    }

    Timer::stop();

    nodes.vx.block(0, 0, nodes.n_size, 1) = (nodes.px.block(0, 0, nodes.n_size, 1) - nodes.vx.block(0, 0, nodes.n_size, 1))/dt;
    nodes.vy.block(0, 0, nodes.n_size, 1) = (nodes.py.block(0, 0, nodes.n_size, 1) - nodes.vy.block(0, 0, nodes.n_size, 1))/dt;
    nodes.vz.block(0, 0, nodes.n_size, 1) = (nodes.pz.block(0, 0, nodes.n_size, 1) - nodes.vz.block(0, 0, nodes.n_size, 1))/dt;

    if (iteration_counter % 60 == 0 || 1) {
        timer0 += Timer::get_timing();

        std::ofstream file;
        file.open (filename, std::ios::app);
        for (int i = 0; i < nodes.n_size; i++)
            file << nodes.px[i] << " " << nodes.py[i] << " " << nodes.pz[i] << std::endl;
        file << std::endl;
        // file << Timer::get_timing() << std::endl;
        file.close();
    }

    // J1.topRows(nodes.n_size) = J.leftCols(springs.n_size);    
    // gather_for_rendering();
}

void MSNWorld::create_springs(int i_node) {

}

void MSNWorld::spawn_nodes(float px, float py) {

}

void MSNWorld::delete_nodes(float px, float py) {

}

void MSNWorld::mouse_button_callback(GLFWwindow *window, int button, int action, int mods) {

}


void MSNWorld::keyboard_callback(GLFWwindow *window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_A && action == GLFW_PRESS) {
        get_instance().spawn_nodes(0, 0);
    } else if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
        get_instance().is_floating = !get_instance().is_floating;
    } else if (key == GLFW_KEY_DOWN && action == GLFW_PRESS) {
        get_instance().zoom -= 2e-2;
        get_instance().vao.model_matrix = glm::scale(glm::mat4(1), glm::vec3(get_instance().zoom));
    } else if (key == GLFW_KEY_UP && action == GLFW_PRESS) {
        get_instance().zoom += 2e-2;
        get_instance().vao.model_matrix = glm::scale(glm::mat4(1), glm::vec3(get_instance().zoom));
    }
}
