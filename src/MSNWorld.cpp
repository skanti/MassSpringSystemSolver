#include "MSNWorld.h"
#include "Engine.h"
#include "MeshLoader.h"
#include <eigen3/Eigen/Sparse>
#include "MathKernels.h"
#include "Timer.h"
#include <iostream>
#include <fstream>

const double dt = 0.2;
#define N_MAX_NODES 5000
#define N_MAX_SPRINGS 10000
#define N_CLOTH 20

MSNWorld::MSNWorld() : World() {
    flag = 1;
    is_floating = 0;
    i_counter = 0;
    zoom = 0.7f;
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
    M.setIdentity();

    K.setIdentity();
    std::fill(&K.valuePtr()[0], &K.valuePtr()[0] + N_MAX_SPRINGS, 20);

    px_tmp << nodes.px[0], nodes.px[N_CLOTH - 1];
    py_tmp << nodes.py[0], nodes.py[N_CLOTH - 1];

    mt.seed(999);

    f_gravity = Eigen::MatrixXd::Constant(N_MAX_NODES, 1, -0.0005f);
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
    
    init_shader();
    init_shape();
    init_instances();
    init_drawable();
}

void MSNWorld::init() {
    if (!msn2d_world) msn2d_world = std::unique_ptr<MSNWorld>(new MSNWorld());
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


    for (int i = 0; i < 10; i++) {
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


    }

    
    nodes.px[0] = px_tmp[0];
    nodes.py[0] = py_tmp[0];
    nodes.pz[0] = 0.4*std::sin(iteration_counter*1e-2);
    nodes.px[N_CLOTH - 1] = px_tmp[1];
    nodes.py[N_CLOTH - 1] = py_tmp[1];
    nodes.pz[N_CLOTH - 1] = 0.4*std::sin(iteration_counter*1e-2);

    nodes.vx.block(0, 0, nodes.n_size, 1) = (nodes.px.block(0, 0, nodes.n_size, 1) - nodes.vx.block(0, 0, nodes.n_size, 1))/dt;
    nodes.vy.block(0, 0, nodes.n_size, 1) = (nodes.py.block(0, 0, nodes.n_size, 1) - nodes.vy.block(0, 0, nodes.n_size, 1))/dt;
    nodes.vz.block(0, 0, nodes.n_size, 1) = (nodes.pz.block(0, 0, nodes.n_size, 1) - nodes.vz.block(0, 0, nodes.n_size, 1))/dt;

    // J1.topRows(nodes.n_size) = J.leftCols(springs.n_size);    
    gather_for_rendering();
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
