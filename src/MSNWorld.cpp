#include <mkl.h>
#include "MSNWorld.h"
#include "Engine.h"
#include "MeshLoader.h"
#include <eigen3/Eigen/Sparse>
#include "MathKernels.h"
#include "Timer.h"

const double dt = 1.0/60.0;
#define N_MAX_NODES 500
#define N_MAX_SPRINGS 500
#define N_PT 3
#define N_PITCH 2

MSNWorld::MSNWorld() : World() {
    flag = 1;
    is_floating = 0;
    zoom = 0.7f;
    // -> reserve memory
    nodes.reserve(N_MAX_NODES);
    springs.reserve(N_MAX_SPRINGS);
    T0.resize(N_PT);
    T2.resize(N_PT + 1);
    S0.resize(N_MAX_NODES);
    V0.resize(N_MAX_NODES);
    T1.resize(N_PT + N_PITCH);
    // S1.resize(13 + 3);
    A.reserve(2*N_MAX_SPRINGS);
    A.resize(N_MAX_NODES, N_MAX_SPRINGS);
    J.reserve(2*N_MAX_SPRINGS);
    J.resize(N_MAX_NODES, N_MAX_SPRINGS);
    J1.reserve(2*N_MAX_SPRINGS);
    J1.resize(N_MAX_NODES, N_MAX_SPRINGS);
    int h1 = -2;
    std::generate(&J.outerIndexPtr()[0], &J.outerIndexPtr()[0] + N_MAX_SPRINGS + 1, [&h1]{h1 += 2; return h1;});
    Q.reserve(N_MAX_NODES + 2*N_MAX_SPRINGS);
    Q.resize(N_MAX_NODES, N_MAX_NODES);
    M.reserve(N_MAX_NODES);
    M.resize(N_MAX_NODES, N_MAX_NODES);
    // <-

    // -> load and set mesh
    //load_mesh_ply2<float>("/dtome/amon/grive/development/MassSpringNetwork/data/canstick.ply2", nodes, springs);
    create_microtubule<double>(nodes, springs, A, T0, T1, S0, V0, N_PT, N_PITCH, 2);
    normalize_and_recenter_nodes<double>(nodes);
    springs.set_as_equilibrium(nodes.px, nodes.py, nodes.pz, A);
    T2[0] = 0;
    std::partial_sum(&T0[0], &T0[0] + N_PT, &T2[1], std::plus<int>());
    // <-

    M.setIdentity();
    for (int i = 0, j0 = 0; i < N_PT; i++, j0 += T0[i]) {
        for (int j = 0; j < N_PITCH; j++) {
        int k = j + j0;
        M.valuePtr()[k] = 1e6;
        }
    }

    mt.seed(999);
    dist_int_uniform = std::uniform_int_distribution<int>(0, N_PT - 1);

    f_gravity = Eigen::MatrixXd::Constant(N_MAX_NODES, 1, 0.01f);
    fx_langevin = Eigen::MatrixXd::Constant(N_MAX_NODES, 1, 0);
    fy_langevin = Eigen::MatrixXd::Constant(N_MAX_NODES, 1, 0);
    fz_langevin = Eigen::MatrixXd::Constant(N_MAX_NODES, 1, 0);

    J.leftCols(springs.n_size) = A.leftCols(springs.n_size);
    Q.leftCols(nodes.n_size) = M.block(0, 0, nodes.n_size, nodes.n_size) + (SparseMatrix<double>)(dt*dt*J.block(0, 0, nodes.n_size, springs.n_size)*J.block(0, 0, nodes.n_size, springs.n_size).transpose());
    chol.compute(Q.block(0,0, nodes.n_size, nodes.n_size));

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
    vao.shape = ga::Shape::make_sphere(0.02f);
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
    std::vector<int> t(N_PT, 0);

    for (int i = 0, j0 = 0; i < N_PT; j0 += T0[i], i++) {
        for (int j = 0; j < T0[i]; j++) {
            int k = j + j0;
            vao.p_xyz[k*3 + 0] = (float)nodes.px[k];
            vao.p_xyz[k*3 + 1] = (float)nodes.py[k];
            vao.p_xyz[k*3 + 2] = (float)nodes.pz[k];
            vao.color[k*3 + 0] = t[V0[k]] % 2 ? 0.2 : 0.9;
            vao.color[k*3 + 1] = t[V0[k]] % 2 ? 0.2 : 0.9;
            vao.color[k*3 + 2] = 0.9;
            t[V0[k]]++;
        }

    }



   for (int j = 0; j < springs.n_size; j++) {
        int pi0 = J.outerIndexPtr()[j];
        int a = J.innerIndexPtr()[pi0];
        int b = J.innerIndexPtr()[pi0 + 1];
        vao.s_ab[j*2] = a;
        vao.s_ab[j*2+1] = b;
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
        springs.dx_rhs.block(0, 0, springs.n_size, 1) = (nodes.px.block(0, 0, nodes.n_size, 1).transpose()*J.block(0, 0, nodes.n_size, springs.n_size)).transpose();
        springs.dy_rhs.block(0, 0, springs.n_size, 1) = (nodes.py.block(0, 0, nodes.n_size, 1).transpose()*J.block(0, 0, nodes.n_size, springs.n_size)).transpose();
        springs.dz_rhs.block(0, 0, springs.n_size, 1) = (nodes.pz.block(0, 0, nodes.n_size, 1).transpose()*J.block(0, 0, nodes.n_size, springs.n_size)).transpose();

        for (int j = 0; j < springs.n_size; j++) {
            springs.d_rhs[j] = std::sqrt(springs.dx_rhs[j]*springs.dx_rhs[j] + springs.dy_rhs[j]*springs.dy_rhs[j] + springs.dz_rhs[j]*springs.dz_rhs[j]);
            springs.dx[j] = springs.d[j]*springs.dx_rhs[j]/springs.d_rhs[j];
            springs.dy[j] = springs.d[j]*springs.dy_rhs[j]/springs.d_rhs[j];
            springs.dz[j] = springs.d[j]*springs.dz_rhs[j]/springs.d_rhs[j];
        }

        nodes.px_rhs.block(0, 0, nodes.n_size, 1) = M.block(0, 0, nodes.n_size, nodes.n_size)*nodes.qx.block(0, 0, nodes.n_size, 1) + h2*(J.block(0, 0, nodes.n_size, springs.n_size)*springs.dx.block(0, 0, springs.n_size, 1) 
            + fx_langevin.block(0, 0, nodes.n_size, 1));
        nodes.py_rhs.block(0, 0, nodes.n_size, 1) = M.block(0, 0, nodes.n_size, nodes.n_size)*nodes.qy.block(0, 0, nodes.n_size, 1) + h2*(J.block(0, 0, nodes.n_size, springs.n_size)*springs.dy.block(0, 0, springs.n_size, 1) 
            + fy_langevin.block(0, 0, nodes.n_size, 1));
        nodes.pz_rhs.block(0, 0, nodes.n_size, 1) = M.block(0, 0, nodes.n_size, nodes.n_size)*nodes.qz.block(0, 0, nodes.n_size, 1) + h2*(J.block(0, 0, nodes.n_size, springs.n_size)*springs.dz.block(0, 0, springs.n_size, 1)
            + fz_langevin.block(0, 0, nodes.n_size, 1));


        nodes.px.block(0, 0, nodes.n_size, 1) = chol.solve(nodes.px_rhs.block(0, 0, nodes.n_size, 1));
        nodes.py.block(0, 0, nodes.n_size, 1) = chol.solve(nodes.py_rhs.block(0, 0, nodes.n_size, 1));
        nodes.pz.block(0, 0, nodes.n_size, 1) = chol.solve(nodes.pz_rhs.block(0, 0, nodes.n_size, 1));
    
    }
    nodes.vx.block(0, 0, nodes.n_size, 1) = (nodes.px.block(0, 0, nodes.n_size, 1) - nodes.vx.block(0, 0, nodes.n_size, 1))/dt;
    nodes.vy.block(0, 0, nodes.n_size, 1) = (nodes.py.block(0, 0, nodes.n_size, 1) - nodes.vy.block(0, 0, nodes.n_size, 1))/dt;
    nodes.vz.block(0, 0, nodes.n_size, 1) = (nodes.pz.block(0, 0, nodes.n_size, 1) - nodes.vz.block(0, 0, nodes.n_size, 1))/dt;

    J1.topRows(nodes.n_size) = J.leftCols(springs.n_size);
    
    gather_for_rendering();
}

void MSNWorld::spawn_nodes(float px, float py) {
    if (nodes.n_size < N_MAX_NODES && springs.n_size < N_MAX_SPRINGS) {
        int i_protofilament = dist_int_uniform(mt);

        // std::cout << "n_size " <<  nodes.n_size << std::endl;
        // std::cout << "i_pf " <<  i_protofilament << std::endl;


        int j = nodes.n_size - 1;
        for (int i = N_PT - 1; i > i_protofilament; i--) {
            j -= T0[i];
        }
        // std::cout << "j " << j << std::endl;
        double px0 = nodes.px[j];
        double py0 = nodes.py[j] + 0.1;
        double pz0 = nodes.pz[j];

        nodes.set(nodes.n_size, px0, py0, pz0, 0, 0, 0, 1.0f);
        S0[nodes.n_size] = nodes.n_size;
        V0[nodes.n_size] = i_protofilament;
        // std::cout << V0.block(0, 0, nodes.n_size + 1, 1) << std::endl;

        // std::cout << S0.block(0, 0, nodes.n_size, 1) << std::endl;
        nodes.insert(i_protofilament, N_PT, T0, S0);
        nodes.n_size++;
        T0[i_protofilament]++;

        std::partial_sum(&T0[0], &T0[0] + N_PT, &T2[1], std::plus<int>());

        // J.coeffRef(nodes.n_size - 2, springs.n_size-1) = -1;
        // J.coeffRef(nodes.n_size - 1, springs.n_size-1) = 1;
        int ia = J.outerIndexPtr()[springs.n_size];
        J.innerIndexPtr()[ia] = j;
        J.innerIndexPtr()[ia+1] = nodes.n_size - 1;
        J.valuePtr()[ia] = -1;
        J.valuePtr()[ia+1] = 1;
        springs.key[springs.n_size] = springs.n_size;

        springs.n_size++;
        
        springs.set_as_equilibrium1(nodes.px, nodes.py, nodes.pz, J, springs.n_size-1);
        Q.leftCols(nodes.n_size) = M.block(0, 0, nodes.n_size, nodes.n_size) + (SparseMatrix<double>)(dt*dt*J.block(0, 0, nodes.n_size, springs.n_size)*J.block(0, 0, nodes.n_size, springs.n_size).transpose());
        chol.compute(Q.block(0,0, nodes.n_size, nodes.n_size));

        // Eigen::IOFormat CleanFmt(2, 0, ", ", "\n", "[", "]");
        // Eigen::Matrix<float, N_MAX_NODES, N_MAX_NODES> Q1 = Q;
        // Eigen::Matrix<float, N_MAX_NODES, N_MAX_SPRINGS> J2 = J1;
        // std::cout << J2.format(CleanFmt) << std::endl << std::endl;
        // std::cout << Q1.format(CleanFmt) << std::endl << std::endl;

    }
}

void MSNWorld::delete_nodes(float px, float py) {
        
        nodes.n_size--;

        for (int i = J1.outerIndexPtr()[nodes.n_size]; i < J1.outerIndexPtr()[nodes.n_size + 1]; i++) {
            springs.n_size--;
            int a = springs.key[J1.innerIndexPtr()[i]];
            int b = springs.n_size;
            springs.swap(a, b, J);
        }

        // std::iota(&springs.key[0], &springs.key[0] + springs.n_size, 0);
        Q.leftCols(nodes.n_size) = M.block(0, 0, nodes.n_size, nodes.n_size) + (SparseMatrix<double>)(dt*dt*J.block(0, 0, nodes.n_size, springs.n_size)*J.block(0, 0, nodes.n_size, springs.n_size).transpose());
        chol.compute(Q.block(0,0, nodes.n_size, nodes.n_size));

        // Eigen::IOFormat CleanFmt(2, 0, ", ", "\n", "[", "]");
        // Eigen::Matrix<float, N_MAX_NODES, N_MAX_SPRINGS> J2 = J1;
        // std::cout << J2.format(CleanFmt) << std::endl << std::endl;
        // Eigen::Matrix<float, N_MAX_NODES, N_MAX_NODES> Q1 = Q;
        // std::cout << Q1.format(CleanFmt) << std::endl << std::endl;

}
void MSNWorld::mouse_button_callback(GLFWwindow *window, int button, int action, int mods) {
    // if (action == GLFW_PRESS) {
    //     double px_target, py_target;
    //     ga::Engine::get_instance().get_cursor_position(&px_target, &py_target);
    //     px_target = px_target*2.0f / ga::WindowManager::width_window - 1.0f;
    //     py_target = (-py_target*2.0f / ga::WindowManager::height_window + 1.0f)*ga::WindowManager::aspect_ratio_inv_window;
    //     glm::mat4 z = glm::scale(glm::mat4(1), glm::vec3(get_instance().zoom));
    //     glm::mat4 t = glm::translate(glm::mat4(1), glm::vec3(0, 0, 0));
    //     glm::vec4 p = glm::inverse(z)*t*glm::vec4(px_target, py_target, 0, 1);
    //     if (button == GLFW_MOUSE_BUTTON_LEFT) {
    //         get_instance().spawn_nodes(p[0], p[1]);
    //     } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
    //         get_instance().delete_nodes(p[0], p[1]);
    //     }
    // }
}


void MSNWorld::keyboard_callback(GLFWwindow *window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_A && action == GLFW_PRESS) {
        get_instance().spawn_nodes(0, 0);
    } else if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
        get_instance().is_floating = !get_instance().is_floating;
    }
}
