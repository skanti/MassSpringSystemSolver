#include "Common.h"
#include "MSNWorld.h"
#include "gf/Visualization.h"
#include "MeshLoader.h"
#include "MathKernels.h"
#include "Timer.h"
#include <iostream>
#include <fstream>
 #include <unsupported/Eigen/KroneckerProduct>

const double dt = 0.2;
#define N_MAX_NODES 1000
#define N_MAX_SPRINGS 1000
#define N_CLOTH 10

void MSNWorld::init() {
    flag = 1;
    is_floating = 0;
    i_counter = 0;
    zoom = 0.7f;
    // -> reserve memory
    nodes.reserve(N_MAX_NODES);
    springs.reserve(N_MAX_SPRINGS);
    A.reserve(2*N_MAX_SPRINGS);
    A.resize(N_MAX_NODES, N_MAX_SPRINGS);
    J.reserve(3*2*N_MAX_SPRINGS);
    J.resize(3*N_MAX_NODES, 3*N_MAX_SPRINGS);
    J1.reserve(3*2*N_MAX_SPRINGS);
    J1.resize(3*N_MAX_NODES, 3*N_MAX_SPRINGS);
    int h1 = -2;
    std::generate(&J.outerIndexPtr()[0], &J.outerIndexPtr()[0] + 3*N_MAX_SPRINGS + 1, [&h1]{return h1 += 2;});
    Q.reserve(3*(N_MAX_NODES + 2*N_MAX_SPRINGS));
    Q.resize(3*N_MAX_NODES, 3*N_MAX_NODES);
    M.reserve(3*N_MAX_NODES);
    M.resize(3*N_MAX_NODES, 3*N_MAX_NODES);
    K.reserve(3*N_MAX_SPRINGS);
    K.resize(3*N_MAX_SPRINGS, 3*N_MAX_SPRINGS);
    // <-

    // -> load and set mesh
    //load_mesh_ply2<float>("/dtome/amon/grive/development/MassSpringNetwork/data/canstick.ply2", nodes, springs);
    create_cloth<double>(nodes, springs, A, N_CLOTH);
    normalize_and_recenter_nodes<double>(nodes);
    springs.set_as_equilibrium(nodes.p, A);
    // <-

    M.setIdentity();

    K.setIdentity();
    std::fill(&K.valuePtr()[0], &K.valuePtr()[0] + 3*N_MAX_SPRINGS, 20);

    p0_tmp = nodes.p.col(0);
    p1_tmp = nodes.p.col(N_CLOTH - 1);

    mt.seed(999);

    // f_gravity = Eigen::MatrixXd::Constant(3, N_MAX_NODES, -0.0005f);
    f_gravity = Matrix<double>::Constant(3, N_MAX_NODES, 0);
    for (int i = 0; i < N_MAX_NODES; i++) {
        f_gravity.col(i) = Vector3<double>(0, -0.0005, 0);
    }

    f_langevin = Matrix<double>::Constant(3, N_MAX_NODES, 0);

    J.leftCols(3*springs.n_size) = (SparseMatrix<double>)(Eigen::kroneckerProduct(A.leftCols(springs.n_size), Eigen::Matrix3d::Identity())).leftCols(3*springs.n_size);
    Q.leftCols(3*nodes.n_size) = M.block(0, 0, 3*nodes.n_size, 3*nodes.n_size) 
    + (SparseMatrix<double>)(dt*dt*J.block(0, 0, 3*nodes.n_size, 3*springs.n_size)*K.block(0, 0, 3*springs.n_size, 3*springs.n_size)*J.block(0, 0, 3*nodes.n_size, 3*springs.n_size).transpose());

    chol.compute(Q.block(0,0, 3*nodes.n_size, 3*nodes.n_size));

    
    init_shader();
    init_shape();
    init_instances();
    init_drawable();
}

void MSNWorld::term() {

}

void MSNWorld::init_shape() {
    glm::mat4 trans_mat = glm::translate(glm::mat4(1), glm::vec3(0, 0, 0));
    vao.model_matrix = trans_mat*glm::scale(glm::mat4(1), glm::vec3(zoom));
    vao.shape = gf::Shape::make_sphere(0.01f);
}

void MSNWorld::init_shader() {
    std::string dir = std::string(MSN_SRC_PATH) + "/glsl/";
	std::cout << dir << std::endl;
    gf::Visualization::load_shaders_and_link_to_program(mass_spring_program, dir + "MassSpringVS3D.glsl", dir + "MassSpringFS3D.glsl", "", "", "");
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
    glUniformMatrix4fv(mass_spring_program.uniform("ViewMatrix"), 1, GL_FALSE, &gf::Visualization::view_window[0][0]);
    glUniformMatrix4fv(mass_spring_program.uniform("ProjectionMatrix"), 1, GL_FALSE, &gf::Visualization::projection_window[0][0]);
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
            vao.p_xyz[k*3 + 0] = (float)nodes.p(0, k);
            vao.p_xyz[k*3 + 1] = (float)nodes.p(1, k);
            vao.p_xyz[k*3 + 2] = (float)nodes.p(2, k);
            vao.color[k*3 + 0] = i%2 ? 0.2 : 0.9;
            vao.color[k*3 + 1] = i%2 ? 0.2 : 0.9;
            vao.color[k*3 + 2] = 0.9;
        }

    }

   for (int j = 0; j < springs.n_size; j++) {
        int pi0 = A.outerIndexPtr()[j];
        int a = A.innerIndexPtr()[pi0];
        int b = A.innerIndexPtr()[pi0 + 1];
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
    
    // std::generate(&f_langevin[0], &f_langevin[0] + 3*nodes.n_size, [&](){return 0.01*dist_normal(mt);});

    nodes.q.block(0, 0, 3, nodes.n_size) = nodes.p.block(0, 0, 3, nodes.n_size) + nodes.v.block(0, 0, 3, nodes.n_size)*dt*(1.0 - dt*0.1); 

    nodes.v.block(0, 0, 3, nodes.n_size) = nodes.p.block(0, 0, 3, nodes.n_size);
    
    nodes.p.block(0, 0, 3, nodes.n_size) = nodes.q.block(0, 0, 3, nodes.n_size); 


    for (int i = 0; i < 10; i++) {
        Matrix2Vector(springs.d_rhs, 3*springs.n_size) = (Matrix2Vector(nodes.p, 3*nodes.n_size).block(0, 0, 3*nodes.n_size, 1).transpose()*J.block(0, 0, 3*nodes.n_size, 3*springs.n_size)
            *K.block(0, 0, 3*springs.n_size, 3*springs.n_size)).transpose();


        springs.d0_rhs.block(0, 0, springs.n_size, 1) = springs.d_rhs.block(0, 0, 3, springs.n_size).colwise().norm().transpose();
        springs.d.block(0, 0, 3, springs.n_size) = springs.d_rhs.block(0, 0, 3, springs.n_size).colwise().normalized()*springs.d0.block(0, 0, springs.n_size, 1).asDiagonal();

        Matrix2Vector(nodes.p_rhs, 3*nodes.n_size) = M.block(0, 0, 3*nodes.n_size, 3*nodes.n_size)*Matrix2Vector(nodes.q, 3*nodes.n_size) + dt*dt*(J.block(0, 0, 3*nodes.n_size, 3*springs.n_size)
            *K.block(0, 0, 3*springs.n_size, 3*springs.n_size)*Matrix2Vector(springs.d, 3*springs.n_size) + Matrix2Vector(f_langevin, 3*nodes.n_size) + Matrix2Vector(f_gravity, 3*nodes.n_size));
  
        Matrix2Vector(nodes.p, 3*nodes.n_size) = chol.solve(Matrix2Vector(nodes.p_rhs, 3*nodes.n_size));
    }

    nodes.p.col(0) = p0_tmp;
    nodes.p(2, 0) = 0.4*std::sin(iteration_counter*1e-2);
    nodes.p.col(N_CLOTH - 1) = p1_tmp;
    nodes.p(2, N_CLOTH - 1) = 0.4*std::sin(iteration_counter*1e-2);

    nodes.v.block(0, 0, 3, nodes.n_size) = (nodes.p.block(0, 0, 3, nodes.n_size) - nodes.v.block(0, 0, 3, nodes.n_size))/dt;

    // J1.topRows(nodes.n_size) = J.leftCols(springs.n_size);    
    gather_for_rendering();
}

void MSNWorld::create_springs(int i_node) {

}

void MSNWorld::spawn_nodes(float px, float py) {

}

void MSNWorld::delete_nodes(float px, float py) {

}

void MSNWorld::keyboard_callback(GLFWwindow *window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_A && action == GLFW_PRESS) {
        spawn_nodes(0, 0);
    } else if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
        is_floating = !is_floating;
    } else if (key == GLFW_KEY_DOWN && action == GLFW_PRESS) {
        zoom -= 2e-2;
        vao.model_matrix = glm::scale(glm::mat4(1), glm::vec3(zoom));
    } else if (key == GLFW_KEY_UP && action == GLFW_PRESS) {
        zoom += 2e-2;
        vao.model_matrix = glm::scale(glm::mat4(1), glm::vec3(zoom));
    }
}
