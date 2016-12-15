#include <mkl.h>
#include "MSN2DWorld.h"
#include "Solver.h"
#include "Engine.h"
#include "MeshLoader.h"
#include <eigen3/Eigen/Sparse>
#include "MathKernels.h"
#include "Timer.h"

#define H1 1e-2

MSN2DWorld::MSN2DWorld() : World() {
    
    load_mesh_ply2<float>("/home/amon/grive/development/MassSpringNetwork/data/mesh.ply2", nodes, springs);
    springs.set_as_equilibrium(nodes.p_x, nodes.p_y);
    px_rhs.resize(nodes.n_size);
    py_rhs.resize(nodes.n_size);
    dx_rhs.resize(springs.n_size);
    dy_rhs.resize(springs.n_size);
    d_rhs.resize(springs.n_size);
    M.resize(nodes.n_size, nodes.n_size);
    M.setIdentity();
    std::vector<int> fixed_nodes = {0,1,2, 3, 4};
    for (int i : fixed_nodes) {
        M.valuePtr()[i] = 100000;        
    }
    J = springs.A*springs.K;
    Q = M + (SparseMatrix<float>)((float)(H1*H1)*springs.A*springs.K*springs.A.transpose());
    //std::cout << Q << std::endl;
    //sp_mm1_mul(springs.A, springs.k, J);
    //sp_mm1_add(springs.AA, nodes.m, L, (float)(H1*H1));
    chol.compute(Q);
    init_shader();
    init_shape();
    init_instances();
    init_drawable();
}

void MSN2DWorld::init() {
    if (!msn2d_world) msn2d_world = std::unique_ptr<MSN2DWorld>(new MSN2DWorld());
}

MSN2DWorld &MSN2DWorld::get_instance() {
    return *msn2d_world;
}


void MSN2DWorld::init_shape() {
    glm::mat4 trans_mat = glm::translate(glm::mat4(1), glm::vec3(0, -0.3, 0));
    vao.model_matrix = trans_mat * glm::scale(glm::mat4(1), glm::vec3(0.3, 0.3, 1.0));
    vao.shape = ga::Shape::make_circle(0.03);
}

void MSN2DWorld::init_shader() {
    std::string dir = "/home/amon/grive/development/GameFramework/src/glsl";
    ga::Visualization::load_shaders(mass_spring_program, dir, "MassSpringVS.glsl", "MassSpringFS.glsl", "", "", "");
}

void MSN2DWorld::init_instances() {
    vao.n_instances = nodes.n_size;
    vao.p_xy.resize(2 * nodes.n_size);

    vao.n_springs = springs.n_size;
    vao.s_ab.resize(2 * springs.n_size);
}

void MSN2DWorld::init_drawable() {
    glUseProgram(mass_spring_program.id);
    glGenVertexArrays(1, &vao.vao_id);
    glBindVertexArray(vao.vao_id);

    glUniformMatrix4fv(mass_spring_program.uniform("ModelMatrix"), 1, GL_FALSE,
                       &vao.model_matrix[0][0]);
    glUniformMatrix4fv(mass_spring_program.uniform("ViewMatrix"), 1, GL_FALSE,
                       &ga::Visualization::view_window[0][0]);
    glUniformMatrix4fv(mass_spring_program.uniform("ProjectionMatrix"), 1, GL_FALSE,
                       &ga::Visualization::projection_window[0][0]);
    std::vector<float> color{{0.2f, 0.2f, 0.8f, 0.8f}};
    glUniform4fv(mass_spring_program.uniform("color"), 1, color.data());

    glGenBuffers(1, &vao.vbo_shape_id);
    glBindBuffer(GL_ARRAY_BUFFER, vao.vbo_shape_id);
    glBufferData(GL_ARRAY_BUFFER, 2 * vao.shape.n_vertices * sizeof(GLfloat), vao.shape.vertices.data(),
                 GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);

    glGenBuffers(1, &vao.vbo_instance_id);
    glBindBuffer(GL_ARRAY_BUFFER, vao.vbo_instance_id);
    glBufferData(GL_ARRAY_BUFFER, 2 * nodes.n_size * sizeof(GLfloat), vao.p_xy.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(1);
    glVertexAttribDivisor(1, 1);

    glGenBuffers(1, &vao.vbo_springs_id);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vao.vbo_springs_id);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 2 * springs.n_size * sizeof(GLuint), vao.s_ab.data(), GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
};

void MSN2DWorld::draw() {
    glUseProgram(mass_spring_program.id);
    glBindVertexArray(vao.vao_id);

    
    // -> draw springs
    glUniform1i(mass_spring_program.uniform("mode"), 1);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vao.vbo_springs_id);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 2 * springs.n_size * sizeof(GLuint), vao.s_ab.data(), GL_DYNAMIC_DRAW);
    glDrawElements(GL_LINES, 2 * springs.n_size, GL_UNSIGNED_INT, 0);
    // <-
    

    // -> draw nodes
    glUniform1i(mass_spring_program.uniform("mode"), 0);
    glVertexAttribDivisor(1, 1);
    glBindBuffer(GL_ARRAY_BUFFER, vao.vbo_instance_id);
    glBufferData(GL_ARRAY_BUFFER, 2 * nodes.n_size * sizeof(GLfloat), vao.p_xy.data(), GL_DYNAMIC_DRAW);
    glDrawArraysInstanced(vao.shape.type_primitive, 0, vao.shape.n_vertices, nodes.n_size);
    glVertexAttribDivisor(1, 0);
    // <-

};

void MSN2DWorld::gather_for_rendering() {

    for (int i = 0; i < nodes.n_size; i++) {
        vao.p_xy[i * 2] = static_cast<float>(nodes.p_x[i]);
        vao.p_xy[i * 2 + 1] = static_cast<float>(nodes.p_y[i]);
    }
   for (int j = 0; j < springs.n_size; j++) {
        int pi0 = springs.A.outerIndexPtr()[j];
        int a = springs.A.innerIndexPtr()[pi0];
        int b = springs.A.innerIndexPtr()[pi0 + 1];
        vao.s_ab[j*2] = a;
        vao.s_ab[j*2+1] = b;
    } 
};


void MSN2DWorld::advance(std::size_t &iteration_counter, long long int ms_per_frame) {
    float h = (float)(H1);
    float h2 = (float)(H1*H1);
    Vector<float> fgravity =  Eigen::MatrixXf::Constant(nodes.n_size, 1, -0.5f);


    Vector<float> Y_x = nodes.p_x + nodes.v_x*h*(1.0f - h*0.5f); 
    Vector<float> Y_y = nodes.p_y + nodes.v_y*h*(1.0f - h*0.5f);

    nodes.v_x = nodes.p_x;
    nodes.v_y = nodes.p_y;

    nodes.p_x = Y_x; 
    nodes.p_y = Y_y;

    for (int i = 0; i < 10; i++) {

        dx_rhs = nodes.p_x.transpose()*J;
        dy_rhs = nodes.p_y.transpose()*J;
        for (int j = 0; j < springs.n_size; j++) {
            d_rhs[j] = std::sqrt(dx_rhs[j]*dx_rhs[j] + dy_rhs[j]*dy_rhs[j]);
        }

        springs.dx = springs.rd.cwiseProduct(dx_rhs).cwiseQuotient(d_rhs);
        springs.dy = springs.rd.cwiseProduct(dy_rhs).cwiseQuotient(d_rhs);
        
        px_rhs = M*Y_x + h2*(J*springs.dx);
        py_rhs = M*Y_y + h2*(J*springs.dy + fgravity);

        nodes.p_x = chol.solve(px_rhs);
        nodes.p_y = chol.solve(py_rhs);
    
        // std::cout << nodes.p_x << std::endl;
        // exit(0);
    }
    nodes.v_x = (nodes.p_x - nodes.v_x)/h;
    nodes.v_y = (nodes.p_y - nodes.v_y)/h;
    // nodes.p_x = nodes.p_x;
    // nodes.p_y = nodes.p_y;
    gather_for_rendering();
}


void MSN2DWorld::mouse_button_callback(GLFWwindow *window, int button, int action, int mods) {
    return;
}