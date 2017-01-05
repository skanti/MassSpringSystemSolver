#include <mkl.h>
#include "MSNWorld.h"
#include "Solver.h"
#include "Engine.h"
#include "MeshLoader.h"
#include <eigen3/Eigen/Sparse>
#include "MathKernels.h"
#include "Timer.h"

#define H1 1e-1

template<typename value_type_node>
void fix_some_nodes( Nodes<value_type_node> &nodes, std::vector<int> &fixed_nodes) {
    for (int i = 0; i < nodes.n_size; i++) {
        if (nodes.p_y[i] >= 0.8) {
            fixed_nodes.push_back(i);
        }
    }
}

MSNWorld::MSNWorld() : World() {
    load_mesh_ply2<float>("/home/amon/grive/development/MassSpringNetwork/data/canstick.ply2", nodes, springs);
    normalize_and_recenter_nodes<float>(nodes);
    springs.set_as_equilibrium(nodes.p_x, nodes.p_y, nodes.p_z);
    px_rhs.resize(nodes.n_size);
    py_rhs.resize(nodes.n_size);
    pz_rhs.resize(nodes.n_size);
    dx_rhs.resize(springs.n_size);
    dy_rhs.resize(springs.n_size);
    dz_rhs.resize(springs.n_size);
    d_rhs.resize(springs.n_size);
    M.resize(nodes.n_size, nodes.n_size);
    M.setIdentity();
    fix_some_nodes(nodes, fixed_nodes);
    for (int i : fixed_nodes) {
        M.valuePtr()[i] = 1e6;        
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

void MSNWorld::init() {
    if (!msn2d_world) msn2d_world = std::unique_ptr<MSNWorld>(new MSNWorld());
}

MSNWorld &MSNWorld::get_instance() {
    return *msn2d_world;
}


void MSNWorld::init_shape() {
    glm::mat4 trans_mat = glm::translate(glm::mat4(1), glm::vec3(0, 0, 0));
    vao.model_matrix = trans_mat * glm::scale(glm::mat4(1), glm::vec3(0.5));
    vao.shape = ga::Shape::make_sphere(0.01f);
}

void MSNWorld::init_shader() {
    std::string dir = "/home/amon/grive/development/GameFramework/src/glsl";
    ga::Visualization::load_shaders(mass_spring_program, dir, "MassSpringVS3D.glsl", "MassSpringFS3D.glsl", "", "", "");
}

void MSNWorld::init_instances() {
    vao.n_instances = nodes.n_size;
    vao.p_xyz.resize(3 * nodes.n_size);

    vao.n_springs = springs.n_size;
    vao.s_ab.resize(2 * springs.n_size);
}

void MSNWorld::init_drawable() {
    glUseProgram(mass_spring_program.id);
    glGenVertexArrays(1, &vao.vao_id);
    glBindVertexArray(vao.vao_id);

    glUniform1f(mass_spring_program.uniform("l_base"), vao.shape.l_base);

    glUniformMatrix4fv(mass_spring_program.uniform("ModelMatrix"), 1, GL_FALSE, &vao.model_matrix[0][0]);
    glUniformMatrix4fv(mass_spring_program.uniform("ViewMatrix"), 1, GL_FALSE, &ga::Visualization::view_window[0][0]);
    glUniformMatrix4fv(mass_spring_program.uniform("ProjectionMatrix"), 1, GL_FALSE, &ga::Visualization::projection_window[0][0]);
    float nodes_color[] = {0.2f, 0.2f, 0.8f, 0.8f};
    glUniform4fv(mass_spring_program.uniform("nodes_color"), 1, nodes_color);
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
    glGenBuffers(1, &vao.vbo_instance_id);
    glBindBuffer(GL_ARRAY_BUFFER, vao.vbo_instance_id);
    glBufferData(GL_ARRAY_BUFFER, 3*nodes.n_size*sizeof(GLfloat), vao.p_xyz.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    // <-

    // -> springs
    glGenBuffers(1, &vao.vbo_springs_id);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vao.vbo_springs_id);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 2 * springs.n_size * sizeof(GLuint), vao.s_ab.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    // <-

    // -> culling options
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    // <-
};

void MSNWorld::draw() {
    glUseProgram(mass_spring_program.id);
    glBindVertexArray(vao.vao_id);

    // -> draw springs
    glVertexAttribDivisor(1, 0);
    glUniform1i(mass_spring_program.uniform("mode"), 1);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vao.vbo_springs_id);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 2 * springs.n_size * sizeof(GLuint), vao.s_ab.data(), GL_DYNAMIC_DRAW);
    glDrawElements(GL_LINES, 2 * springs.n_size, GL_UNSIGNED_INT, 0);
    // <-

    // -> draw nodes
    glUniform1i(mass_spring_program.uniform("mode"), 0);
    glVertexAttribDivisor(1, 1);
    glUniformMatrix4fv(mass_spring_program.uniform("ModelMatrix"), 1, GL_FALSE, &vao.model_matrix[0][0]);

    glBindBuffer(GL_ARRAY_BUFFER, vao.vbo_instance_id);
    glBufferData(GL_ARRAY_BUFFER, 3*nodes.n_size*sizeof(GLfloat), vao.p_xyz.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vao.vbo_shape_index_id);
    glDrawElementsInstanced(GL_TRIANGLES, 3*vao.shape.n_segments, GL_UNSIGNED_INT, 0, nodes.n_size);
    // <-
};

void MSNWorld::gather_for_rendering() {

    for (int i = 0; i < nodes.n_size; i++) {
        vao.p_xyz[i*3 + 0] = static_cast<float>(nodes.p_x[i]);
        vao.p_xyz[i*3 + 1] = static_cast<float>(nodes.p_y[i]);
        vao.p_xyz[i*3 + 2] = static_cast<float>(nodes.p_z[i]);
    }
   for (int j = 0; j < springs.n_size; j++) {
        int pi0 = springs.A.outerIndexPtr()[j];
        int a = springs.A.innerIndexPtr()[pi0];
        int b = springs.A.innerIndexPtr()[pi0 + 1];
        vao.s_ab[j*2] = a;
        vao.s_ab[j*2+1] = b;
    } 
};


void MSNWorld::advance(std::size_t &iteration_counter, long long int ms_per_frame) {
    double t = iteration_counter/100.0;
    glm::vec3 axis(std::sin(t), std::cos(t), 0);
    glm::mat4 rot = glm::rotate(ms_per_frame/1000.0f, axis);
    vao.model_matrix = rot*vao.model_matrix;

    float h = (float)(H1);
    float h2 = (float)(H1*H1);
    Vector<float> fgravity =  Eigen::MatrixXf::Constant(nodes.n_size, 1, -0.01f);
    // Vector<float> fgravity =  Eigen::MatrixXf::Constant(nodes.n_size, 1, 0.0f);


    nodes.q_x = nodes.p_x + nodes.v_x*h*(1.0f - h*0.5f); 
    nodes.q_y = nodes.p_y + nodes.v_y*h*(1.0f - h*0.5f);
    nodes.q_z = nodes.p_z + nodes.v_z*h*(1.0f - h*0.5f);

    nodes.v_x = nodes.p_x;
    nodes.v_y = nodes.p_y;
    nodes.v_z = nodes.p_z;

    nodes.p_x = nodes.q_x; 
    nodes.p_y = nodes.q_y;
    nodes.p_z = nodes.q_z;

    for (int i = 0; i < 5; i++) {
        dx_rhs = nodes.p_x.transpose()*J;
        dy_rhs = nodes.p_y.transpose()*J;
        dz_rhs = nodes.p_z.transpose()*J;
        for (int j = 0; j < springs.n_size; j++) {
            d_rhs[j] = std::sqrt(dx_rhs[j]*dx_rhs[j] + dy_rhs[j]*dy_rhs[j] + dz_rhs[j]*dz_rhs[j]);
        }

        springs.dx = springs.rd.cwiseProduct(dx_rhs).cwiseQuotient(d_rhs);
        springs.dy = springs.rd.cwiseProduct(dy_rhs).cwiseQuotient(d_rhs);
        springs.dz = springs.rd.cwiseProduct(dz_rhs).cwiseQuotient(d_rhs);
        
        px_rhs = M*nodes.q_x + h2*(J*springs.dx);
        py_rhs = M*nodes.q_y + h2*(J*springs.dy + fgravity);
        pz_rhs = M*nodes.q_z + h2*(J*springs.dz);

        nodes.p_x = chol.solve(px_rhs);
        nodes.p_y = chol.solve(py_rhs);
        nodes.p_z = chol.solve(pz_rhs);
    
    }
    nodes.v_x = (nodes.p_x - nodes.v_x)/h;
    nodes.v_y = (nodes.p_y - nodes.v_y)/h;
    nodes.v_z = (nodes.p_z - nodes.v_z)/h;
    gather_for_rendering();
}


void MSNWorld::mouse_button_callback(GLFWwindow *window, int button, int action, int mods) {
    return;
}