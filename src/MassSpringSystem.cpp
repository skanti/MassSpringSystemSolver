#include "MassSpringSystem.h"
#include "Visualization.h"
#include "Solver.h"

const int n_springs_per_node = 8;

MassSpringSystem::MassSpringSystem() : nodes(30), springs(30 * 8) {
    load_file("/Users/amon/grive/development/Springs2D/data/mesh.msh");
    int i_spring = 0;
    for (auto &m : sm) {
        springs.push_back(m.first.first, m.first.second, 1.0, 0, i_spring++);
    }
    Springs::set_deq_by_given_state(nodes.p_x.data(), nodes.p_y.data(), springs.a.data(), springs.b.data(),
                                    springs.d_eq.data(), springs.n_size);
    /*
    for (int i = 0; i < springs.n_size; i++) {
        std::cout << springs.a[i] << " " << springs.b[i] << " " << springs.d_eq[i] << std::endl;
    }
    */
    init_shape();
    init_instances();
    init_drawable();
}

void MassSpringSystem::load_file(std::string file) {
    std::ifstream infile(file);
    std::string line;
    while (std::getline(infile, line)) {
        if (line == std::string("$Nodes")) {
            std::getline(infile, line);
            std::istringstream issn(line);
            int n_nodes;
            issn >> n_nodes;
            for (int i = 0; i < n_nodes; i++) {
                std::getline(infile, line);
                std::istringstream issf(line);
                int k;
                double x, y;
                // -> reading from .msh file: k = index, x = pos_x, y = pos_y
                issf >> k >> x >> y;
                nodes.push_back_free(x, y, 0, 0, 1.0, 1.0, k - 1);
            }
        } else if (line == std::string("$Elements")) {
            const int tag_phase_fix = 12;
            int i_spring = 0;
            std::getline(infile, line);
            std::istringstream issn(line);
            int n_springs;
            issn >> n_springs;
            for (int i = 0; i < n_springs; i++) {
                std::getline(infile, line);
                std::istringstream isse(line);
                int k, type;
                isse >> k >> type;
                if (type == 1) {
                    int phase, tag, ent, n1, n2;
                    isse >> phase >> tag >> ent >> n1 >> n2;
                    if (tag == tag_phase_fix) {
                        int nn1 = nodes.key[n1 - 1];
                        int nn2 = nodes.key[n2 - 1];
                        //nodes.transfer_free2fix(nn1);
                        //nodes.transfer_free2fix(nn2);
                    }
                }
                if (type == 2) {
                    int phase, tag, ent, n1, n2, n3;
                    isse >> phase >> tag >> ent >> n1 >> n2 >> n3;
                    int nn1 = nodes.key[n1 - 1];
                    int nn2 = nodes.key[n2 - 1];
                    int nn3 = nodes.key[n3 - 1];
                    sm.stack({nn1, nn2}, i_spring++);
                    sm.stack({nn1, nn3}, i_spring++);
                    sm.stack({nn2, nn3}, i_spring++);
                }
            }
        }
    }
}


void MassSpringSystem::init_shape() {
    vao.model_matrix = glm::scale(glm::mat4(1), glm::vec3(0.5, 0.5, 1.0));
    vao.shape = ga::Shape::make_circle(0.02);
}

void MassSpringSystem::init_instances() {
    vao.n_instances = nodes.n_size;
    vao.p_xy.resize(2 * nodes.n_size);

    vao.n_springs = springs.n_size;
    vao.s_ab.resize(2 * springs.n_size);
}


void MassSpringSystem::init_drawable() {
    glUseProgram(ga::Visualization::mass_spring_program.get_id());
    glGenVertexArrays(1, &vao.vao_id);
    glBindVertexArray(vao.vao_id);

    glUniformMatrix4fv(ga::Visualization::mass_spring_program.uniform("ModelMatrix"), 1, GL_FALSE,
                       &vao.model_matrix[0][0]);
    glUniformMatrix4fv(ga::Visualization::mass_spring_program.uniform("ViewMatrix"), 1, GL_FALSE,
                       &ga::Visualization::view_window[0][0]);
    glUniformMatrix4fv(ga::Visualization::mass_spring_program.uniform("ProjectionMatrix"), 1, GL_FALSE,
                       &ga::Visualization::projection_window[0][0]);
    std::vector<float> color{{0.2f, 0.2f, 0.8f, 0.8f}};
    glUniform4fv(ga::Visualization::mass_spring_program.uniform("color"), 1, color.data());

    //glUniform2fv(ga::Visualization::mass_spring_program.uniform("pos"), vao.n_instances, vao.p_xy.data());

    glGenBuffers(1, &vao.vbo_shape_id);
    glBindBuffer(GL_ARRAY_BUFFER, vao.vbo_shape_id);
    glBufferData(GL_ARRAY_BUFFER, 2 * vao.shape.n_vertices * sizeof(GLfloat), vao.shape.vertices.data(),
                 GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);

    glGenBuffers(1, &vao.vbo_instance_id);
    glBindBuffer(GL_ARRAY_BUFFER, vao.vbo_instance_id);
    glBufferData(GL_ARRAY_BUFFER, 2 * vao.n_instances * sizeof(GLfloat), vao.p_xy.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(1);
    glVertexAttribDivisor(1, 1);

    glGenBuffers(1, &vao.vbo_springs_id);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vao.vbo_springs_id);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 2 * vao.n_springs * sizeof(GLuint), vao.s_ab.data(), GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
};

void MassSpringSystem::draw() {
    glUseProgram(ga::Visualization::mass_spring_program.get_id());
    glBindVertexArray(vao.vao_id);

    // -> draw nodes
    glUniform1i(ga::Visualization::mass_spring_program.uniform("mode"), 0);
    glVertexAttribDivisor(1, 1);
    glBindBuffer(GL_ARRAY_BUFFER, vao.vbo_instance_id);
    glBufferData(GL_ARRAY_BUFFER, 2 * vao.n_instances * sizeof(GLfloat), vao.p_xy.data(), GL_DYNAMIC_DRAW);
    glDrawArraysInstanced(vao.shape.type_primitive, 0, vao.shape.n_vertices, vao.n_instances);
    glVertexAttribDivisor(1, 0);
    // <-

    // -> draw springs
    glUniform1i(ga::Visualization::mass_spring_program.uniform("mode"), 1);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vao.vbo_springs_id);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 2 * vao.n_springs * sizeof(GLuint), vao.s_ab.data(), GL_DYNAMIC_DRAW);
    glDrawElements(GL_LINES, 2 * vao.n_springs, GL_UNSIGNED_INT, 0);
    // <-
};

void MassSpringSystem::gather() {
    for (int i = 0; i < springs.n_size; i++) {
        vao.s_ab[i * 2] = static_cast<unsigned int>(springs.a[i]);
        vao.s_ab[i * 2 + 1] = static_cast<unsigned int>(springs.b[i]);
    }

    for (int i = 0; i < nodes.n_size; i++) {
        vao.p_xy[i * 2] = static_cast<float>(nodes.p_x[i]);
        vao.p_xy[i * 2 + 1] = static_cast<float>(nodes.p_y[i]);
    }
};

void MassSpringSystem::move() {
    double dt = 1e-3;
    for (int i = 0; i < nodes.n_size_free; i++) {
        nodes.f_x[i] = 0;
        nodes.f_y[i] = -9.81;
    }

    Solver::mss_force(nodes.p_x.data(), nodes.p_y.data(), nodes.f_x.data(), nodes.f_y.data(), nodes.n_size_fix,
                      springs.a.data(), springs.b.data(), springs.k.data(), springs.d_eq.data(), springs.n_size);

    Solver::euler_forward(nodes.p_x.data(), nodes.p_y.data(), nodes.v_x.data(), nodes.v_y.data(), nodes.f_x.data(),
                          nodes.f_y.data(), nodes.m.data(), dt, nodes.n_size_free);
    gather();
}