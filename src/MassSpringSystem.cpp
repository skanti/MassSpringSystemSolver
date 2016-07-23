#include "MassSpringSystem.h"
#include "Visualization.h"
#include "Solver.h"

const int n_springs_per_node = 8;

MassSpringSystem::MassSpringSystem() : nodes(30), springs(30 * 8) {
    load_file("/Users/amon/grive/development/Spring2D/data/mesh.msh");
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
                nodes.push_back_free(x, y, 0, 0, 1.0, 1.0);
            }
        } else if (line == std::string("$Elements")) {
            const int tag_phase_fix = 12;
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
                        //nodes.transfer_free2fix(nn1);
                        int nn2 = nodes.key[n2 - 1];
                        //nodes.transfer_free2fix(nn2);
                    }
                }
                if (type == 2) {
                    int phase, tag, ent, n1, n2, n3;
                    isse >> phase >> tag >> ent >> n1 >> n2 >> n3;
                    int nn1 = nodes.key[n1 - 1];
                    int nn2 = nodes.key[n2 - 1];
                    int nn3 = nodes.key[n3 - 1];
                    springs.push_back_sym_if_unique(nn1, nn2, 50.0, 0);
                    springs.push_back_sym_if_unique(nn1, nn3, 50.0, 0);
                    springs.push_back_sym_if_unique(nn2, nn3, 50.0, 0);
                }
            }
        }
    }
}

void MassSpringSystem::init_shape() {
    glm::mat4 trans_mat = glm::translate(glm::mat4(1), glm::vec3(0, -0.3, 0));
    vao.model_matrix = trans_mat * glm::scale(glm::mat4(1), glm::vec3(0.3, 0.3, 1.0));
    vao.shape = ga::Shape::make_circle(0.03);
}

void MassSpringSystem::init_instances() {
    vao.n_instances = nodes.n_size;
    vao.p_xy.resize(2 * nodes.n_size_reserved);

    vao.n_springs = springs.n_size;
    vao.s_ab.resize(2 * springs.n_size_reserved);
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

    glGenBuffers(1, &vao.vbo_shape_id);
    glBindBuffer(GL_ARRAY_BUFFER, vao.vbo_shape_id);
    glBufferData(GL_ARRAY_BUFFER, 2 * vao.shape.n_vertices * sizeof(GLfloat), vao.shape.vertices.data(),
                 GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);

    glGenBuffers(1, &vao.vbo_instance_id);
    glBindBuffer(GL_ARRAY_BUFFER, vao.vbo_instance_id);
    glBufferData(GL_ARRAY_BUFFER, 2 * nodes.n_size_fix * sizeof(GLfloat), vao.p_xy.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(1);
    glVertexAttribDivisor(1, 1);

    glGenBuffers(1, &vao.vbo_springs_id);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vao.vbo_springs_id);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 2 * springs.n_size * sizeof(GLuint), vao.s_ab.data(), GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
};

void MassSpringSystem::draw() {
    glUseProgram(ga::Visualization::mass_spring_program.get_id());
    glBindVertexArray(vao.vao_id);

    // -> draw springs
    glUniform1i(ga::Visualization::mass_spring_program.uniform("mode"), 1);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vao.vbo_springs_id);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 2 * vao.n_springs * sizeof(GLuint), vao.s_ab.data(), GL_DYNAMIC_DRAW);
    glDrawElements(GL_LINES, 2 * vao.n_springs, GL_UNSIGNED_INT, 0);
    // <-

    // -> draw nodes
    glUniform1i(ga::Visualization::mass_spring_program.uniform("mode"), 0);
    glVertexAttribDivisor(1, 1);
    glBindBuffer(GL_ARRAY_BUFFER, vao.vbo_instance_id);
    glBufferData(GL_ARRAY_BUFFER, 2 * nodes.n_size_fix * sizeof(GLfloat), vao.p_xy.data(), GL_DYNAMIC_DRAW);
    glDrawArraysInstanced(vao.shape.type_primitive, 0, vao.shape.n_vertices, nodes.n_size_fix);
    glVertexAttribDivisor(1, 0);
    // <-

};

void MassSpringSystem::gather() {
    for (int i = 0; i < springs.n_size; i++) {
        vao.s_ab[i * 2] = static_cast<unsigned int>(nodes.key[springs.a[i]]);
        vao.s_ab[i * 2 + 1] = static_cast<unsigned int>(nodes.key[springs.b[i]]);
    }

    for (int i = 0; i < nodes.n_size_fix; i++) {
        vao.p_xy[i * 2] = static_cast<float>(nodes.p_x[i]);
        vao.p_xy[i * 2 + 1] = static_cast<float>(nodes.p_y[i]);
    }
};

void MassSpringSystem::attach_detach() {

    for (int i = 0; i < nodes.n_size_free; i++) {
        for (int j = i + 1; j < nodes.n_size_fix; j++) {
            double dx = nodes.p_x[i] - nodes.p_x[j];
            double dy = nodes.p_y[i] - nodes.p_y[j];
            double d = std::sqrt(dx * dx + dy * dy);
            if (d <= 0.5) {
                springs.push_back_sym_if_unique(i, j, 50.0, d);
            } else if (d > 1.0) {
                springs.remove_sym_if_unique(i, j);
            }
        }
    }
}

void MassSpringSystem::move() {
    double dt = 1e-2;
    /*
    attach_detach();

    for (int i = 0; i < nodes.n_size_free; i++) {
        nodes.f_x[i] = 0;
        nodes.f_y[i] = -2.5;
    }
    Solver::mss_force(nodes.p_x.data(), nodes.p_y.data(), nodes.f_x.data(), nodes.f_y.data(), nodes.n_size_fix,
                      springs.a.data(), springs.b.data(), springs.k.data(), springs.d_eq.data(), springs.n_size);

    Solver::euler_forward(nodes.p_x.data(), nodes.p_y.data(), nodes.v_x.data(), nodes.v_y.data(), nodes.f_x.data(),
                          nodes.f_y.data(), nodes.m.data(), dt, nodes.n_size_free);
    */
    gather();
}

void MassSpringSystem::spawn_floating_nodes(double pxi, double pyi) {
    if (nodes.n_size < nodes.n_size_reserved) {
        nodes.push_back_idle(pxi, pyi, 0, 0, 1.0, 1.0);
        nodes.transfer_idle2free(nodes.n_size - 1);
    }
}
