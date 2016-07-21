#include "MassSpringSystem.h"
#include "Visualization.h"
#include "ImageManager.h"

MassSpringSystem::MassSpringSystem() {
    load_file("/Users/amon/grive/development/Springs2D/data/mesh.msh");
    init_nodes_and_springs();
    n_nodes = nodes.size();
    n_springs = springs.size();
    init_shape();
    init_instances();
    init_drawable();
}

void MassSpringSystem::load_file(std::string file) {
    const int tag_phase_fix = 12;
    std::ifstream infile(file);
    std::string line;
    while (std::getline(infile, line)) {
        if (line == std::string("$Nodes")) {
            std::getline(infile, line);
            std::istringstream issn(line);
            issn >> n_nodes;
            for (int i = 0; i < n_nodes; i++) {
                std::getline(infile, line);
                std::istringstream issf(line);
                int k;
                double x, y;
                issf >> k >> x >> y;
                arma::vec xx = {x, y};
                arma::vec z = {0, 0};
                nodes[k - 1] = {xx, z, z, 1.0, false};
            }
        } else if (line == std::string("$Elements")) {

            std::getline(infile, line);
            std::istringstream issn(line);
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
                        nodes[n1 - 1].fix = true;
                        nodes[n2 - 1].fix = true;
                    }
                }
                if (type == 2) {
                    int phase, tag, ent, n1, n2, n3;
                    isse >> phase >> tag >> ent >> n1 >> n2 >> n3;
                    springs.stack({n1 - 1, n2 - 1}, {250.0, 1.0});
                    springs.stack({n1 - 1, n3 - 1}, {250.0, 1.0});
                    springs.stack({n2 - 1, n3 - 1}, {250.0, 1.0});
                }
            }
        }
    }
}

void MassSpringSystem::init_nodes_and_springs() {
    for (auto &s : springs) {
        const arma::vec dx = nodes[s.first.second].x - nodes[s.first.first].x;
        const double dl = arma::norm(dx, 2);
        s.second.d = dl;
    }
}

void MassSpringSystem::init_shape() {
    vao.model_matrix = glm::scale(glm::mat4(1), glm::vec3(0.5, 0.5, 1.0));
    vao.shape = ga::Shape::make_circle(0.01);
    vao.n_springs = n_springs;
    vao.s_ab.resize(n_springs * 2);
}

void MassSpringSystem::init_instances() {
    vao.n_instances = n_nodes;
    vao.p_xy.resize(2 * n_nodes);
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
    std::vector<float> color{{0.2f, 0.2f, 0.8f, 0.6f}};
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

void MassSpringSystem::move() {
    /*
    const double dt = 0.01;
    const arma::vec f_g = {0.0, -9.81};
    for (auto &n : nodes)
        n.second.f *= 0.0;

    for (auto &s : springs) {
        const arma::vec dx = nodes[s.first.second].x - nodes[s.first.first].x;
        const double dl = arma::norm(dx, 2);
        const arma::vec f = (dl - s.second.d) * s.second.k * dx / dl;

        nodes[s.first.first].f += f;
        nodes[s.first.second].f -= f;
    }
    for (auto &n : nodes) {
        if (!n.second.fix) {
            n.second.v += (-0.5 * n.second.v + n.second.f / n.second.m + f_g) * dt;
            n.second.x += n.second.v * dt;
        }
    }
    */

    int i_spring = 0;
    for (auto &s : springs) {
        vao.s_ab[i_spring++] = static_cast<unsigned int> (s.first.first);
        vao.s_ab[i_spring++] = static_cast<unsigned int> (s.first.second);
    }
    int i_node = 0;
    for (auto &n : nodes) {
        vao.p_xy[i_node++] = (float) (n.second.x[0]);
        vao.p_xy[i_node++] = (float) (n.second.x[1]);
    }

};
