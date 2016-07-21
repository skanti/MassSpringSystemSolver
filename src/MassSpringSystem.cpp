#include "MassSpringSystem.h"
#include "Visualization.h"
#include "ImageManager.h"

MassSpringSystem::MassSpringSystem() {
    load_file("/Users/amon/grive/development/Springs2D/data/mesh.msh");
    init_nodes_and_springs();
    n_nodes = nodes.size();
    n_springs = springs.size();
    init_shape();
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
    vao.shape = ga::Shape::make_circle(0.003);
}

void MassSpringSystem::init_drawable() {
    glUseProgram(ga::Visualization::mass_spring_program.get_id());
    glGenVertexArrays(1, &vao.vao_id);
    glBindVertexArray(vao.vao_id);

    glUniformMatrix4fv(ga::Visualization::mass_spring_program.uniform("ViewMatrix"), 1, GL_FALSE,
                       &ga::Visualization::view_window[0][0]);
    glUniformMatrix4fv(ga::Visualization::mass_spring_program.uniform("ProjectionMatrix"), 1, GL_FALSE,
                       &ga::Visualization::projection_window[0][0]);
    std::vector<float> green{{0.2f, 0.8f, 0.2f, 0.5f}};
    glUniform4fv(ga::Visualization::mass_spring_program.uniform("color"), 1, green.data());
    glUniform2fv(ga::Visualization::mass_spring_program.uniform("shape"), vao.shape.n_vertices,
                 vao.shape.vertices.data());

    glGenBuffers(1, &vao.vbo_node);
    glBindBuffer(GL_UNIFORM_BUFFER, vao.vbo_node);
    glBufferData(GL_UNIFORM_BUFFER, n_nodes * sizeof(glm::vec4), 0, GL_DYNAMIC_DRAW);
    glUniformBlockBinding(ga::Visualization::mass_spring_program.get_id(),
                          glGetUniformBlockIndex(ga::Visualization::mass_spring_program.get_id(), "nodes"), 0);

    glGenBuffers(1, &vao.vbo_springs);
    glBindBuffer(GL_UNIFORM_BUFFER, vao.vbo_springs);
    glBufferData(GL_UNIFORM_BUFFER, n_springs * sizeof(glm::uvec4), 0, GL_DYNAMIC_DRAW);
    glUniformBlockBinding(ga::Visualization::mass_spring_program.get_id(),
                          glGetUniformBlockIndex(ga::Visualization::mass_spring_program.get_id(), "springs"), 1);
};

void MassSpringSystem::draw() {
    glUseProgram(ga::Visualization::mass_spring_program.get_id());
    glBindVertexArray(vao.vao_id);

    // nodes
    glBindBufferBase(GL_UNIFORM_BUFFER, 0, vao.vbo_node);
    glm::vec4 *node = (glm::vec4 *) glMapBufferRange(
            GL_UNIFORM_BUFFER, 0, n_nodes * sizeof(glm::vec4), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
    for (int i = 0; i < n_nodes; i++) {
        node[i] = glm::vec4(nodes[i].x[0], nodes[i].x[1], 0, 1);
    }
    glUnmapBuffer(GL_UNIFORM_BUFFER);

    glUniform1i(ga::Visualization::mass_spring_program.uniform("mode"), 0);
    for (int i = 0; i < n_nodes; i++) {
        glVertexAttribI1i(0, i);
        glDrawArrays(vao.shape.type_primitive, 0, vao.shape.n_vertices);
    }

    // springs
    glBindBufferBase(GL_UNIFORM_BUFFER, 1, vao.vbo_springs);
    glm::uvec4 *spring = (glm::uvec4 *) glMapBufferRange(
            GL_UNIFORM_BUFFER, 0, n_springs * sizeof(glm::uvec4), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
    int j = 0;
    for (auto &s : springs) {
        spring[j] = glm::uvec4(s.first.first, s.first.second, 0, 1);
        j++;
    }
    glUnmapBuffer(GL_UNIFORM_BUFFER);

    glUniform1i(ga::Visualization::mass_spring_program.uniform("mode"), 1);
    glLineWidth(10.0);
    glEnable(GL_LINE_SMOOTH);
    for (int i = 0; i < n_springs; i++) {
        glVertexAttribI1ui(1, i);
        glDrawArrays(GL_LINES, 0, 2);
    }
    glDisable(GL_LINE_SMOOTH);

};

void MassSpringSystem::move() {
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
};
