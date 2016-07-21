#include "MassSpringSystem.h"
#include "Visualization.h"
#include "Solvers.h"

const int n_springs_per_node = 8;

MassSpringSystem::MassSpringSystem() : nodes(30), springs(30 * 8) {
    load_file("/Users/amon/grive/development/Springs2D/data/mesh.msh");
    //init_shape();
    //init_instances();
    //init_drawable();
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
                        nodes.transfer_free2fix(nn1);
                        nodes.transfer_free2fix(nn2);
                    }
                }
                if (type == 2) {
                    int phase, tag, ent, n1, n2, n3;
                    isse >> phase >> tag >> ent >> n1 >> n2 >> n3;
                    int nn1 = nodes.key[n1 - 1];
                    int nn2 = nodes.key[n2 - 1];
                    int nn3 = nodes.key[n3 - 1];
                    springs.push_back(nn1, nn2, 1.0, i_spring++);
                    springs.push_back(nn1, nn3, 1.0, i_spring++);
                    springs.push_back(nn2, nn3, 1.0, i_spring++);
                }
            }
        }
    }
}


void MassSpringSystem::init_shape() {
    vao.model_matrix = glm::scale(glm::mat4(1), glm::vec3(0.5, 0.5, 1.0));
    vao.shape = ga::Shape::make_circle(0.01);
    vao.s_ab.resize(springs.n_size * 2);
}

void MassSpringSystem::init_instances() {
    vao.n_instances = nodes.n_size;
    vao.p_xy.resize(2 * nodes.n_size);
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
    /*
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
    */
};

void MassSpringSystem::move() {

};
