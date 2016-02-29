#include "World.h"
#include "Engine.h"
#include "Movable.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <regex>
#include <unordered_map>
#include <boost/algorithm/string.hpp>
#include <functional>


#define BOOST_DISABLE_ASSERTS 1

float DT = 0.001;

using namespace lb;
using namespace boost::numeric;

class Command {
public:
    virtual void execute() = 0;
};

class MassSpringSystem : public Drawable, public Movable {
public:
    struct Spring {
        double k, rt;
    };
    struct Node {
        double x, y, m; // x-pos, y-pos, mass
        bool fix;
        std::deque<Spring *> s;
    };

    struct HashPair {
    public:
        template<typename T, typename U>
        std::size_t operator()(const std::pair<T, U> &x) const {
            return std::hash<T>()(x.first) ^ std::hash<U>()(x.second);
        }
    };

    class SUM : public std::unordered_map<std::pair<sizet, sizet>, Spring, HashPair> {
    public:
        bool stack(std::pair<sizet, sizet> i, Spring s) {
            std::pair<sizet, sizet> p;
            if (i.first <= i.second)
                p = {i.first, i.second};
            else
                p = {i.second, i.first};
            bool duplicate = false;
            for (auto &m: *this) {
                if (m.first == p) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate)
                this->operator[](p) = s;
            return !duplicate;
        }

        Spring &get(std::pair<sizet, sizet> i) {
            std::pair<sizet, sizet> p;
            if (i.first <= i.second)
                p = {i.first, i.second};
            else
                p = {i.second, i.first};
            return this->operator[](p);
        }

    };

    MassSpringSystem()
            : vao(),
              c1(0, 0, 0.03f, 20) {
        load_file("/Users/amon/grive/development/springs/data/mesh1.msh");
        n_nodes = nodes.size();
        n_springs = springs.size();
        init_drawable();
    }

    void load_file(std::string file) {
        std::ifstream infile(file);
        std::string line;
        while (std::getline(infile, line)) {
            if (line == std::string("$Nodes")) {
                std::getline(infile, line);
                std::istringstream issn(line);
                issn >> n_nodes;
                for (sizet i = 0; i < n_nodes; i++) {
                    std::getline(infile, line);
                    std::istringstream issf(line);
                    int k;
                    double x, y;
                    issf >> k >> x >> y;
                    nodes[k - 1] = {x, y, 1.0, false};
                }
            } else if (line == std::string("$Elements")) {
                std::getline(infile, line);
                std::istringstream issn(line);
                issn >> n_springs;
                for (sizet i = 0; i < n_springs; i++) {
                    std::getline(infile, line);
                    std::istringstream isse(line);
                    int k, type;
                    isse >> k >> type;
                    if (type == 2) {
                        sizet phase, tag, geo, n1, n2, n3;
                        isse >> phase >> tag >> geo >> n1 >> n2 >> n3;
                        if (springs.stack({n1 - 1, n2 - 1}, {1.0, 1.0})) {
                            nodes[n1 - 1].s.push_back(&springs.get({n1 - 1, n2 - 1}));
                            nodes[n2 - 1].s.push_back(&springs.get({n1 - 1, n2 - 1}));
                        }
                        if (springs.stack({n1 - 1, n3 - 1}, {1.0, 1.0})) {
                            nodes[n1 - 1].s.push_back(&springs.get({n1 - 1, n3 - 1}));
                            nodes[n3 - 1].s.push_back(&springs.get({n1 - 1, n3 - 1}));
                        }
                        if (springs.stack({n2 - 1, n3 - 1}, {1.0, 1.0})) {
                            nodes[n2 - 1].s.push_back(&springs.get({n2 - 1, n3 - 1}));
                            nodes[n3 - 1].s.push_back(&springs.get({n2 - 1, n3 - 1}));
                        }
                    }
                }
            }
        }
    }

    void init_drawable() {
        glUseProgram(mass_spring_program.get_id());
        glGenVertexArrays(1, &vao.vao_id);
        glBindVertexArray(vao.vao_id);

        glUniformMatrix4fv(mass_spring_program.uniform("ViewMatrix"), 1, GL_FALSE, &view_window[0][0]);
        glUniformMatrix4fv(mass_spring_program.uniform("ProjectionMatrix"), 1, GL_FALSE, &projection_window[0][0]);
        std::vector<float> green{{0.2f, 0.8f, 0.2f, 0.5f}};
        glUniform4fv(mass_spring_program.uniform("color"), 1, green.data());
        glUniform2fv(mass_spring_program.uniform("shape"), c1.N_VERTICES_FAN, c1.vertices_fan.data());

        glGenBuffers(1, &vao.vbo_node);
        glBindBuffer(GL_UNIFORM_BUFFER, vao.vbo_node);
        glBufferData(GL_UNIFORM_BUFFER, n_nodes * sizeof(glm::vec4), 0, GL_DYNAMIC_DRAW);
        glUniformBlockBinding(mass_spring_program.get_id(),
                              glGetUniformBlockIndex(mass_spring_program.get_id(), "nodes"), 0);

        glGenBuffers(1, &vao.vbo_springs);
        glBindBuffer(GL_UNIFORM_BUFFER, vao.vbo_springs);
        glBufferData(GL_UNIFORM_BUFFER, n_springs * sizeof(glm::uvec4), 0, GL_DYNAMIC_DRAW);
        glUniformBlockBinding(mass_spring_program.get_id(),
                              glGetUniformBlockIndex(mass_spring_program.get_id(), "springs"), 1);
    };

    void draw() {
        glUseProgram(mass_spring_program.get_id());
        glBindVertexArray(vao.vao_id);

        // nodes
        glBindBufferBase(GL_UNIFORM_BUFFER, 0, vao.vbo_node);
        glm::vec4 *node = (glm::vec4 *) glMapBufferRange(
                GL_UNIFORM_BUFFER, 0, n_nodes * sizeof(glm::vec4), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
        for (sizet i = 0; i < n_nodes; i++) {
            node[i] = glm::vec4(nodes[i].x, nodes[i].y, 0, 1);
        }
        glUnmapBuffer(GL_UNIFORM_BUFFER);

        glUniform1i(mass_spring_program.uniform("mode"), 0);
        for (sizet i = 0; i < n_nodes; i++) {
            glVertexAttribI1i(0, i);
            glDrawArrays(GL_TRIANGLE_FAN, 0, c1.N_VERTICES_FAN);
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

        glUniform1i(mass_spring_program.uniform("mode"), 1);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(10.0f);
        for (sizet i = 0; i < n_springs; i++) {
            glVertexAttribI1ui(1, i);
            glDrawArrays(GL_LINES, 0, 2);
        }
        glDisable(GL_LINE_SMOOTH);

    };

    void move() { return; };

private:
    VAOMassSpring vao;
    Circle c1;

private:
    sizet n_nodes;
    sizet n_springs;
    std::unordered_map<sizet, Node> nodes;
    SUM springs;
};

class DrawUnitCommand : public Command {
public:
    DrawUnitCommand(Drawable *unit) : unit(unit) {
    }

    void execute() {
        unit->draw();
    }

private:
    Drawable *unit;
};


class Earth : public World {
public:
    Earth()
            : World(),
              mss(),
              draw_unit_command((Drawable *) &mss) {

    };

    void advance(unsigned int &iteration_counter, long long int ms_per_frame) {
        return;
    }

    void draw() {
        draw_unit_command.execute();
    }

private:
    MassSpringSystem mss;
    DrawUnitCommand draw_unit_command;

};

int main(int argc, char *argv[]) {
    Engine::init();
    Earth earth = Earth();
    Engine::get_instance().add_world(&earth);
    lb::Engine::get_instance().run();

    return 0;
}
