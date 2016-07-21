#ifndef SPRINGS2D_MASSSPRINGSYSTEM_H
#define SPRINGS2D_MASSSPRINGSYSTEM_H

#include "Drawable.h"
#include <armadillo>
#include "Shape.h"
#include "VertexObject.h"
#include <unordered_map>
#include <utility>

class MassSpringSystem : public ga::Drawable {
public:

    struct Spring {
        double k, d;
    };
    struct Node {
        arma::Col<double> x;
        arma::Col<double> v;
        arma::Col<double> f;
        double m;
        bool fix;
    };

    struct HashPair {
    public:
        template<typename T, typename U>
        int operator()(const std::pair<T, U> &x) const {
            return std::hash<T>()(x.first) ^ std::hash<U>()(x.second);
        }
    };

    // symmetric unordered map
    class SymUnMap : public std::unordered_map<std::pair<int, int>, Spring, HashPair> {
    public:
        bool stack(std::pair<int, int> i, Spring s) {
            std::pair<int, int> p;
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

        Spring &get(std::pair<int, int> i) {
            std::pair<int, int> p;
            if (i.first <= i.second)
                p = {i.first, i.second};
            else
                p = {i.second, i.first};
            return this->operator[](p);
        }

    };

    MassSpringSystem();

    void load_file(std::string file);

    void init_nodes_and_springs();

    void init_shape();

    void init_instances();

    void init_drawable();

    void draw();

    void move();

private:
    ga::VAOMassSpring vao;

    int n_nodes;
    int n_springs;
    std::unordered_map<int, Node> nodes;
    SymUnMap springs;
};


#endif //SPRINGS2D_MASSSPRINGSYSTEM_H
