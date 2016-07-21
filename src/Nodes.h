#ifndef NODES_H
#define NODES_H

#include <vector>
#include <utility>


struct Nodes {
    Nodes(int n_size_reserved_)
            : n_size_free(0), n_size_fix(0), n_size(0),
              n_size_reserved(n_size_reserved_),
              p_x(n_size_reserved), p_y(n_size_reserved),
              v_x(n_size_reserved), v_y(n_size_reserved),
              m(n_size_reserved), d_size(n_size_reserved),
              key(n_size_reserved),
              f_x(n_size_reserved), f_y(n_size_reserved) {};

    void reserve(int n_size_reserved_) {
        n_size_reserved = n_size_reserved_;

        // node arrays
        p_x.resize(n_size_reserved);
        p_y.resize(n_size_reserved);
        v_x.resize(n_size_reserved);
        v_y.resize(n_size_reserved);
        m.resize(n_size_reserved);
        d_size.resize(n_size_reserved);
        key.reserve(n_size_reserved);

        // temporary arrays
        f_x.resize(n_size_reserved);
        f_y.resize(n_size_reserved);
    }

    void set(int i, double pxi, double pyi, double vxi, double vyi, double mi, double di, int keyi) {
        p_x[i] = pxi;
        p_y[i] = pyi;
        v_x[i] = vxi;
        v_y[i] = vyi;
        m[i] = mi;
        d_size[i] = di;
        key[i] = keyi;
    }

    void push_back_free(double pxi, double pyi, double vxi, double vyi, double mi, double di, int keyi) {
        set(n_size_free, pxi, pyi, vxi, vyi, mi, di, keyi);
        n_size_free++;
        n_size_fix++;
        n_size++;
    }

    void push_back_fix(double pxi, double pyi, double vxi, double vyi, double mi, double di, int keyi) {
        set(n_size_fix, pxi, pyi, vxi, vyi, mi, di, keyi);
        n_size_fix++;
        n_size++;
    }

    void push_back_idle(double pxi, double pyi, double vxi, double vyi, double mi, double di, int keyi) {
        set(n_size, pxi, pyi, vxi, vyi, mi, di, keyi);
        n_size++;
    }

    void swap(int i, int j) {
        std::swap(p_x[i], p_x[j]);
        std::swap(p_y[i], p_y[j]);
        std::swap(v_x[i], v_x[j]);
        std::swap(v_y[i], v_y[j]);
        std::swap(m[i], m[j]);
        std::swap(d_size[i], d_size[j]);
        std::swap(key[i], key[j]);
    }

    void remove_free(int i) {
        swap(i, n_size_free - 1);
        swap(n_size_free - 1, n_size_fix - 1);
        swap(n_size_fix - 1, n_size - 1);
        n_size_free--;
        n_size_fix--;
        n_size--;
    }

    void remove_fix(int i) {
        swap(i, n_size_fix - 1);
        swap(n_size_fix - 1, n_size - 1);
        n_size_fix--;
        n_size--;
    }

    void remove_idle(int i) {
        swap(i, n_size - 1);
        n_size--;
    }

    void transfer_free2fix(int i) {
        swap(i, n_size_free - 1);
        n_size_free--;
    }

    void transfer_free2idle(int i) {
        swap(i, n_size_free - 1);
        swap(n_size_free - 1, n_size_fix - 1);
        n_size_free--;
        n_size_fix--;
    }

    void transfer_fix2free(int i) {
        swap(i, n_size_free);
        n_size_free++;
    }

    void transfer_fix2idle(int i) {
        swap(i, n_size_fix - 1);
        n_size_fix--;
    }

    void transfer_idle2free(int i) {
        swap(i, n_size_fix);
        swap(n_size_fix, n_size_free);
        n_size_free++;
        n_size_fix++;
    }

    void transfer_idle2fix(int i) {
        swap(i, n_size_fix);
        n_size_fix++;
    }

    // -> nodes
    int n_size_free;
    int n_size_fix;
    int n_size;
    int n_size_reserved;
    std::vector<double> p_x;
    std::vector<double> p_y;
    std::vector<double> v_x;
    std::vector<double> v_y;
    std::vector<double> m;
    std::vector<double> d_size;
    std::vector<int> key;
    // <-

    // -> dynamic-temporary. reset each iteration
    std::vector<double> f_x;
    std::vector<double> f_y;
    // <-
};

#endif //GRAVITYASSIST_PHYSICALPROPERTIES_H
