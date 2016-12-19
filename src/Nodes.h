#pragma once

#include <utility>
#include "Common.h"

template <typename value_type>
struct Nodes {

    void init(int n_size_) {
        n_size = n_size_;

        p_x.resize(n_size);
        p_y.resize(n_size);
        q_x.resize(n_size);
        q_y.resize(n_size);
        v_x.resize(n_size);
        v_y.resize(n_size);
        m.resize(n_size);
        f_x.resize(n_size);
        f_y.resize(n_size);
    }

    void set(int i, value_type pxi, value_type pyi, value_type vxi, value_type vyi, value_type mi) {
        p_x[i] = pxi;
        p_y[i] = pyi;
        v_x[i] = vxi;
        v_y[i] = vyi;
        m[i] = mi;
    }

    // -> nodes
    int n_size;
    Vector<value_type> p_x;
    Vector<value_type> p_y;
    Vector<value_type> q_x;
    Vector<value_type> q_y;
    Vector<value_type> v_x;
    Vector<value_type> v_y;
    Vector<value_type> g_x;
    Vector<value_type> g_y;
    Vector<value_type> m;
    // <-

    // -> dynamic-temporary. reset each iteration
    Vector<value_type> f_x;
    Vector<value_type> f_y;
    // <-
};
