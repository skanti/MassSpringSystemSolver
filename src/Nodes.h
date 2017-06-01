#pragma once

#include <utility>
#include <cassert>
#include "Common.h"

template <typename value_type>
struct Nodes {

    void reserve(int n_size_reserve_) {
        n_size = 0;
        n_size_reserve = n_size_reserve_;

        p.resize(3, n_size_reserve);
        p_rhs.resize(3, n_size_reserve);
        q.resize(3, n_size_reserve);
        v.resize(3, n_size_reserve);
        f.resize(3, n_size_reserve);
        m.resize(n_size_reserve);
    }

    void init(int n_size_) {
        assert(n_size_ <= n_size_reserve);
        n_size = n_size_;
    }

    void set(int i, value_type pxi, value_type pyi, value_type pzi, value_type vxi, value_type vyi, value_type vzi, value_type mi) {
        p.col(i) = Vector3<value_type>(pxi, pyi, pzi);
        v.col(i) = Vector3<value_type>(vxi, vyi, vzi);
        m(i) = mi;
    }


    void insert(int i_protofilament, const int n_pt, Vector<int> &T0, Vector<int> &S0) {

        for (int i = n_pt - 1, j0 = n_size; i > i_protofilament; j0 -= T0[i], i--) {
            int j1 = j0 - T0[i];
            std::swap(S0[j0], S0[j1]);

        }
    }

    // -> nodes
    int n_size;
    int n_size_reserve;
    Matrix<value_type> p;
    Matrix<value_type> p_rhs;
    Matrix<value_type> q;
    Matrix<value_type> v;
    Matrix<value_type> f;
    Vector<value_type> m;
    // <-
};
