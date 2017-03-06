#pragma once

#include <utility>
#include <cassert>
#include "Common.h"

template <typename value_type>
struct Nodes {

    void reserve(int n_size_reserve_) {
        n_size = 0;
        n_size_reserve = n_size_reserve_;
        px.resize(n_size_reserve);
        py.resize(n_size_reserve);
        pz.resize(n_size_reserve);
        px_rhs.resize(n_size_reserve);
        py_rhs.resize(n_size_reserve);
        pz_rhs.resize(n_size_reserve);
        qx.resize(n_size_reserve);
        qy.resize(n_size_reserve);
        qz.resize(n_size_reserve);
        ix.resize(n_size_reserve);
        iy.resize(n_size_reserve);
        iz.resize(n_size_reserve);
        vx.resize(n_size_reserve);
        vy.resize(n_size_reserve);
        vz.resize(n_size_reserve);
        m.resize(n_size_reserve);
        fx.resize(n_size_reserve);
        fy.resize(n_size_reserve);
        fz.resize(n_size_reserve);
    }

    void init(int n_size_) {
        assert(n_size_ <= n_size_reserve);
        n_size = n_size_;
    }

    void set(int i, value_type pxi, value_type pyi, value_type pzi, value_type vxi, value_type vyi, value_type vzi, value_type mi) {
        px[i] = pxi;
        py[i] = pyi;
        pz[i] = pzi;
        vx[i] = vxi;
        vy[i] = vyi;
        vz[i] = vzi;
        m[i] = mi;
    }

    // void swap(int i, int j) {
    //     std::swap(px[i], px[j]);
    //     std::swap(py[i], py[j]);
    //     std::swap(pz[i], pz[j]);
    //     std::swap(vx[i], vx[j]);
    //     std::swap(vy[i], vy[j]);
    //     std::swap(vz[i], vz[j]);
    //     std::swap(m[i], m[j]);
    // }

    void insert(int i_protofilament, const int n_pt, Vector<int> &T0, Vector<int> &S0) {

        for (int i = n_pt - 1, j0 = n_size; i > i_protofilament; j0 -= T0[i], i--) {
            int j1 = j0 - T0[i];
            std::swap(S0[j0], S0[j1]);

        }
    }

    // -> nodes
    int n_size;
    int n_size_reserve;
    Vector<value_type> px;
    Vector<value_type> py;
    Vector<value_type>  pz;
    Vector<value_type> px_rhs;
    Vector<value_type> py_rhs;
    Vector<value_type> pz_rhs;
    Vector<value_type> qx;
    Vector<value_type> qy;
    Vector<value_type> qz;
    Vector<value_type> ix;
    Vector<value_type> iy;
    Vector<value_type> iz;
    Vector<value_type> vx;
    Vector<value_type> vy;
    Vector<value_type> vz;
    Vector<value_type> m;
    // <-

    // -> dynamic-temporary. reset each iteration
    Vector<value_type> fx;
    Vector<value_type> fy;
    Vector<value_type> fz;
    // <-
};
