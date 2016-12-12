#pragma once

#include <vector>
#include <utility>


template <typename value_type, int alignment = 16>
struct Nodes {

    void init(int n_size_) {
        n_size = n_size_;

        p_x.resize(n_size);
        p_y.resize(n_size);
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
    std::vector<value_type, AlignedAllocator<value_type, alignment>> p_x;
    std::vector<value_type, AlignedAllocator<value_type, alignment>> p_y;
    std::vector<value_type, AlignedAllocator<value_type, alignment>> v_x;
    std::vector<value_type, AlignedAllocator<value_type, alignment>> v_y;
    std::vector<value_type, AlignedAllocator<value_type, alignment>> m;
    // <-

    // -> dynamic-temporary. reset each iteration
    std::vector<value_type, AlignedAllocator<value_type, alignment>> f_x;
    std::vector<value_type, AlignedAllocator<value_type, alignment>> f_y;
    // <-
};
