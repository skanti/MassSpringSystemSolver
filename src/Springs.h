#pragma once

#include <cstdlib>
#include "Common.h"

template<typename value_type>
struct Springs {

    void init(int32_t n_nodes, int32_t n_springs) {
        n_size = n_springs;
        k.resize(n_springs);
        rd.resize(n_springs);
        dx.resize(n_springs);
        dy.resize(n_springs);
        K.resize(n_springs, n_springs);
        A.resize(n_nodes, n_springs);
        AA.resize(n_nodes, n_nodes);
    }
    

    void set_as_equilibrium(Vector<value_type> &px, Vector<value_type> &py) {
        for (int j = 0; j < n_size; j++) {
            int pi0 = A.outerIndexPtr()[j];
            int a = A.innerIndexPtr()[pi0];
            int b = A.innerIndexPtr()[pi0 + 1];
            value_type dxj = -px[a] + px[b];
            value_type dyj = -py[a] + py[b];
            rd[j] = std::sqrt(dxj*dxj + dyj*dyj);
            dx[j] = dxj/rd[j];
            dy[j] = dyj/rd[j];
        }    
    }

    int32_t n_size;
    Vector<value_type> k;
    Vector<value_type> rd;
    Vector<value_type> dx;
    Vector<value_type> dy;
    SparseMatrix<value_type> K;
    SparseMatrix<value_type> A;
    SparseMatrix<value_type> AA;
};

