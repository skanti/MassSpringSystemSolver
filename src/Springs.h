#pragma once

#include <cstdlib>
#include <cassert>
#include "Common.h"

template<typename value_type>
struct Springs {

    void reserve(int32_t n_size_reserve_) {
        n_size = 0;
        n_size_reserve = n_size_reserve_;
        k.resize(n_size_reserve);
        key.resize(n_size_reserve);
        d.resize(n_size_reserve);
        dx.resize(n_size_reserve);
        dy.resize(n_size_reserve);
        dz.resize(n_size_reserve);
        d_rhs.resize(n_size_reserve);
        dx_rhs.resize(n_size_reserve);
        dy_rhs.resize(n_size_reserve);
        dz_rhs.resize(n_size_reserve);
    }
    

    void init(int32_t n_size_) {
        assert(n_size_ <= n_size_reserve);
        n_size = n_size_;
    }

    void swap(int32_t a, int32_t b, SparseMatrix<value_type> &A) {
        key[a] = b;
        key[b] = a;
        int ia = A.outerIndexPtr()[a];
        int ib = A.outerIndexPtr()[b];
        std::swap(A.innerIndexPtr()[ia], A.innerIndexPtr()[ib]);
        std::swap(A.innerIndexPtr()[ia+1], A.innerIndexPtr()[ib+1]);
        std::swap(A.valuePtr()[ia], A.valuePtr()[ib]);
        std::swap(A.valuePtr()[ia+1], A.valuePtr()[ib+1]);
        std::swap(k[a], k[b]);
        std::swap(d[a], d[b]);
        std::swap(dx[a], dx[b]);
        std::swap(dy[a], dy[b]);
        std::swap(dz[a], dz[b]);
        std::swap(dx_rhs[a], dx_rhs[b]);
        std::swap(dy_rhs[a], dy_rhs[b]);
        std::swap(dz_rhs[a], dz_rhs[b]);
        std::swap(d_rhs[a], d_rhs[b]);
    }

    void set_as_equilibrium(Vector<value_type> &px, Vector<value_type> &py, Vector<value_type> &pz, SparseMatrix<value_type> &A) {
        for (int j = 0; j < n_size; j++)
             set_as_equilibrium1(px, py, pz, A, j);
    }

    void set_as_equilibrium1(Vector<value_type> &px, Vector<value_type> &py, Vector<value_type> &pz, SparseMatrix<value_type> &A, int j) {
        int pi0 = A.outerIndexPtr()[j];
        int a = A.innerIndexPtr()[pi0];
        int b = A.innerIndexPtr()[pi0 + 1];
        value_type dxj = -px[a] + px[b];
        value_type dyj = -py[a] + py[b];
        value_type dzj = -pz[a] + pz[b];
        d[j] = std::sqrt(dxj*dxj + dyj*dyj + dzj*dzj);
        dx[j] = dxj/d[j];
        dy[j] = dyj/d[j];
        dz[j] = dzj/d[j];
    }

    int32_t n_size;
    int32_t n_size_reserve;
    Vector<int> key;
    Vector<value_type> k;
    Vector<value_type> d;
    Vector<value_type> dx;
    Vector<value_type> dy;
    Vector<value_type> dz;
    Vector<value_type> d_rhs;
    Vector<value_type> dx_rhs;
    Vector<value_type> dy_rhs;
    Vector<value_type> dz_rhs;
};

