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
        d0.resize(n_size_reserve);
        d.resize(3, n_size_reserve);
        d0_rhs.resize(n_size_reserve);
        d_rhs.resize(3, n_size_reserve);
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
        std::swap(k(a), k(b));
        std::swap(d0(a), d0(b));
        std::swap(d(a, 0), d(b, 0));
        std::swap(d(a, 1), d(b, 1));
        std::swap(d(a, 2), d(b, 2));
        std::swap(d_rhs(a, 0), d_rhs(b, 0));
        std::swap(d_rhs(a, 1), d_rhs(b, 1));
        std::swap(d_rhs(a, 2), d_rhs(b, 2));
        std::swap(d0_rhs(a), d0_rhs(b));
    }

    void set_as_equilibrium(Matrix<value_type> &p, SparseMatrix<value_type> &A) {
        for (int j = 0; j < n_size; j++)
             set_as_equilibrium1(p, A, j);
    }

    void set_as_equilibrium1(Matrix<value_type> &p, SparseMatrix<value_type> &A, int j) {
        int pi0 = A.outerIndexPtr()[j];
        int a = A.innerIndexPtr()[pi0];
        int b = A.innerIndexPtr()[pi0 + 1];
        Vector3<value_type> dj = p.col(b) - p.col(a);
        d0(j) = dj.norm();
        d.col(j) = dj.normalized();
    }

    int32_t n_size;
    int32_t n_size_reserve;
    Vector<int32_t> key;
    Vector<value_type> k;
    Vector<value_type> d0;
    Matrix<value_type> d;
    Vector<value_type> d0_rhs;
    Matrix<value_type> d_rhs;
    
};

