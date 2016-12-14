#pragma once

#include <cmath>
#include <cassert>
#include "Common.h"

template<typename value_type>
static void sp_mm1_mul(SparseMatrix<value_type> &A, Vector<value_type> &B, SparseMatrix<value_type> &C) {
	int nj = A.cols();
	assert(A.cols() == B.rows());
	for (int j = 0; j < nj; j++) {
	   	for (int pi = A.outerIndexPtr()[j]; pi < A.outerIndexPtr()[j+1]; pi++) {
			C.valuePtr()[pi] = A.valuePtr()[pi] * B[j];
		}
    }

}
template<typename value_type>
static void sp_mm1_add(SparseMatrix<value_type> &A, Vector<value_type> &B, SparseMatrix<value_type> &C, value_type alpha) {
	int nj = A.cols();
	assert(A.cols() == B.rows());
	for (int j = 0; j < nj; j++) {
	   	for (int pi = A.outerIndexPtr()[j]; pi < A.outerIndexPtr()[j+1]; pi++) {
			int i = A.innerIndexPtr()[pi];
			C.valuePtr()[pi] = A.valuePtr()[pi] *alpha +  (i == j)*B[j];
		}
    }

}
