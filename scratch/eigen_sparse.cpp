#include <iostream>
#include "Common.h"
#include <utility>
#include <algorithm>
#include <numeric>
#include <vector>
#include "Timer.h"
#include <cstring>
#include <eigen3/Eigen/SparseCholesky>
#include <eigen3/Eigen/Sparse>

typedef Eigen::SparseMatrix<float> SparseMatrix;
typedef Eigen::Triplet<float> Triplet;

int main() {
	// -> init matrices
	int n = 1 << 8;
	SparseMatrix A(n, n);
	Eigen::VectorXf b(n);
	for (int i = 0; i < n; i++) b[i] = 1.0;
	std::vector<Triplet> coo(n);
	for (int i = 0; i < n; i++) {
		coo[i] = Triplet(i,i, 1.0f);
	}
	A.setFromTriplets(coo.begin(), coo.end());
	// <-


	// -> solve A*x = b
	Timer::start();
	Eigen::SimplicialCholesky<SparseMatrix> chol(A);
	Eigen::VectorXf x = chol.solve(b);
	Timer::stop();
	std::cout << "timing (ms): " << Timer::get_timing()*1000 << std::endl;
	// <-

	return 0;
}