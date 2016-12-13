#include <iostream>
#include <mkl.h>
#include "Common.h"
#include <utility>
#include <algorithm>
#include <numeric>
#include <vector>
#include "Timer.h"
#include <cstring>

template<typename value_type>
void coo2csr(std::vector<std::pair<std::pair<int32_t, int32_t>, value_type>> &coo, CSR<value_type> &matrix) {
	int n_nz = coo.size();
	matrix.i_y[0] = coo[0].first.first;
	matrix.i_x[0] = coo[0].first.second;
	matrix.data[0] = coo[0].second;
	int i1 = 1; // <-- current row pointer
	matrix.i_y[1] = 1;
	for (int i = 1; i < n_nz; i++) {
		matrix.i_x[i] = coo[i].first.second;
		matrix.data[i] = coo[i].second;

		if (i1 == coo[i].first.first) {
			i1++;
			matrix.i_y[i1] += matrix.i_y[i1-1] + 1; 
		} else { 
			matrix.i_y[i1]++;
			matrix.i_y[i1 + 1] = 0;
		}
	}
};

int main() {
	// -> init matrices
	int n = 1 << 8;
	CSR<float> A;
	std::vector<float> b(n, 1.0);
	std::vector<float> x(n, -1.0);
	A.init(n, n, n);
    std::vector<std::pair<std::pair<int32_t, int32_t>, float>> coo;
	for (int i = 0; i < n; i++) {
		coo.push_back({{i,i}, 1.0});
	}
	coo2csr(coo, A);
	// <-
	
	// -> init pardiso
	void *pt[64];
	int32_t iparm[64];
	const int32_t mtype = 2;
	pardisoinit(pt, &mtype,  iparm);
	// <-
	iparm[36] = -50;
	// for (int i = 0; i < 64; i++) {
	// 	std::cout << i << " " << iparm[i] << std::endl;
	// }
	//exit(0);

	// -> solve
	const int32_t maxfct = 1;
	const int32_t mnum = 1;
	const int32_t phase = 11;
	int32_t int_dummy;
	const int32_t nrhs = 1;
	const int32_t msglvl = 1;
	int32_t error;
	Timer::start();
	pardiso(pt, &maxfct, &mnum, &mtype, &phase, &n, A.data, A.i_y, A.i_x, &int_dummy, &nrhs, iparm, &msglvl, b.data(), x.data(), &error );
	Timer::stop();
	std::cout << "timing (ms): " << Timer::get_timing()*1000 << std::endl;
	// <-


	// -> output
	for (int i = 0; i < n; i++)
		std::cout << x[i] << std::endl;
	// <-
	return 0;
}