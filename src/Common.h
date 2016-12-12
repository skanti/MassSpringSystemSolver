#pragma once

#include <cstdlib>
#include <cassert>

template<typename value_type, int alignment = 16>
struct CSR {

	void init(int n_y_, int n_x_, int n_nz_) {
		n_y = n_y_;
		n_x = n_x_;
		n_nz = n_nz_;
		assert(n_y*n_x >= n_nz_);
		posix_memalign((void**) &data, alignment, sizeof(value_type)*n_nz);
		posix_memalign((void**) &i_x, alignment, sizeof(int32_t)*n_nz);
		posix_memalign((void**) &i_y, alignment, sizeof(int32_t)*(n_y + 1));
	}

	~CSR() {
		if (data != nullptr) free(data);
		if (i_y != nullptr) free(i_y);
		if (i_x != nullptr) free(i_x);
	}

	int32_t n_y;
	int32_t n_x;
	int32_t n_nz;
	value_type* data;
	int32_t* i_x;
	int32_t* i_y;
};

