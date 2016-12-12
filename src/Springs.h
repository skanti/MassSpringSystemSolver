#pragma once

#include <cstdlib>
#include "Common.h"

template<typename value_type, int alignment = 16>
struct Springs {

    void init(int32_t n_nodes, int32_t n_springs) {
        n_size = n_springs;
        A.init(n_nodes, n_springs, 2*n_springs);
        AA.init(n_nodes, n_nodes, 2*n_springs + n_nodes);
    }
    
    int32_t n_size;
    CSR<value_type, alignment> A;
    CSR<value_type, alignment> AA;
};

