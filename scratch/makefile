MKL_DIR=/opt/intel/compilers_and_libraries/linux/mkl

all : ./build/hashmap ./build/pardiso ./build/eigen_sparse

./build/hashmap : hashmap.cpp
	c++ -O3 -std=c++14 $< -o $@

./build/pardiso : pardiso.cpp
	c++ -Wall -O3 -std=c++14 -I../ -I../src -I$(MKL_DIR)/include -L$(MKL_DIR)/lib/intel64 \
	$< -o $@ \
	-Wl,--start-group ${MKL_DIR}/lib/intel64_lin/libmkl_intel_lp64.a ${MKL_DIR}/lib/intel64_lin/libmkl_core.a ${MKL_DIR}/lib/intel64_lin/libmkl_sequential.a -Wl,--end-group \
    -lm -ldl -m64 -lmkl_rt -pthread
    

./build/eigen_sparse: eigen_sparse.cpp
	c++ -O3 -std=c++14 -I../ -I../src $< -o $@
