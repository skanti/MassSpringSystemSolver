cmake_minimum_required(VERSION 3.5)
project(MSN CXX)

set(CMAKE_CXX_COMPILER c++)

set(OPTIMIZATION_FLAGS "-O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fopenmp -std=c++14  -DGLEW_NO_GLU")

set(SOURCE_FILES_CPP
        src/main.cpp
        src/MSN2DWorld.cpp
        src/Solver.cpp
        )

set(SOURCE_FILES_H
        src/MSN2DWorld.h
        src/Nodes.h
        src/Springs.h
        src/Solver.h
        )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build)

set(GAMEFRAMEWORK_DIR /home/amon/grive/development/GameFramework)
set(FREETYPE_DIR /usr/include/freetype2)

set(MKL_DIR /opt/intel/compilers_and_libraries/linux/mkl)

include_directories(${GAMEFRAMEWORK_DIR}/src/cpp ${FREETYPE_DIR} ${MKL_DIR}/include)
link_directories(${GAMEFRAMEWORK_DIR}/build ${MKL_DIR}/lib/intel64_lin/ ${MKL_DIR}/lib/intel64)
add_executable(MSNExample ${SOURCE_FILES_CPP} ${SOURCE_FILES_H})
target_link_libraries(MSNExample GameFramework GLEW glfw GL freetype png mkl_intel_lp64 mkl_core mkl_sequential m dl)



