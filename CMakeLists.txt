cmake_minimum_required(VERSION 3.0)
project(MSN CXX)

find_package(Eigen3 3.3.2 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
MESSAGE( STATUS "EIGEN_DIR:         " ${EIGEN3_INCLUDE_DIR})

set(CMAKE_CXX_COMPILER "c++")

# -DEIGEN_NO_DEBUG
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -fopenmp -std=c++14  -DGLEW_NO_GLU -DEIGEN_NO_DEBUG")

set(SOURCE_FILES_CPP
        src/main.cpp
        src/MSNWorld.cpp
        )

set(SOURCE_FILES_H
        src/MSNWorld.h
        src/Nodes.h
        src/Springs.h
        )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build)


include_directories(${GAMEFRAMEWORK_DIR}/src/cpp)
add_executable(MSNExample ${SOURCE_FILES_CPP} ${SOURCE_FILES_H})

