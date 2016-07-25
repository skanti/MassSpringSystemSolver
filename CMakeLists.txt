cmake_minimum_required(VERSION 3.5)
project(MSN CXX)

set(CMAKE_CXX_COMPILER "/usr/local/bin/c++-6")

set(OPTIMIZATION_FLAGS "-O3 -fomit-frame-pointer -ffast-math -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fopenmp -std=c++14 -framework OpenGL")

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

set(GAMEFRAMEWORK_DIR /Users/amon/grive/development/GameFramework)
set(FREETYPE_DIR /usr/local/Cellar/freetype/2.6.5)
set(ARMADILLO_DIR /usr/local/Cellar/armadillo/7.200.2)

include_directories(${GAMEFRAMEWORK_DIR}/src/cpp ${FREETYPE_DIR}/include/freetype2 ${ARMADILLO_DIR}/include)
link_directories(${GAMEFRAMEWORK_DIR}/build ${ARMADILLO_DIR}/lib)
add_executable(MSNExample ${SOURCE_FILES_CPP} ${SOURCE_FILES_H})
target_link_libraries(MSNExample GameFramework Glew glfw3 freetype armadillo png)



