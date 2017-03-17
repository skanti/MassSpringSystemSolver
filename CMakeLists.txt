cmake_minimum_required(VERSION 3.5)
project(MSN CXX)

set(CMAKE_CXX_COMPILER c++)

# -DEIGEN_NO_DEBUG
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -fopenmp -std=c++14  -DGLEW_NO_GLU")

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

set(GAMEFRAMEWORK_DIR /home/amon/grive/development/GameFramework)
set(FREETYPE_DIR /usr/include/freetype2)

set(MKL_DIR /opt/intel/compilers_and_libraries/linux/mkl)

include_directories(${GAMEFRAMEWORK_DIR}/src/cpp ${FREETYPE_DIR})
link_directories(${GAMEFRAMEWORK_DIR}/build)
add_executable(MSNExample ${SOURCE_FILES_CPP} ${SOURCE_FILES_H})
target_link_libraries(MSNExample GameFramework GLEW glfw GL freetype png)



