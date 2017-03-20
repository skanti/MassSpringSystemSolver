cmake_minimum_required(VERSION 3.5)
project(MSN CXX)

# -> required packages
find_package(Freetype REQUIRED)
include_directories(${FREETYPE_INCLUDE_DIRS})

find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
# <-

set(CMAKE_CXX_COMPILER "c++")

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


include_directories(${GAMEFRAMEWORK_DIR}/src/cpp)
link_directories(${GAMEFRAMEWORK_DIR}/build)
add_executable(MSNExample ${SOURCE_FILES_CPP} ${SOURCE_FILES_H})
target_link_libraries(MSNExample GameFramework GLEW glfw GL freetype png)



