cmake_minimum_required(VERSION 3.0)
project(MassSpringWorld)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# -> REQUIRED PACKAGES
find_package(GameFramework REQUIRED)
include_directories(${GameFramework_INCLUDE_DIRS}) 

find_package(Freetype REQUIRED)
include_directories(${FREETYPE_INCLUDE_DIRS})

find_package(PNG REQUIRED)
include_directories(${PNG_INCLUDE_DIRS})

find_package(JPEG REQUIRED)
include_directories(${JPEG_INCLUDE_DIRS})

find_package(glfw3 3.2 REQUIRED)
include_directories(${glfw3_INCLUDE_DIRS})

find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})

find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIRS})

find_package(glm REQUIRED)
include_directories(${GLM_INCLUDE_DIRS})

find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
# <- 

set(CMAKE_CXX_COMPILER c++)

# -DEIGEN_NO_DEBUG
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -fopenmp -std=c++14 -DGLEW_NO_GLU -DGLM_ENABLE_EXPERIMENTAL=1")

set(SOURCE_FILES_CPP
        src/main.cpp
        src/MSNWorld.cpp
        )

set(SOURCE_FILES_H
        src/MSNWorld.h
        src/Springs.h 
		src/Nodes.h
        )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build)

set(MSN_SRC_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src")
configure_file (
  "${CMAKE_CURRENT_SOURCE_DIR}/src/Common.h.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/Common.h"
)


add_executable(msn_example ${SOURCE_FILES_CPP} ${SOURCE_FILES_H})
target_link_libraries(msn_example gameframework GLEW glfw GL png freetype)

