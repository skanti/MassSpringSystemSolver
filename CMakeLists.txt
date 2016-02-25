cmake_minimum_required(VERSION 3.3)
project(springs CXX)


set(CMAKE_CXX_COMPILER
        "/usr/local/bin/c++-5")

find_package(Boost
        1.59
        REQUIRED             # Fail with error if Boost is not found
        )

find_package(freetype
        REQUIRED)

SET(OPTIMIZATION_FLAGS "-O3 -fomit-frame-pointer -march=native")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OPTIMIZATION_FLAGS} -Wall -fopenmp -std=c++11 -framework OpenGL")


set(SOURCE_FILES
        shaders/TextureVS.glsl
        shaders/TextureFS.glsl
        shaders/PolygonMonoColorVS.glsl
        shaders/PolygonMonoColorFS.glsl
        shaders/MassSpringFS.glsl
        shaders/MassSpringVS.glsl
        CMakeLists.txt
        Engine.h
        GLSLProgram.h
        Shader.h
        World.h
        Types.h
        Visualization.h
        WindowManager.h
        VertexObject.h
        Shape.h Drawable.h SolidObject.h)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")
#set_target_properties(springs PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")

set(FREETYPE_DIR /usr/local/Cellar/freetype/2.6_1/include/freetype2)
set(BOX2D_DIR /Users/amon/grive/development/Box2D/Box2D/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${FREETYPE_DIR} ${BOX2D_DIR} ${BOX2D_DIR}/Box2D)
add_executable(springs test/test.cpp ${SOURCE_FILES})
LINK_DIRECTORIES(${BOX2D_DIR}/Build)
target_link_libraries(springs Glew glfw3 freetype)


