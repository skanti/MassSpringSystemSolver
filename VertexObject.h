#ifndef LB_VERTEXOBJECT_H
#define LB_VERTEXOBJECT_H

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <vector>
#include "Types.h"
#include "Shape.h"
#include <iostream>

namespace lb {

    struct VAOBase {
        GLuint vao_id, vbo_id;
        glm::mat4 model_matrix;
        GLsizei N_VERTICES, N_TRIANGLES;
        std::vector<GLfloat> vertices;

        VAOBase(size_t N_VERTICES, sizet N_TRIANGLES)
                : vao_id(0), vbo_id(0),
                  model_matrix(glm::mat4(1.0f)),
                  N_VERTICES((GLsizei) N_VERTICES),
                  N_TRIANGLES((GLsizei) N_TRIANGLES),
                  vertices(N_VERTICES * 2) {
        };
    };

    template<typename T>
    struct VAOTex : public VAOBase {
        GLuint tex_id;
        GLsizei SIZE_X, SIZE_Y, N_CHANNEL;
        std::vector<T> data;

        VAOTex(sizet N_VERTICES, sizet N_TRIANGLES, sizet SIZE_X, sizet SIZE_Y, sizet N_CHANNEL)
                : VAOBase(N_VERTICES, N_TRIANGLES), tex_id(0), SIZE_X((GLsizei) SIZE_X), SIZE_Y((GLsizei) SIZE_Y),
                  N_CHANNEL((GLsizei) N_CHANNEL), data(N_CHANNEL * SIZE_X * SIZE_Y) { }
    };

    struct VAOMonoColor : public VAOBase {
        std::vector<float> color;

        VAOMonoColor(sizet N_VERTICES, sizet N_TRIANGLES)
                : VAOBase(N_VERTICES, N_TRIANGLES), color({0.0f, 0.0f, 0.0f, 1.0f}) {
        }
    };

    struct VAOMassSpring {
        GLuint vao_id, vbo_node, vbo_springs;
    };

}

#endif //LB_VERTEXOBJECT_H
