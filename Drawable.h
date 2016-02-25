#ifndef SPRINGS_DRAWABLE_H
#define SPRINGS_DRAWABLE_H

#include "VertexObject.h"
#include "GLSLProgram.h"
#include "Visualization.h"

namespace lb {

    class Drawable {
    public:
        virtual void init_drawable() = 0;

        virtual void draw() = 0;

    };

    template<class T>
    class SolidBody : public Drawable {
    public:
        SolidBody(T shape, VAOMonoColor vao)
                : shape(shape), vao(vao) { }

        void init_drawable() {
            glUseProgram(polygon_program.get_id());
            glGenVertexArrays(1, &vao.vao_id);
            glBindVertexArray(vao.vao_id);

            vao.model_matrix = glm::translate(vao.model_matrix, glm::vec3(shape.pos_x, shape.pos_y, 0.0f));
            vao.model_matrix = glm::scale(vao.model_matrix, glm::vec3(0.1f, 0.1f, 1.0f));

            glUniformMatrix4fv(polygon_program.uniform("ModelMatrix"), 1, GL_FALSE, &vao.model_matrix[0][0]);
            glUniformMatrix4fv(polygon_program.uniform("ViewMatrix"), 1, GL_FALSE, &view_window[0][0]);
            glUniformMatrix4fv(polygon_program.uniform("ProjectionMatrix"), 1, GL_FALSE, &projection_window[0][0]);

            glGenBuffers(1, &vao.vbo_id);
            glBindBuffer(GL_ARRAY_BUFFER, vao.vbo_id);
            glBufferData(GL_ARRAY_BUFFER, vao.vertices.size() * sizeof(GLfloat), vao.vertices.data(), GL_STATIC_DRAW);
            glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, (GLubyte *) NULL);
            glEnableVertexAttribArray(0);  // Vertex position

            glBindVertexArray(0);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            glUseProgram(0);

        }

        void draw() {
            glUseProgram(polygon_program.get_id());
            glBindVertexArray(vao.vao_id);
            glUniform4fv(polygon_program.uniform("color"), 1, vao.color.data());
            glDrawArrays(GL_TRIANGLES, 0, vao.N_TRIANGLES * 3);
            glUseProgram(0);
        }

        T shape;
        VAOMonoColor vao;
    };

}

#endif //SPRINGS_DRAWABLE_H
