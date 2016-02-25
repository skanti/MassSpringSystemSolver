#ifndef LB_SIMULATION_H
#define LB_SIMULATION_H

#include "Shape.h"
#include "Drawable.h"
#include "Visualization.h"
#include <iostream>
#include <list>
#include <memory>

namespace lb {

    class TimingLabel : public Drawable {
    public:
        TimingLabel(float pos_x, float pos_y, sizet width, sizet height)
                : box(pos_x, pos_y, width, height),
                  vao(box.N_VERTICES, box.N_TRIANGLES, width, height, 1) { init_drawable(); }

        void init_drawable() {
            glUseProgram(render_program.get_id());
            glGenVertexArrays(1, &vao.vao_id);
            glBindVertexArray(vao.vao_id);

            vao.vertices = box.vertices;
            vao.model_matrix = glm::translate(vao.model_matrix, glm::vec3(box.pos_x, box.pos_y, 0.0f));
            vao.model_matrix = glm::scale(vao.model_matrix,
                                          glm::vec3(vao.SIZE_X / 800.0f, vao.SIZE_Y / 800.0f, 1.0f));


            glUniformMatrix4fv(render_program.uniform("ModelMatrix"), 1, GL_FALSE,
                               &vao.model_matrix[0][0]);
            glUniformMatrix4fv(render_program.uniform("ViewMatrix"), 1, GL_FALSE, &view_window[0][0]);
            glUniformMatrix4fv(render_program.uniform("ProjectionMatrix"), 1, GL_FALSE, &projection_text[0][0]);


            glGenBuffers(1, &vao.vbo_id);


            glBindBuffer(GL_ARRAY_BUFFER, vao.vbo_id);
            glBufferData(GL_ARRAY_BUFFER, 2 * vao.N_VERTICES * sizeof(GLfloat), vao.vertices.data(), GL_STATIC_DRAW);

            glEnableVertexAttribArray(0);  // Vertex position
            glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, (GLubyte *) NULL);

            GLuint tex_pos_handler;
            glGenBuffers(1, &tex_pos_handler);

            GLfloat tex_pos[] = {
                    // pos
                    0.0f, 0.0f,
                    1.0f, 1.0f,
                    1.0f, 0.0f,
                    0.0f, 0.0f,
                    0.0f, 1.0f,
                    1.0f, 1.0f
            };

            glBindBuffer(GL_ARRAY_BUFFER, tex_pos_handler);
            glBufferData(GL_ARRAY_BUFFER, 12 * sizeof(GLfloat), tex_pos, GL_STATIC_DRAW);

            glEnableVertexAttribArray(1);  // Vertex position
            glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, (GLubyte *) NULL);

            glActiveTexture(GL_TEXTURE0);
            glGenTextures(1, &vao.tex_id);
            glBindTexture(GL_TEXTURE_2D, vao.tex_id);

            glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, vao.SIZE_X, vao.SIZE_Y, 0, GL_RED, GL_UNSIGNED_BYTE,
                         vao.data.data());

            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

            glBindVertexArray(0);
            glBindTexture(GL_TEXTURE_2D, 0);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }

        void populate_data(std::vector<std::string> content) {
            FT_GlyphSlot slot = face->glyph;
            FT_Error error;
            std::fill(vao.data.begin(), vao.data.end(), 0.0f);
            std::vector<float> black{{0.0f, 0.0f, 0.0f}};
            int pen_y = 10, pen_x = 10;
            for (auto &row : content) {
                for (unsigned i = 0; i < row.size(); i++) {
                    error = FT_Load_Char(face, row[i], FT_LOAD_RENDER);
                    if (error) throw std::runtime_error("Failed to load char\n");
                    Visualization::add_glyph_to_data(&slot->bitmap,
                                                     pen_x + slot->bitmap_left,
                                                     pen_y - slot->bitmap_top, black,
                                                     &vao);
                    pen_x += slot->advance.x >> 6;
                }
                pen_x = 10;
                pen_y += 20;
            }

            glBindTexture(GL_TEXTURE_2D, vao.tex_id);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, vao.SIZE_X, vao.SIZE_Y, 0, GL_RED, GL_UNSIGNED_BYTE,
                         vao.data.data());
        }

        void draw() {
            glUseProgram(render_program.get_id());
            glBindVertexArray(vao.vao_id);
            glUniform1i(render_program.uniform("mode"), 1);
            glBindTexture(GL_TEXTURE_2D, vao.tex_id);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, vao.SIZE_X, vao.SIZE_Y, 0, GL_RED, GL_UNSIGNED_BYTE,
                         vao.data.data());
            glDrawArrays(GL_TRIANGLES, 0, vao.N_VERTICES);
            glBindTexture(GL_TEXTURE_2D, 0);
        }

        Rectangle box;
        VAOTex<unsigned char> vao;
    };

    class RectTest : public Drawable {
    public:
        RectTest(float pos_x, float pos_y, float width, float height)
                : box(pos_x, pos_y, width, height),
                  vao(box.N_VERTICES, box.N_TRIANGLES) { init_drawable(); }

        void init_drawable() {
            glUseProgram(polygon_program.get_id());
            glGenVertexArrays(1, &vao.vao_id);
            glBindVertexArray(vao.vao_id);
            vao.vertices = box.vertices;
            vao.color = std::vector<float>({0.2f, 0.8f, 0.2f, 0.5f});
            vao.model_matrix = glm::translate(vao.model_matrix, glm::vec3(box.pos_x, box.pos_y, 0.0f));
            vao.model_matrix = glm::scale(vao.model_matrix, glm::vec3(box.width, box.height, 1.0f));

            glUniformMatrix4fv(polygon_program.uniform("ModelMatrix"), 1, GL_FALSE, &vao.model_matrix[0][0]);
            glUniformMatrix4fv(polygon_program.uniform("ViewMatrix"), 1, GL_FALSE, &view_window[0][0]);
            glUniformMatrix4fv(polygon_program.uniform("ProjectionMatrix"), 1, GL_FALSE, &projection_text[0][0]);

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


        Rectangle box;
        VAOMonoColor vao;
    };

    class World {
    public:
        World()
                : timing_label(0.8f, 0.65f, 120, 40) {
        }

        virtual void advance(unsigned int &iteration_counter, long long int ms_per_frame) = 0;

        void advance_all(unsigned int &iteration_counter, long long int ms_per_frame) {
            timing_label.populate_data({"ms/frame: " + std::to_string(ms_per_frame)});
            advance(iteration_counter, ms_per_frame);
        }

        virtual void draw() = 0;

        void draw_all() {
            timing_label.draw();
            draw();
        }

        TimingLabel timing_label;
    };

}

#endif
