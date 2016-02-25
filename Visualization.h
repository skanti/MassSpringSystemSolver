#ifndef LB_VISUALIZATION_H
#define LB_VISUALIZATION_H

#include "VertexObject.h"
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <ft2build.h>
#include FT_FREETYPE_H
#include "Shader.h"
#include "GLSLProgram.h"
#include <string>
#include <cmath>
#include <vector>

namespace lb {

    const float ASPECT_RATIO_WINDOW = 800.0f / 600.0f;
    glm::mat4 projection_window, projection_text;
    glm::mat4 view_window;
    GLSLProgram render_program = GLSLProgram(),
            polygon_program = GLSLProgram(),
            mass_spring_program = GLSLProgram();

    static FT_Face face;

    class Visualization // singleton
    {
    public: // ctors

        Visualization() {
            load_fonts();
            load_shaders();
            // window
            float zoom = 2.0f;
            projection_window = glm::ortho(-1.0f * zoom, 1.0f * zoom, -6.0f / 8.0f * zoom, 6.0f / 8.0f * zoom, 1.0f,
                                           0.0f);
            projection_text = glm::ortho(-1.0f, 1.0f, -6.0f / 8.0f, 6.0f / 8.0f, 1.0f, 0.0f);
            view_window = glm::lookAt(glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
        }

        void load_fonts() {
            static FT_Library library;
            FT_Error error = FT_Init_FreeType(&library);
            if (error)
                throw std::runtime_error("FT_Init_FreeType failed\n");

            std::string font_path = "/Users/amon/grive/development/springs/fonts/calibri.ttf";
            error = FT_New_Face(library, font_path.c_str(), 0, &face);
            if (error) throw std::runtime_error("Failed to load: " + font_path + "\n");

            // width, height
            error = FT_Set_Pixel_Sizes(face, 16, 16);
            if (error) throw std::runtime_error("FT_Set_Pixel_Sizes failed!\n");
        }

        void load_shaders() {
            Shader texture_vs, texture_fs;
            std::string shader_path = "/Users/amon/grive/development/springs/shaders/";
            texture_vs.LoadShader(shader_path + "TextureVS.glsl", GL_VERTEX_SHADER);
            texture_fs.LoadShader(shader_path + "TextureFS.glsl", GL_FRAGMENT_SHADER);
            std::vector<Shader> shaders_texture{texture_vs, texture_fs};
            render_program.create_program(shaders_texture);

            Shader polygon_vs, polygon_fs;
            polygon_vs.LoadShader(shader_path + "PolygonMonoColorVS.glsl", GL_VERTEX_SHADER);
            polygon_fs.LoadShader(shader_path + "PolygonMonoColorFS.glsl", GL_FRAGMENT_SHADER);
            std::vector<Shader> shaders_polygon{polygon_vs, polygon_fs};
            polygon_program.create_program(shaders_polygon);


            Shader mass_spring_vs, mass_spring_fs;
            mass_spring_vs.LoadShader(shader_path + "MassSpringVS.glsl", GL_VERTEX_SHADER);
            mass_spring_fs.LoadShader(shader_path + "MassSpringFS.glsl", GL_FRAGMENT_SHADER);
            std::vector<Shader> shaders_mass_spring{mass_spring_vs, mass_spring_fs};
            mass_spring_program.create_program(shaders_mass_spring);
        }

        static void add_glyph_to_data(FT_Bitmap *bitmap, FT_Int x, FT_Int y, std::vector<float> color,
                                      VAOTex<unsigned char> *obj) {
            FT_Int x_max = x + bitmap->width;
            FT_Int y_max = y + bitmap->rows;

            for (FT_Int i = y, q = 0; i < y_max; i++, q++)
                for (FT_Int j = x, p = 0; j < x_max; j++, p++)
                    if (bitmap->buffer[q * bitmap->width + p] > 0)
                        obj->data[i * obj->SIZE_X + j] |= bitmap->buffer[q * bitmap->width + p]; // red
        }

        static void clear_screen() {
            glClearColor(0.9, 0.9, 0.9f, 1); // black
            glClear(GL_COLOR_BUFFER_BIT);
        }

        static void init() {
            if (!visualization) visualization = std::unique_ptr<Visualization>(new Visualization());
        }

        static Visualization &get_instance() {
            return *visualization;
        }

    private:
        static std::unique_ptr<Visualization> visualization;
    };

    std::unique_ptr<Visualization> Visualization::visualization;

} // lb


#endif