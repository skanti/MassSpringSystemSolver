#ifndef LB_SHADER_H
#define LB_SHADER_H

#include <GL/glew.h>
#include <string>
#include <fstream>
#include <vector>
#include <iostream>

namespace lb {

    class Shader {
    public:
        Shader() {
            shaderID = 0;
        }

        GLuint LoadShader(std::string shaderFile, GLenum type) {
            std::ifstream in(shaderFile);
            std::string src = "";
            std::string line = "";
            while (std::getline(in, line))
                src += line + "\n";
            //std::cout << src;
            //GLint compiled;

            shaderID = glCreateShader(type);

            const char *source = src.c_str();
            glShaderSource(shaderID, 1, &source, NULL);
            glCompileShader(shaderID);

            if (!shaderID) {
                std::cerr << "Could not compile the shader";
                return 0;
            }

            GLint status;
            glGetShaderiv(shaderID, GL_COMPILE_STATUS, &status);
            if (status == GL_FALSE) {
                std::string msg("Compile failure in shader:\n");

                GLint infoLogLength;
                glGetShaderiv(shaderID, GL_INFO_LOG_LENGTH, &infoLogLength);
                char *strInfoLog = new char[infoLogLength + 1];
                glGetShaderInfoLog(shaderID, infoLogLength, NULL, strInfoLog);
                msg += strInfoLog;
                delete[] strInfoLog;

                glDeleteShader(shaderID);
                shaderID = 0;
                throw std::runtime_error(msg);
            }
            return shaderID;
        }

        GLuint ReturnShaderID() const {
            return shaderID;
        }

    private:
        GLuint shaderID;
    };


}

#endif // LB_SHADER_HPP_INCLUDED
