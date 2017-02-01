#version 410 core

#define MODE_NODES 0
#define MODE_SPRINGS 1

const int n_nodes_max = 10000;

layout (location = 0 ) in vec3 vertex;
layout (location = 1 ) in vec3 pos;
layout (location = 2 ) in vec3 color;

out vec3 nodes_color_vs;

uniform float l_base;

uniform int mode;
uniform mat4 ModelMatrix;
uniform mat4 ViewMatrix;
uniform mat4 ProjectionMatrix;

void main() {
    mat4 MVPMatrix = ProjectionMatrix*ViewMatrix*ModelMatrix;
    if (mode == MODE_NODES) {
        gl_Position = MVPMatrix*vec4(vertex*l_base + pos, 1.0);
    } else if (mode == MODE_SPRINGS) {
        gl_Position = MVPMatrix*vec4(pos, 1.0);
    }
    nodes_color_vs = color;
}


