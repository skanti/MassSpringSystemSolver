#version 410 core

#define MODE_NODES 0
#define MODE_SPRINGS 1

const int N_MAX_NODES = 50;
const int N_MAX_SPRINGS = 50;
const int N_MAX_SHAPE_VERTICES = 75;

uniform int mode;

// 16 byte allignment because of std140 layout style
layout( std140 ) uniform nodes {
    struct Node {
        vec4 x;
    } node[N_MAX_NODES];
};

layout( std140 ) uniform springs {
    struct Spring {
        vec2 s[2];
    } spring[N_MAX_SPRINGS];
};

vec2 line1[2] = vec2[2](vec2(0,0),vec2(0.5,0.5));

uniform mat4 ViewMatrix;
uniform mat4 ProjectionMatrix;

uniform vec2 shape[N_MAX_SHAPE_VERTICES];
layout (location = 0) in int node_index;
layout (location = 1) in int springs_index;
uniform vec4 color;

void main()
{
    mat4 VPMatrix = ProjectionMatrix*ViewMatrix;
    if (mode == MODE_NODES) {
        vec2 pos = shape[gl_VertexID] + node[node_index].x.xy;
        gl_Position = VPMatrix*vec4(pos, 0.0, 1.0);
    } else if (mode == MODE_SPRINGS) {
        gl_Position = VPMatrix*vec4(line1[gl_VertexID], 0.0, 1.0);
    }
}


