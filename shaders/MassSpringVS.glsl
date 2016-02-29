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


vec2 line1[2] = vec2[2](vec2(0,0),vec2(0.5,0.5));

layout( std140 ) uniform springs {
    struct Spring {
        uvec4 ab;
    } spring[N_MAX_SPRINGS];
};


uniform mat4 ViewMatrix;
uniform mat4 ProjectionMatrix;

uniform vec2 shape[N_MAX_SHAPE_VERTICES];
layout (location = 0) in uint node_index;
layout (location = 1) in uint spring_index;
uniform vec4 color;

void main()
{
    mat4 VPMatrix = ProjectionMatrix*ViewMatrix;
    if (mode == MODE_NODES) {
        vec2 pos = shape[gl_VertexID] + node[node_index].x.xy;
        gl_Position = VPMatrix*vec4(pos, 0.0, 1.0);
    } else if (mode == MODE_SPRINGS) {
        vec2 s = node[spring[spring_index].ab[gl_VertexID]].x.xy;
        gl_Position = VPMatrix*vec4(s, 0.0, 1.0);
    }
}


