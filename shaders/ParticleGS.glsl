#version 410 core


layout( points ) in;
layout( triangle_strip, max_vertices = 4 ) out;

uniform mat4 ModelMatrix;
uniform mat4 ViewMatrix;
uniform mat4 ProjectionMatrix;

void main() {
    float a = 0.05;
    float z = 0.0;
    mat4 s = ModelMatrix*ViewMatrix*ProjectionMatrix;
    mat4 MVPMatrix = ProjectionMatrix*ViewMatrix*ModelMatrix;
    gl_Position = MVPMatrix*vec4(-a,-a,z,1.0);    EmitVertex();
    gl_Position = MVPMatrix*vec4(a,-a, z,1.0);    EmitVertex();
    gl_Position = MVPMatrix*vec4(-a,a, z,1.0);    EmitVertex();
    gl_Position = MVPMatrix*vec4(a,a,  z,1.0);    EmitVertex();
    EndPrimitive();
}
