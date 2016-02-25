#version 410 core

layout (location = 0) in vec2 position;
layout (location=1) in vec2 tex_coord;

uniform mat4 ModelMatrix;
uniform mat4 ViewMatrix;
uniform mat4 ProjectionMatrix;

out vec2 frag_tex_coord;

void main()
{
    frag_tex_coord = tex_coord;
    mat4 MVPatrix = ProjectionMatrix*ViewMatrix*ModelMatrix;
    gl_Position = MVPatrix*vec4(position, 0.0, 1.0);
}


