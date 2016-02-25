#version 410 core

layout (location = 0) in vec2 position;

uniform mat4 ModelMatrix;
uniform mat4 ViewMatrix;
uniform mat4 ProjectionMatrix;
uniform vec4 color;

void main()
{
    mat4 MVPatrix = ProjectionMatrix*ViewMatrix*ModelMatrix;
    gl_Position = MVPatrix*vec4(position, 0.0, 1.0);
}


