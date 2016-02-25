#version 410 core

in vec2 frag_tex_coord;
uniform sampler2D tex;
out vec4 frag_color;
uniform bool mono;

void main()
{
    if (mono) {
        float r = texture(tex, frag_tex_coord).r;
        float a = texture(tex, frag_tex_coord).a*r;
        frag_color = vec4(0,0,0,a);
    } else {
        frag_color = texture(tex, frag_tex_coord);
    }
}