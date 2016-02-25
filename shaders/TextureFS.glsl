#version 410 core

#define MODE_TEXTURE 0
#define MODE_FONT 1

in vec2 frag_tex_coord;
uniform sampler2D tex;
out vec4 frag_color;

uniform int mode;

void main()
{
    if (mode == MODE_TEXTURE) {
        frag_color = texture(tex, frag_tex_coord);
    } else if (mode == MODE_FONT) {
        float r = texture(tex, frag_tex_coord).r;
        float a = texture(tex, frag_tex_coord).a*r;
        frag_color = vec4(0,0,0,a);
    }
}
