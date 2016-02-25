#version 410

#define MODE_NODES 0
#define MODE_SPRINGS 1

uniform int mode;
uniform vec4 color;
out vec4 frag_color;

void main()
{
    if (mode == MODE_NODES) {
        frag_color = color;
    } else if (mode == MODE_SPRINGS) {
        frag_color = vec4(0.8,0.2,0.2,0.8);
    }
}
