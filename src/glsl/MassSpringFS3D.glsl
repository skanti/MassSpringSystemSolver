#version 410

#define MODE_NODES 0
#define MODE_SPRINGS 1

uniform int mode;

uniform vec4 springs_color;
uniform float nodes_color_alpha;
in vec3 nodes_color_vs;

out vec4 frag_color;

void main()
{
    if (mode == MODE_NODES) {
        frag_color = vec4(nodes_color_vs, nodes_color_alpha);
    } else if (mode == MODE_SPRINGS) {
        frag_color = springs_color;
    }
}
