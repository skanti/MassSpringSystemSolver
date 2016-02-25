#version 410 core

layout( location = 0 ) out vec4 FragColor;

void main()
{
    float alpha = 0.0;
    vec2 uv = vec2(gl_FragCoord.x, gl_FragCoord.y);
    float dist =  sqrt(dot(uv, uv));
    if (dist < 500)
        alpha = 1.0f;
    FragColor = vec4(0,1,0,alpha);
}