#version 330 core

layout (points) in;
layout (triangle_strip, max_vertices = 8) out;

in VS_OUT {
    vec3 FragPos;
    vec3 Normal;
    vec2 TexCoords;
} gs_in;

out GS_OUT {
    vec3 FragPos;
    vec3 Normal;
    vec2 TexCoords;
} gs_out;

void main()
{
    
}