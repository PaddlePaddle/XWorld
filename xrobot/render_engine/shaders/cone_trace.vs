#version 430

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;

out VS_OUT {
    vec3 FragPos;
    vec3 Normal;
    vec2 TexCoords;
    vec3 Tangent;
    vec3 BiTangent;
} vs_out;

uniform struct Matrices
{
    mat4 scale;
    mat4 state;
    mat4 model;
} matrices;

uniform mat4 projection;
uniform mat4 view;
uniform float flip = 1;


void main()
{
    vs_out.FragPos = vec3(matrices.state * matrices.model * matrices.scale * vec4(aPos, 1.0));
    vs_out.Normal = flip * inverse(transpose(mat3(matrices.state * matrices.model))) * aNormal;
    vs_out.TexCoords = aTexCoords;
    
    gl_Position = projection * view * vec4(vs_out.FragPos, 1.0);
}

