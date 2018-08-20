#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;
layout (location = 3) in vec3 aTangent;
layout (location = 4) in vec3 aBiTangent;

out VS_OUT {
    vec3 FragPos;
    vec3 Normal;
    vec2 TexCoords;
    vec3 Tangent;
    vec3 BiTangent;
    vec3 TangentViewPos;
    vec3 TangentFragPos;
} vs_out;

uniform struct Matrices
{
    mat4 scale;
    mat4 state;
    mat4 model;
} matrices;

uniform mat4 projection;
uniform mat4 view;
uniform vec3 viewPos;
uniform float flip = 1;

void main()
{
    mat3 n = flip * inverse(transpose(mat3(matrices.state * matrices.model)));

    vs_out.FragPos = vec3(matrices.state * matrices.model * matrices.scale * vec4(aPos, 1.0));
    vs_out.Normal = normalize(n * aNormal);
    vs_out.Tangent = normalize(n * aTangent);
    vs_out.BiTangent = normalize(n * aBiTangent);
    vs_out.TexCoords = aTexCoords;

    mat3 TBN =  mat3(
         vs_out.Tangent.x, vs_out.BiTangent.x, vs_out.Normal.x,
         vs_out.Tangent.y, vs_out.BiTangent.y, vs_out.Normal.y,
         vs_out.Tangent.z, vs_out.BiTangent.z, vs_out.Normal.z
    );

    vs_out.TangentViewPos = TBN * viewPos;
    vs_out.TangentFragPos = TBN * vs_out.FragPos;
    gl_Position = projection * view * vec4(vs_out.FragPos, 1.0);
}

