#version 430 core

layout (location = 0) in vec3 VS_IN_Position;
layout (location = 1) in vec2 VS_IN_TexCoord;
layout (location = 2) in vec3 VS_IN_Normal;
layout (location = 3) in vec3 VS_IN_Tangent;
layout (location = 4) in vec3 VS_IN_Bitangent;

uniform struct Matrices
{
    mat4 scale;
    mat4 state;
    mat4 model;
} matrices;

uniform mat4 view;
uniform mat4 projection;
uniform mat4 crop;

void main()
{
	gl_Position = crop * matrices.state * matrices.model * matrices.scale * vec4(VS_IN_Position, 1.0);
}
