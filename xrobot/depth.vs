#version 330 core

layout (location = 0) in vec3 VS_IN_Position;

uniform struct Matrices
{
    mat4 scale;
    mat4 state;
    mat4 model;
} matrices;

uniform mat4 view;
uniform mat4 projection;

void main()
{
	gl_Position = projection * view * matrices.state * matrices.model * matrices.scale * vec4(VS_IN_Position, 1.0);
}