#version 330 core

layout (location = 0) in vec3 aPos;

uniform struct Matrices
{
    mat4 scale;
    mat4 state;
    mat4 model;
} matrices;

void main()
{
    gl_Position = matrices.state * matrices.model * matrices.scale * vec4(aPos, 1.0);
}

