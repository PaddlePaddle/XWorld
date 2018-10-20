#version 330 core

layout (location = 0) in vec3 VS_IN_Position;

uniform struct Matrices
{
    mat4 scale;
    mat4 state;
    mat4 model;
} matrices;

uniform float radius = 0.0f;
uniform mat4 view;
uniform mat4 projection;

void main()
{
	vec3 dilated_position = (1 + radius) * VS_IN_Position;

	vec4 world_position  = matrices.state * matrices.model * matrices.scale * 
		vec4(dilated_position, 1.0);

	gl_Position = projection * view * world_position;
}