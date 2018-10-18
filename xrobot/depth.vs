#version 330 core

layout (location = 0) in vec3 VS_IN_Position;
layout (location = 1) in vec3 VS_IN_Normal;

uniform struct Matrices
{
    mat4 scale;
    mat4 state;
    mat4 model;
} matrices;

uniform mediump float radius = 0.0f;
uniform mat4 view;
uniform mat4 projection;

void main()
{
	vec4 world_position  = matrices.state * matrices.model * matrices.scale * 
		vec4(VS_IN_Position, 1.0);

	vec3 dilate_position = normalize(VS_IN_Normal) * radius + vec3(world_position);

	gl_Position = projection * view * vec4(dilate_position, 1.0);
}