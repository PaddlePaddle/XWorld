#version 330 core

layout (location = 0) in vec3 aPos;

uniform vec3 scale = vec3(0.05);

void main()
{
	gl_Position = vec4(scale * aPos, 1.0);
}