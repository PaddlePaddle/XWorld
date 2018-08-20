#version 330 core

out vec4 FragColor;

uniform vec3 color = vec3(1,1,0);

void main()
{
	FragColor = vec4(color,0.5);
}