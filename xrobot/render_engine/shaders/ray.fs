#version 330 core

out vec4 FragColor;

in GS_OUT {
	vec4 Color;
} fs_in;

void main()
{
	FragColor = fs_in.Color;
}