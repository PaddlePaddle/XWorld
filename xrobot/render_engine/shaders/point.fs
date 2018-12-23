#version 330 core

out vec3 FragColor;

in float depth_linear;

void main()
{
	if(depth_linear < 0.8 && depth_linear > 0.001) 
		FragColor = vec3(1,1,0);
	else
		discard;
}