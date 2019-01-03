#version 330 core

out vec3 FragColor;

in float depth_linear;

void main()
{
	if(depth_linear < 0.8 && depth_linear > 0.001) 
		FragColor = mix(vec3(1,1,0), vec3(1,0,0), depth_linear);
	else
		discard;
}