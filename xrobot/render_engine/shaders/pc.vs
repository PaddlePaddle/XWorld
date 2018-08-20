#version 330 core

out VS_OUT {
	vec3 color;
} vs_out;

void main()
{
	vs_out.color = vec3(0.5);
	gl_Position = vec4(0, 0, 0, 1);
	gl_PointSize = 5.0f;
}