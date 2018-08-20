#version 330 core

layout (location = 0) in vec3 aPos;

out VS_OUT {
	bool first;
} vs_out;

void main()
{
	vs_out.first = (gl_VertexID == 0);
	gl_Position = vec4(aPos, 1.0);
}