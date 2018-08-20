#version 330 core
layout (points) in;
layout (points, max_vertices = 1) out; 

in VS_OUT {
	vec3 color;
} gs_in[1];

out GS_OUT {
	vec3 color;
} gs_out;

uniform float zNear = 0.01;
uniform float zFar = 20.0;
uniform samplerCube tex;
uniform int dir = 0;

uniform mat4 projection;
uniform mat4 view;
uniform vec3 eye;

const float fov = 3.14159  * 90.0 / 360.0;

float linearize(float depth)
{
    return (2 * zNear) / (zFar + zNear - depth * (zFar - zNear));
}

vec4 visualize(vec3 dir, float phi, float theta)
{
	float depth_lg = texture(tex, normalize(dir)).r;
	float depth_linear = linearize(depth_lg);
	float distance = depth_linear / abs(cos(phi) * cos(theta));
	return vec4(vec3(distance), 1.0f);
}

void main()
{
	float depth_lg = texture(tex, normalize(dir)).r;
	float depth_linear = linearize(depth_lg);
	
}