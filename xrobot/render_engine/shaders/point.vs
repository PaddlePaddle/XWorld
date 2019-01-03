#version 330 core

layout (location = 0) in vec3 aPos;

out float depth_linear;

uniform mat4 projection;
uniform mat4 view;
uniform vec3 camera_worldpos;
uniform mat4 camera_direction;
uniform mat4 inv_cubemap_projection;
uniform samplerCube tex;
uniform int dir = 0;
uniform int size = 256;
uniform float zNear = 0.02;
uniform float zFar = 20.0;

const float fov   = 1.5707962;
const float fov_2 = 0.7853982;

float linearize(float depth)
{
    return (2 * zNear) / (zFar + zNear - depth * (zFar - zNear));
}

float visualize(vec3 dir)
{
	float depth_lg = texture(tex, dir).r;
	return depth_lg;
}

vec3 ViewPosFromDepth(float depth, vec2 uv) {
    float z = depth * 2.0 - 1.0;
    vec4 clipSpacePosition = vec4(uv, z, 1.0);
    vec4 viewSpacePosition = inv_cubemap_projection * clipSpacePosition;
    viewSpacePosition /= viewSpacePosition.w;
    viewSpacePosition *= camera_direction;
    return viewSpacePosition.xyz;
}

void main()
{
	int innovation_x = (gl_VertexID / size);
	int innovation_y = (gl_VertexID % size);

	float u = float(innovation_x) / size;
	float v = float(innovation_y) / size;

	vec2 mapCoord = 2.0 * vec2(u, v) - 1.0;

    float phi = fov_2 * mapCoord.x;
    float theta = fov_2 * mapCoord.y;
    float tanfov = tan(fov_2);
    mapCoord.x = tan(phi) / tanfov;
    mapCoord.y = tan(theta) / (cos(phi) * tanfov);

    vec3 point_view;
	float z = 0.0;
	if(dir == 0) {
    	z = visualize(vec3(1, mapCoord.y, mapCoord.x));
        point_view = ViewPosFromDepth(z, mapCoord);
    }
    else if(dir == 1) {
    	z = visualize(vec3(-1.0, mapCoord.y, -mapCoord.x));
        point_view = ViewPosFromDepth(z, mapCoord);
    }
    else if(dir == 5) {
    	z = visualize(vec3(-mapCoord.x, mapCoord.y, 1.0));
        point_view = ViewPosFromDepth(z, mapCoord);
    }
    else if(dir == 4) {
    	z = visualize(vec3(mapCoord.xy, -1.0));
        point_view = ViewPosFromDepth(z, mapCoord);
    }


    vec3 point_world = point_view + camera_worldpos;

    depth_linear = linearize(z);

    if(v < 0.1 || v > 0.9)
        depth_linear = 0;

	gl_Position = projection * view * vec4(point_world, 1.0);
}