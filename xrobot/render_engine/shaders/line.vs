#version 330 core

layout (location = 0) in vec3 aPos;
// layout (location = 1) in vec3 aNormal;
// layout (location = 2) in vec2 aTexCoords;

uniform mat4 projection;
uniform mat4 view;
uniform vec3 aabbMin;
uniform vec3 aabbMax;

void main()
{

	vec3 aabb = aabbMax - aabbMin;

	mat3 aabbScale = mat3(
		aabb.x, 0, 0,
		0, aabb.y, 0,
		0, 0, aabb.z
	);

	vec3 Pos01 = 0.5 * aPos + vec3(0.5);
	vec3 Pos = aabbScale * Pos01 + aabbMin;

	gl_Position = projection * view * vec4(Pos, 1.0);
}