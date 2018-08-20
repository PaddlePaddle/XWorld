#version 430

out Vertex
{
    vec3 texCoord;
    vec3 normal;
};

uniform struct Matrices
{
	mat4 scale;
	mat4 state;
    mat4 model;
} matrices;

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec3 vertexNormal;
layout(location = 2) in vec2 vertexTexCoord;

void main()
{

	mat4 M = matrices.state * matrices.model * matrices.scale; 
	mat3 M3 = inverse(transpose(mat3(matrices.state * matrices.model)));

    gl_Position = M * vec4(vertexPosition, 1.0f);

    normal = normalize(M3 * vertexNormal);
    texCoord = vec3(vertexTexCoord, 0);
}