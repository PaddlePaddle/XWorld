#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;

out VS_OUT {
    vec3 FragPos;
    vec3 Normal;
    vec2 TexCoords;
} vs_out;

uniform mat4 projection;
uniform mat4 view;

uniform float flip = 1;
uniform mat4 model = mat4(1);
uniform mat4 scale = mat4(1);
uniform mat4 state = mat4(1);
uniform mat4 local_scale = mat4(1);

uniform mat4 con = mat4(
	1, 0, 0, 0,
	0, 0,-1, 0,
	0, 1, 0, 0, 
	0, 0, 0, 1 
);

void main()
{
    vs_out.FragPos = vec3(state * model * scale * vec4(aPos, 1.0));
    vs_out.Normal = flip * inverse(transpose(mat3(state * model))) * aNormal;
    vs_out.TexCoords = aTexCoords;
    
    gl_Position = projection * view * vec4(vs_out.FragPos, 1.0);
}

