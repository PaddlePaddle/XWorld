#version 330 core
out vec3 FragColor;

in vec2 TexCoords;

uniform sampler2D ssao;
uniform sampler2D src;

void main() 
{ 
    FragColor = texture(src, TexCoords).rgb * texture(ssao, TexCoords).r;
}  