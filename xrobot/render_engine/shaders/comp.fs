#version 330 core
out vec3 FragColor;

in vec2 TexCoords;

uniform sampler2D ssao;
uniform sampler2D src;

void main() 
{ 
    FragColor = texture(ssao, TexCoords).r * vec3(1) * texture(src, TexCoords).rgb;
}  