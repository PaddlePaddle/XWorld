#version 330 core

out vec4 FragColor;

in vec2 TexCoords;

uniform float zNear = 0.01;
uniform float zFar = 20.0;
uniform sampler2D tex;

void main()
{
    float z_b = texture(tex, TexCoords).a;
    //z_b = (2 * zNear) / (zFar + zNear - z_b * (zFar - zNear));
    FragColor = vec4(vec3(z_b), 1.0f);
}

