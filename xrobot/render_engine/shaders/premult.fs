#version 330 core

out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D tex;
uniform sampler2D dep;

uniform float zNear = 0.01;
uniform float zFar = 10.0;

float linearize(float depth)
{
    return (2 * zNear) / (zFar + zNear - depth * (zFar - zNear));
}

void main()
{
    FragColor = vec4(
    	vec3(texture(tex, TexCoords).rgb * texture(tex, TexCoords).a),
    	linearize(texture(dep, TexCoords).a)
    );

    if(texture(tex, TexCoords).a < 0.99)
		FragColor.rgb = vec3(0.5);
}

