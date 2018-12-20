#version 330 core

out vec3 FragColor; // Low Dynamics Range
in vec2 TexCoords;

uniform lowp float shading;
uniform lowp float use_ao;
uniform lowp float use_ref;

uniform sampler2D lighting;
uniform sampler2D ssao;
uniform sampler2D ssr;

void main()
{
	float ao  = texture(ssao, TexCoords).r;
	vec3 ref  = texture(ssr, TexCoords).rgb;
	vec3 base = texture(lighting, TexCoords).rgb;
    
    FragColor = mix(base, base * ao, use_ao) + ref * use_ref; 

    // Gamma Correction
    FragColor = mix(FragColor, pow(FragColor, vec3(1.0 / 2.2)), shading);
}
