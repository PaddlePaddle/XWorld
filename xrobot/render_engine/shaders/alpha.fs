#version 330 core

out vec4 FragColor;

in vec2 TexCoords;

uniform lowp float channel;

uniform sampler2D tex;

void main()
{
	if(channel == 0) {
		FragColor = vec4(vec3(texture(tex, TexCoords).r), 1);
	} else if(channel == 1) {
		FragColor = vec4(vec3(texture(tex, TexCoords).g), 1);
	} else if(channel == 2) {
		FragColor = vec4(vec3(texture(tex, TexCoords).b), 1);
	} else {
		FragColor = vec4(vec3(texture(tex, TexCoords).a), 1);
	}
}