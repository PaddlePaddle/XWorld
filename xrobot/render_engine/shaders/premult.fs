#version 330 core

out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D tex;
uniform sampler2D dep;
uniform sampler2D mask;

uniform float zNear = 0.02;
uniform float zFar = 30.0;
uniform int deferred = 0;

float linearize(float depth)
{
  return (2 * zNear) / (zFar + zNear - depth * (zFar - zNear));
}

void main()
{

  FragColor.rgb = texture(tex, TexCoords).rgb;

  if(deferred == 1)
  {
    FragColor.a = linearize(texture(dep, TexCoords).a);
    if(texture(mask, TexCoords).a < 0.5) {
     FragColor.rgb = vec3(0.5f);
     FragColor.a = 1.0f;
    }
  }
  else
  {
    FragColor.a = linearize(texture(dep, TexCoords).r);
  }
}

