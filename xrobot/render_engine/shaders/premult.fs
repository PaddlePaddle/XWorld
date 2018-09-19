#version 330 core

out vec4 FragColor;

in mediump vec2 TexCoords;

uniform mediump float zNear = 0.02;
uniform mediump float zFar = 30.0;
uniform lowp float deferred = 0;

uniform sampler2D tex;
uniform sampler2D dep;
uniform sampler2D mask;

float linearize(float depth)
{
  return (2 * zNear) / (zFar + zNear - depth * (zFar - zNear));
}

void main()
{

  FragColor.rgb = texture(tex, TexCoords).rgb;

  if(deferred > 0)
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

