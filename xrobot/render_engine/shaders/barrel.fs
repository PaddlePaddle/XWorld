#version 330 core
out vec4 FragColor;
in vec2 TexCoords;
uniform sampler2D tex;
const float PI = 3.1415926535;
uniform float BarrelPower0 = 0.8f;
uniform float BarrelPower1 = 0.8f;
vec2 Distort(vec2 p)
{
    float theta  = atan(p.y, p.x);
    float radius = length(p);
    p.x = pow(radius, BarrelPower0) * cos(theta);
    p.y = pow(radius, BarrelPower1) * sin(theta);
    return 0.5 * (p + 1.0);
}

void main()
{
  vec2 xy = 2.0 * TexCoords.xy - 1.0;
  vec2 uv;
  float d = length(xy);
  if (d < 1.0)
  {
    uv = Distort(xy);
  }
  else
  {
    uv = TexCoords.xy;
  }
  vec4 c = texture(tex, uv);
  FragColor = c;
}