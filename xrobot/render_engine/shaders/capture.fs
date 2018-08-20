#version 330 core

out vec4 FragColor;
in vec2 TexCoords;

in VS_OUT {
    vec3 FragPos;
    vec3 Normal;
    vec2 TexCoords;
} fs_in;

uniform sampler2D texture_diffuse0;

uniform vec3 kA = vec3(0.0);
uniform vec3 kD = vec3(1,0,0);
uniform vec3 kS = vec3(1,0,0);
uniform float d = 1;
uniform float Ns = 1;
uniform int diffuseMap = 0;
uniform vec3 light_directional = vec3(1, 2, 1);

float lum(vec3 color) { return dot(color, vec3(1)); }

float max3(vec3 color) { return max(color.r, max(color.y, color.z)); }


void main()
{
    // vec4 diffuse_tex = texture(texture_diffuse0, fs_in.TexCoords).rgba;

    // vec3 diffuse;
    // float alpha;

    // vec3 N = normalize(fs_in.Normal);
    // vec3 L = normalize(light_directional);

    // if(diffuseMap > 0)
    // {
    //     diffuse = kD * diffuse_tex.rgb;
    // }
    // else
    // {
    //     diffuse = kD;
    // }

    // alpha = min(diffuse_tex.a, d);

    // if(lum(diffuse) < 0.05 && alpha < 0.05)
    // {
    //     discard;
    // }

    // // Lambert
    // float lambert = max(0, dot(N, L));

    // // Ambient
    // vec3 ambient = vec3(0.4) + kA * 0.5;

    // // Shading
    // vec3 outColor = lambert * diffuse + ambient * diffuse;

    // FragColor = vec4(outColor, 1);
}
