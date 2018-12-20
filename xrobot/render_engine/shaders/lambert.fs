#version 330 core

layout (location = 0) out vec3 FragColor;

in VS_OUT {
    vec3 FragPos;
    vec3 Normal;
    vec2 TexCoords;
} fs_in;

uniform sampler2D texture_diffuse0;


// URDF
uniform vec3 urdf_color = vec3(1,1,1);

// MTL
uniform vec3 kA = vec3(0.0);
uniform vec3 kD = vec3(0,0,0);
uniform vec3 kS = vec3(1,0,0);
uniform float d = 1;
uniform float Ns = 1;
uniform int diffuseMap = 0;

// Light
uniform vec3 light_directional = -vec3(1, 2, 3);


float lum(vec3 color) { return dot(color, vec3(1)); }

float max3(vec3 color) { return max(color.r, max(color.y, color.z)); }

void main()
{

    vec4 diffuse_tex = texture(texture_diffuse0, fs_in.TexCoords).rgba;

    vec3 diffuse;
    float alpha;

    if(diffuseMap > 0)
    {
        diffuse = kD * diffuse_tex.rgb;
    }
    else
    {
        diffuse = kD * urdf_color;
    }

    // vec3 diffuse = urdf_color;
    // float alpha = 0;
    // vec4 tex_color;

    // if(diffuseMap == 1)
    // {
    //     tex_color = texture(texture_diffuse0, fs_in.TexCoords);
    //     diffuse = tex_color.rgb;
    //     alpha = tex_color.a;
    
    //     if(lum(kD) > 0.005)
    //     {
    //         diffuse = kD * tex_color.rgb;
    //     }
    // }
    // else
    // {
    //     if(max3(kD) > 0.005)
    //     {
    //         diffuse = kD;
    //     }
    // }

    alpha = min(diffuse_tex.a, d);

    if(lum(diffuse) < 0.05 && alpha < 0.05) {
        discard;
    }

    vec3 N = normalize(fs_in.Normal);
    vec3 L = normalize(light_directional);

    // Lambert
    float lambert = max(0, dot(N, L));

    // Shading
    vec3 outColor = lambert * diffuse + 0.45 * diffuse;
    FragColor.rgb = outColor;
}

