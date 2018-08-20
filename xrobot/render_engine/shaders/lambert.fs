#version 330 core

layout (location = 0) out vec4 FragColor;
layout (location = 1) out vec4 LabelAndDepth;

in VS_OUT {
    vec3 FragPos;
    vec3 Normal;
    vec2 TexCoords;
} fs_in;

uniform sampler2D texture_diffuse1;
uniform samplerCube cmap;

uniform float translucent = 1.0;

// Color Picking
uniform vec3 id_color = vec3(1,0,0);
uniform float zNear = 0.01;
uniform float zFar = 10.0;

// MTL
uniform vec3 kA = vec3(0.0);
uniform vec3 kD = vec3(1,0,0);
uniform vec3 kS = vec3(1,0,0);
uniform float d = 1;
uniform float Ns = 1;

// Light
uniform float exposure = 0.0;
uniform vec3 light_directional = vec3(1, -2, 2);
uniform vec3 camPos = vec3(1,1,1);


float lum(vec3 color) { return dot(color, vec3(1)); }

float max3(vec3 color) { return max(color.r, max(color.y, color.z)); }

float linearize(float depth)
{
    return (2 * zNear) / (zFar + zNear - depth * (zFar - zNear));
}

void main()
{

    vec4 diffuse_tex = texture(texture_diffuse1, fs_in.TexCoords).rgba;

    vec3 diffuse;
    float alpha;

    alpha = min(diffuse_tex.a, d);

    if(lum(kD) <= 0.01)
        diffuse = diffuse_tex.rgb;
    else if(lum(diffuse_tex.rgb) > 0.01)
        diffuse = kD * diffuse_tex.rgb;
    else
    {
        diffuse = kD;
    }


    if(lum(diffuse) < 0.05 && alpha < 0.05)
    {
        discard;
    }


    vec3 I = normalize(fs_in.FragPos - camPos);
    vec3 R = reflect(I, normalize(fs_in.Normal));
    vec3 N = normalize(fs_in.Normal);
    vec3 L = normalize(light_directional);
    vec3 H = normalize(L + I);

    // Reflection
    float ref = 0;
    if(alpha < 0.85)
        ref = lum(kS) * 0.1;
    if(lum(kS) > 1.5)
        ref = lum(kS) * 0.03;
    vec3 reflection = texture(cmap, R).rgb;
    if(Ns < 0.01) ref = 0;

    // Blinn-Phong
    float NdotH = dot(N, H);
    float specular = pow(clamp(NdotH, 0, 1), Ns) * 0.6;
    if(Ns < 0.01) specular = 0;

    // Lambert
    float lambert = max(0, dot(N, L));

    // Ambient
    vec3 ambient = vec3(0.2) + kA * 0.5;

    // Shading
    vec3 outColor = lambert * diffuse + ambient * diffuse + specular * kS + reflection * ref;

    // Exposure
    outColor = clamp(outColor * pow(2.0, exposure), vec3(0), vec3(1));

    FragColor.rgb = outColor;
    //FragColor.rgb = 0.5 + 0.5 * fs_in.Normal;
    FragColor.a = 1;
    //FragColor.a = 1;

    // if(translucent < 1)
    // {
    //    FragColor.rgb = vec3(1,1,1); 
    //    FragColor.a = 0.5;
    // }

    float linear_depth = linearize(gl_FragCoord.z);
    // if(linear_depth * (zFar - zNear) < 0.5)
    // {
    //     float temp = clamp(linear_depth * (zFar - zNear), 0, 0.5) * 20.0;
    //     FragColor.a = 0.2;
    //     FragColor.a = smoothstep( 0.0, alpha, temp);
    // }

    if(linear_depth * (zFar - zNear) < 2.0f)
        LabelAndDepth = vec4(id_color, 1);
    else
        LabelAndDepth = vec4(1,1,1,1);
}

