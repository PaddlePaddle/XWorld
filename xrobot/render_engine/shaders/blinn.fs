#version 330 core

out vec4 FragColor;
in vec2 TexCoords;

struct DirectionalLight {
    vec3 ambient;
    vec3 specular;
    vec3 diffuse;
    vec3 direction;
};

uniform sampler2D gPosition;
uniform sampler2D gNormal;
uniform sampler2D gAlbedo;
uniform sampler2D shadowMap[8];

uniform mat4 invView;
uniform mat4 projection;
uniform vec3 cameraPosition;

uniform float exposure = 0.0f;
uniform DirectionalLight light;

uniform float use_shadow;
uniform vec4 direction;
uniform vec4 options;
uniform int num_cascades;
uniform float bias_scale;
uniform float bias_clamp;
uniform float far_bounds[8];
uniform mat4 texture_matrices[8];

float shadow_occlussion(float frag_depth, vec3 n, vec3 l, vec3 x)
{
    int index = 0;
    
    for (int i = 0; i < num_cascades - 1; i++) {
        if (frag_depth > far_bounds[i])
            index = i + 1;
    }

    vec4 light_space_pos = texture_matrices[index] * vec4(x, 1.0f);
    float current_depth = light_space_pos.z;
    
    float bias = clamp(bias_scale * (1.0 - dot(n, l)), 0.0, bias_clamp);  
    float shadow = 0.0;
    vec2 texelSize = 1.0 / textureSize(shadowMap[index], 0).xy;

    for(int x = -1; x <= 1; ++x) {
        for(int y = -1; y <= 1; ++y) {
            float pcfDepth = texture(shadowMap[index], vec2(light_space_pos.xy + vec2(x, y) * texelSize)).r; 
            shadow += current_depth - bias > pcfDepth ? 1.0 : 0.0;        
        }    
    }

    shadow /= 9.0;
    return shadow;
}

vec3 BRDF(DirectionalLight light, vec3 N, vec3 X, vec3 ka, float r)
{
    vec3 L = normalize(light.direction);
    vec3 V = normalize(cameraPosition - X);
    vec3 H = normalize(V + L);

    float dotNL = max(dot(N, L), 0.0f);
    float dotNH = max(dot(N, H), 0.0f);
    float dotLH = max(dot(L, H), 0.0f);
    float spec = exp2(100.0f * clamp(1 - r, 0, 1) + 1.0f);
    vec3 fresnel = vec3(0.04f) + vec3(0.96f) * pow(1.0f - dotLH, 5.0f);
    float blinnPhong = pow(dotNH, spec);
    blinnPhong *= spec * 0.0397f + 0.3183f;
    vec3 specular = vec3(0.96f) * light.specular * blinnPhong * fresnel;
    vec3 diffuse = ka.rgb * light.diffuse;
    return (diffuse + specular) * dotNL;
}

void main()
{
    vec4 g0 = texture(gPosition, TexCoords);
    vec4 g1 = texture(gNormal, TexCoords);
    vec4 g2 = texture(gAlbedo, TexCoords);

    vec3 view_position  = g0.xyz;
    vec3 world_position = vec3(invView * vec4(view_position, 1.0));
    vec3 world_normal   = -normalize(g1.xyz);
    vec3 albedo         = pow(g2.rgb, vec3(2.2f));
    float roughness     = g0.w;
    float height        = g2.w;
    float ao            = g1.w;

    vec4 proj_coord = projection * vec4(view_position, 1.0);
    float depth = (proj_coord.z / proj_coord.w) * 0.5f + 0.5f;

    float shadow = 1.0f;
    if(use_shadow > 0) {
        shadow = 1 - shadow_occlussion(depth, world_normal, direction.xyz, world_position);
    }

    vec3 color = albedo;

    if(depth < 1) {
        color = BRDF(light, world_normal, world_position, albedo, roughness) * shadow;
        color += albedo * light.ambient;
        color = color * pow(2.0f, exposure);
    }

    FragColor = vec4(color, 1);
}

