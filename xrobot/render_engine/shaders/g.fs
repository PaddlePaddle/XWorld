#version 430 core

layout (location = 0) out vec4 gPosition;
layout (location = 1) out vec4 gNormal;
layout (location = 2) out vec4 gAlbedoSpec;
layout (location = 3) out vec4 gPBR;

in VS_OUT {
    vec3 FragPos;
    vec3 Normal;
    vec2 TexCoords;
    vec3 Tangent;
    vec3 BiTangent;
    vec3 TangentViewPos;
    vec3 TangentFragPos;
} fs_in;

// URDF
uniform vec3 urdf_color = vec3(1,1,1);

// MTL
uniform vec3 kA = vec3(0.0);
uniform vec3 kD = vec3(1,0,0);
uniform vec3 kS = vec3(1,0,0);
uniform float d = 1;
uniform float Ns = 1;
uniform int bump = 0;
uniform int displacement = 0;
uniform int diffuseMap = 0;
uniform int specularMap = 0;
uniform int aoMap = 0;

uniform sampler2D texture_diffuse0;
uniform sampler2D texture_specular0;
uniform sampler2D texture_normal0;
uniform sampler2D texture_height0;
uniform sampler2D texture_ao0;

uniform float heightScale = -0.02f;

// Normal Mapping
vec3 NormalMapping(vec2 Texcoord, mat3 TBN)
{
    vec3 normal = texture(texture_normal0, Texcoord).rgb;
    normal = normalize(normal * 2.0 - vec3(1.0));
    
    return normalize(TBN * normal);
}

// Parallax Occulusion Mapping
vec2 ParallaxMapping(vec2 texCoords, vec3 viewDir)
{
    const float minLayers = 8;
    const float maxLayers = 32;
    float numLayers = mix(maxLayers, minLayers, abs(dot(vec3(0.0, 0.0, 1.0), viewDir)));
    float layerDepth = 1.0 / numLayers;
    float currentLayerDepth = 0.0;
    vec2 P = viewDir.xy / viewDir.z * heightScale;
    vec2 deltaTexCoords = P / numLayers;
    
    vec2  currentTexCoords     = texCoords;
    float currentDepthMapValue = texture(texture_height0, currentTexCoords).r;
    
    while(currentLayerDepth < currentDepthMapValue)
    {
        currentTexCoords -= deltaTexCoords;
        currentDepthMapValue = texture(texture_height0, currentTexCoords).r;
        currentLayerDepth += layerDepth;
    }
    
    vec2 prevTexCoords = currentTexCoords + deltaTexCoords;
    
    float afterDepth  = currentDepthMapValue - currentLayerDepth;
    float beforeDepth = texture(texture_height0, prevTexCoords).r - currentLayerDepth + layerDepth;
    
    float weight = afterDepth / (afterDepth - beforeDepth);
    vec2 finalTexCoords = prevTexCoords * weight + currentTexCoords * (1.0 - weight);
    
    return finalTexCoords;
}

void main()
{   
    vec3 viewDir = normalize(fs_in.TangentViewPos - fs_in.TangentFragPos);
    mat3 TBN = mat3(fs_in.Tangent, fs_in.BiTangent, fs_in.Normal);

    vec2 texcoords_ = fs_in.TexCoords;

    float height = 0;
    if(displacement == 1)
    {
        texcoords_ = ParallaxMapping(texcoords_, viewDir);
        height = texture(texture_height0, fs_in.TexCoords).r;
    }

    if(bump == 1)
    {
        vec3 normal = texture(texture_normal0, texcoords_).rgb;
        normal = normalize(normal * 2.0 - vec3(1.0));
        gNormal = vec4(NormalMapping(texcoords_, TBN), 1.0f);
    }
    else
    {
        gNormal = vec4(fs_in.Normal, 1.0f);
    }

    vec3 diffuse = kD;
    float alpha = 0;
    if(diffuseMap == 1)
    {
        vec4 color = texture(texture_diffuse0, texcoords_);
        diffuse = color.rgb;
        alpha = color.a;

        if(alpha < 0.01)
        {
            discard;
        }
    }

    float roughness = 1.0f - clamp(Ns / 100.0f, 0.0f, 1.0f);
    if(specularMap == 1)
    {
        roughness = texture(texture_specular0, texcoords_).r;
    }

    float ao = 1;
    if(aoMap == 1)
    {
        ao = texture(texture_ao0, texcoords_).r;
    }

    gNormal.a = 1.0f;
    gPosition = vec4(fs_in.FragPos, gl_FragCoord.z);
    gAlbedoSpec.rgb = diffuse.rgb;
    gAlbedoSpec.a = Ns;
    gPBR.r = 0.0f;
    gPBR.g = roughness;
    gPBR.b = height;
    gPBR.a = ao;
} 