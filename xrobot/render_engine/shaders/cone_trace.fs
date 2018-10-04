#version 430

#pragma optionNV(unroll all)

layout (location = 0) out vec4 fragColor;
layout (location = 1) out vec4 LabelAndDepth;

layout(binding = 0) uniform sampler3D voxelVisibility;
layout(binding = 1) uniform sampler3D voxelTex;
layout(binding = 2) uniform samplerCube irradianceMap; // PBR
layout(binding = 3) uniform samplerCube prefilterMap;
layout(binding = 4) uniform sampler2D brdfLUT;
layout(binding = 5) uniform sampler2D gAlbedoSpec; // G-Buffer
layout(binding = 6) uniform sampler2D gNormal;
layout(binding = 7) uniform sampler2D gPosition;
layout(binding = 8) uniform sampler2D gPBR;
layout(binding = 9) uniform sampler3D voxelTexMipmap[6];
layout(binding = 15) uniform sampler2D s_ShadowMap[8];

noperspective in vec2 TexCoords;

const float PI = 3.14159265f;
const float HALF_PI = 1.57079f;
const float EPSILON = 1e-30;
const float SQRT_3 = 1.73205080f;
const uint MAX_DIRECTIONAL_LIGHTS = 1;

// Directional Light
struct Light {
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    vec3 direction;
};

uniform vec3 id_color = vec3(1,0,0);
uniform int mode = 0;
uniform int shadowMode = 0;
uniform float exposure = 1.2;
uniform vec3 cameraPosition;
uniform int numDirectionalLight;
uniform Light directionalLight[MAX_DIRECTIONAL_LIGHTS];
uniform float voxelScale;
uniform vec3 worldMinPoint;
uniform vec3 worldMaxPoint;
uniform int volumeDimension;
uniform float maxTracingDistanceGlobal = 0.5f;
uniform float bounceStrength = 0.5f;
uniform float aoFalloff = 900.0f;
uniform float aoAlpha = 0.01f;
uniform float samplingFactor = 0.4f;
uniform float coneShadowTolerance = 0.1f;
uniform float coneShadowAperture = 0.01f;
uniform float zNear = 0.02f;
uniform float zFar = 25.0f;
uniform vec4 direction;
uniform vec4 options;
uniform int num_cascades;
uniform float far_bounds[8];
uniform mat4 texture_matrices[8];

uniform float bias_scale = 0.05f;
uniform float bias_clamp = 0.0002f;
uniform float ibl_factor = 0.3f;

const vec3 diffuseConeDirections[] =
{
    vec3(0.0f, 1.0f, 0.0f),
    vec3(0.0f, 0.5f, 0.866025f),
    vec3(0.823639f, 0.5f, 0.267617f),
    vec3(0.509037f, 0.5f, -0.7006629f),
    vec3(-0.50937f, 0.5f, -0.7006629f),
    vec3(-0.823639f, 0.5f, 0.267617f)
};

const float diffuseConeWeights[] =
{
    PI / 4.0f,
    3.0f * PI / 20.0f,
    3.0f * PI / 20.0f,
    3.0f * PI / 20.0f,
    3.0f * PI / 20.0f,
    3.0f * PI / 20.0f,
};

float depth_compare(float a, float b, float bias)
{
    return a - bias > b ? 1.0 : 0.0;
}
float shadow_occlussion(float frag_depth, vec3 n, vec3 l, vec3 x)
{
    int index = 0;
    float blend = 0.0;
    
    for (int i = 0; i < num_cascades - 1; i++)
    {
        if (frag_depth > far_bounds[i])
            index = i + 1;
    }

    blend = clamp( (frag_depth - far_bounds[index] * 0.995) * 200.0, 0.0, 1.0);
    
    // Apply blend options.
    blend *= options.z;
    // Transform frag position into Light-space.
    vec4 light_space_pos = texture_matrices[index] * vec4(x, 1.0f);
    float current_depth = light_space_pos.z;
    
    float bias = clamp(bias_scale * (1.0 - dot(n, l)), 0.0, bias_clamp);  
    float shadow = 0.0;
    vec2 texelSize = 1.0 / textureSize(s_ShadowMap[index], 0).xy;
    for(int x = -1; x <= 1; ++x)
    {
        for(int y = -1; y <= 1; ++y)
        {
            float pcfDepth = texture(s_ShadowMap[index], vec2(light_space_pos.xy + vec2(x, y) * texelSize)).r; 
            shadow += current_depth - bias > pcfDepth ? 1.0 : 0.0;        
        }    
    }
    shadow /= 9.0;

    if (options.x == 1.0)
    {
        //if (blend > 0.0 && index != num_cascades - 1)
        //{
        //    light_space_pos = texture_matrices[index + 1] * vec4(PS_IN_WorldFragPos, 1.0f);
        //    shadow_map_depth = texture(s_ShadowMap, vec3(light_space_pos.xy, float(index + 1))).r;
        //    current_depth = light_space_pos.z;
        //    float next_shadow = depth_compare(current_depth, shadow_map_depth, bias);
        //    
        //    return (1.0 - blend) * shadow + blend * next_shadow;
        //}
        //else
            return shadow;
    }
    else
        return 0.0;
}

vec3 debug_cascade(float frag_depth)
{
    int index = 0;

    // Find shadow cascade.
    for (int i = 0; i < num_cascades - 1; i++)
    {
        if (frag_depth > far_bounds[i])
            index = i + 1;
    }
    if (index == 0)
        return vec3(1.0, 0.0, 0.0);
    else if (index == 1)
        return vec3(0.0, 1.0, 0.0);
    else if (index == 2)
        return vec3(0.0, 0.0, 1.0);
    else
        return vec3(1.0, 1.0, 0.0);
}

float linearize(float depth)
{
    return (2 * zNear) / (zFar + zNear - depth * (zFar - zNear));
}

vec3 WorldToVoxel(vec3 position)
{
    vec3 voxelPos = position - worldMinPoint;
    return voxelPos * voxelScale;
}

vec4 AnistropicSample(vec3 coord, vec3 weight, uvec3 face, float lod)
{
    // anisotropic volumes level
    float anisoLevel = max(lod - 1.0f, 0.0f);
    // directional sample
    vec4 anisoSample = weight.x * textureLod(voxelTexMipmap[face.x], coord, anisoLevel)
                     + weight.y * textureLod(voxelTexMipmap[face.y], coord, anisoLevel)
                     + weight.z * textureLod(voxelTexMipmap[face.z], coord, anisoLevel);
    // linearly interpolate on base level
    if(lod < 1.0f)
    {
        vec4 baseColor = texture(voxelTex, coord);
        anisoSample = mix(baseColor, anisoSample, clamp(lod, 0.0f, 1.0f));
    }

    return anisoSample;                    
}

bool IntersectRayWithWorldAABB(vec3 ro, vec3 rd, out float enter, out float leave)
{
    vec3 tempMin = (worldMinPoint - ro) / rd; 
    vec3 tempMax = (worldMaxPoint - ro) / rd;
    
    vec3 v3Max = max (tempMax, tempMin);
    vec3 v3Min = min (tempMax, tempMin);
    
    leave = min (v3Max.x, min (v3Max.y, v3Max.z));
    enter = max (max (v3Min.x, 0.0), max (v3Min.y, v3Min.z));    
    
    return leave > enter;
}

vec4 TraceCone(vec3 position, vec3 normal, vec3 direction, float aperture, bool traceOcclusion)
{
    uvec3 visibleFace;
    visibleFace.x = (direction.x < 0.0) ? 0 : 1;
    visibleFace.y = (direction.y < 0.0) ? 2 : 3;
    visibleFace.z = (direction.z < 0.0) ? 4 : 5;
    traceOcclusion = traceOcclusion && aoAlpha < 1.0f;
    // world space grid voxel size
    float voxelWorldSize = 2.0 /  (voxelScale * volumeDimension);
    // weight per axis for aniso sampling
    vec3 weight = direction * direction;
    // move further to avoid self collision
    float dst = voxelWorldSize;
    vec3 startPosition = position + normal * dst;
    // final results
    vec4 coneSample = vec4(0.0f);
    float occlusion = 0.0f;
    float maxDistance = maxTracingDistanceGlobal * (1.0f / voxelScale);
    float falloff = 0.5f * aoFalloff * voxelScale;
    // out of boundaries check
    float enter = 0.0; float leave = 0.0;

    if(!IntersectRayWithWorldAABB(position, direction, enter, leave))
    {
        coneSample.a = 1.0f;
    }

    while(coneSample.a < 1.0f && dst <= maxDistance)
    {
        vec3 conePosition = startPosition + direction * dst;
        // cone expansion and respective mip level based on diameter
        float diameter = 2.0f * aperture * dst;
        float mipLevel = log2(diameter / voxelWorldSize);
        // convert position to texture coord
        vec3 coord = WorldToVoxel(conePosition);
        // get directional sample from anisotropic representation
        vec4 anisoSample = AnistropicSample(coord, weight, visibleFace, mipLevel);
        // front to back composition
        coneSample += (1.0f - coneSample.a) * anisoSample;
        // ambient occlusion
        if(traceOcclusion && occlusion < 1.0)
        {
            occlusion += ((1.0f - occlusion) * anisoSample.a) / (1.0f + falloff * diameter);
        }
        // move further into volume
        dst += diameter * samplingFactor;
    }

    return vec4(coneSample.rgb, occlusion);
}

float TraceShadowCone(vec3 position, vec3 direction, float aperture, float maxTracingDistance) 
{
    bool hardShadows = false;

    if(coneShadowTolerance == 1.0f) { hardShadows = true; }

    // directional dominat axis
    uvec3 visibleFace;
    visibleFace.x = (direction.x < 0.0) ? 0 : 1;
    visibleFace.y = (direction.y < 0.0) ? 2 : 3;
    visibleFace.z = (direction.z < 0.0) ? 4 : 5;
    // world space grid size
    float voxelWorldSize = 1.0 /  (voxelScale * volumeDimension);
    // weight per axis for aniso sampling
    vec3 weight = direction * direction;
    // move further to avoid self collision
    float dst = voxelWorldSize;
    vec3 startPosition = position + direction * dst;
    // control vars
    float mipMaxLevel = log2(volumeDimension) - 1.0f;
    // final results
    float visibility = 0.0f;
    float k = exp2(7.0f * coneShadowTolerance);
    // cone will only trace the needed distance
    float maxDistance = maxTracingDistance;
    // out of boundaries check
    float enter = 0.0; float leave = 0.0;

    if(!IntersectRayWithWorldAABB(position, direction, enter, leave))
    {
        visibility = 1.0f;
    }
    
    while(visibility < 1.0f && dst <= maxDistance)
    {
        vec3 conePosition = startPosition + direction * dst;
        float diameter = 2.0f * aperture * dst;
        float mipLevel = log2(diameter / voxelWorldSize);
        // convert position to texture coord
        vec3 coord = WorldToVoxel(conePosition);
        // get directional sample from anisotropic representation
        vec4 anisoSample = AnistropicSample(coord, weight, visibleFace, mipLevel);

        // hard shadows exit as soon cone hits something
        if(hardShadows && anisoSample.a > EPSILON) { return 0.0f; }  
        // accumulate
        visibility += (1.0f - visibility) * anisoSample.a * k;
        // move further into volume
        dst += diameter * samplingFactor;
    }

    return 1.0f - visibility;
}


vec3 Ambient(Light light, vec3 albedo)
{
    return max(albedo * light.ambient, 0.0f);
}


// Microfacet Model for PBR
float DistributionGGX(vec3 N, vec3 H, float roughness)
{
    float a = roughness*roughness;
    float a2 = a*a;
    float NdotH = max(dot(N, H), 0.0);
    float NdotH2 = NdotH*NdotH;
    
    float nom   = a2;
    float denom = (NdotH2 * (a2 - 1.0) + 1.0);
    denom = PI * denom * denom;
    
    return nom / denom;
}
float GeometrySchlickGGX(float NdotV, float roughness)
{
    float r = (roughness + 1.0);
    float k = (r*r) / 8.0;
    
    float nom   = NdotV;
    float denom = NdotV * (1.0 - k) + k;
    
    return nom / denom;
}

float GeometrySmith(vec3 N, vec3 V, vec3 L, float roughness)
{
    float NdotV = max(dot(N, V), 0.0);
    float NdotL = max(dot(N, L), 0.0);
    float ggx2 = GeometrySchlickGGX(NdotV, roughness);
    float ggx1 = GeometrySchlickGGX(NdotL, roughness);
    
    return ggx1 * ggx2;
}

// Disney's Model
vec3 fresnelSchlick(float cosTheta, vec3 F0)
{
    return F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
}

vec3 fresnelSchlickRoughness(float cosTheta, vec3 F0, float roughness)
{
    return F0 + (max(vec3(1.0 - roughness), F0) - F0) * pow(1.0 - cosTheta, 5.0);
}

// UE4's Adapted Model
//
// Real Shading in Unreal Engine 4
// https://cdn2.unrealengine.com/Resources/files/2013SiggraphPresentationsNotes-26915738.pdf

vec3 fresnelSchlickUE4(float cosTheta, vec3 F0, float roughness)
{
    float p = (-5.55473 * cosTheta - 6.98316) * cosTheta;
    return F0 + (vec3(1.0) - F0) * pow(2.0, p);
}

vec3 fresnelSchlickRoughnessUE4(float cosTheta, vec3 F0, float roughness)
{
    float p = (-5.55473 * cosTheta - 6.98316) * cosTheta;
    return F0 + (max(vec3(1.0 - roughness), F0) - F0) * pow(2.0, p);
}

// EA Frostbite
// Specular Occulision
float specOcculision(float NdotV, float AO, float roughness)
{
    return clamp(
        pow(NdotV + AO, exp2(-16.0f * roughness - 1.0f)) - 1.0f + AO,
        0.0f,
        1.0f
    );
}

vec3 CookTorranceBRDF(
    Light light,
    vec3 N,
    vec3 X,
    vec3 albedo,
    float metallic, float roughness, float ao)
{
    vec3 F0 = vec3(0.04);
    F0 = mix(F0, albedo, metallic);

    vec3 Lo = vec3(0.0);
    vec3 L = light.direction;
    vec3 V = normalize(cameraPosition - X);
    vec3 H = normalize(V + L);
    vec3 R = reflect(-V, N);  
    vec3 radiance = light.diffuse;

    // BRDF
    float NDF = DistributionGGX(N, H, roughness);
    float G   = GeometrySmith(N, V, L, roughness);
    vec3 F    = fresnelSchlick(max(dot(H, V), 0.0), F0);

    vec3 nominator    = NDF * G * F;
    float denominator = 4 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0) + 0.001;
    vec3 specular = nominator / denominator;
    vec3 kS = F;
    vec3 kD = vec3(1.0) - kS;
    kD *= 1.0 - metallic;
    
    float NdotL = max(dot(N, L), 0.0);
    Lo = (kD * albedo / PI + specular) * radiance * NdotL;

    vec3 color = Lo * ao;
    return color;
}

vec3 DistanceIBL(
    vec3 N,
    vec3 X,
    vec3 albedo,
    float metallic,
    float roughness,
    float ao
)
{
    vec3 V = normalize(cameraPosition - X);
    vec3 R = reflect(-V, N);
    vec3 F0 = vec3(0.04);
    F0 = mix(F0, albedo, metallic);

    const float MAX_REFLECTION_LOD = 4.0;

    vec3 F = fresnelSchlickRoughness(max(dot(N, V), 0.0), F0, roughness);
    vec3 kS = F;
    vec3 kD = vec3(1.0) - kS;
    kD *= vec3(1.0 - metallic);


    vec3 irradiance = texture(irradianceMap, N).rgb;
    vec3 diffuse      = irradiance * albedo;
    vec2 brdf  = texture(brdfLUT, vec2(max(dot(N, V), 0.0), roughness)).rg;
    vec3 prefilteredColor = textureLod(prefilterMap, R,  roughness * MAX_REFLECTION_LOD).rgb;
    vec3 spec = prefilteredColor * (F * brdf.x + brdf.y);
    vec3 ambient = (kD * diffuse + spec) * ao;

    return ambient;
}

vec3 BRDF(Light light, vec3 N, vec3 X, vec3 ka, vec4 ks)
{
    // common variables
    vec3 L = normalize(light.direction);

    vec3 V = normalize(cameraPosition - X);
    vec3 H = normalize(V + L);
    // compute dot procuts
    float dotNL = max(dot(N, L), 0.0f);
    float dotNH = max(dot(N, H), 0.0f);
    float dotLH = max(dot(L, H), 0.0f);
    // decode specular power
    float spec = exp2(11.0f * ks.a + 1.0f);
    // emulate fresnel effect
    vec3 fresnel = ks.rgb + (1.0f - ks.rgb) * pow(1.0f - dotLH, 5.0f);
    // specular factor
    float blinnPhong = pow(dotNH, spec);
    // energy conservation, aprox normalization factor
    blinnPhong *= spec * 0.0397f + 0.3183f;
    // specular term
    vec3 specular = ks.rgb * light.specular * blinnPhong * fresnel;
    // diffuse term
    vec3 diffuse = ka.rgb * light.diffuse;
    // return composition
    return (diffuse + specular) * dotNL;
}

float CaculateDirectionalShadow(vec3 normal, vec3 position, float depth)
{
    float visibility = 1.0f;

    if(shadowMode == 0)
    {
        visibility = 1.02 - shadow_occlussion(depth, normal, direction.xyz, position);

    }
    else if(shadowMode == 1 && numDirectionalLight > 0)
    {
        Light light = directionalLight[0];
        visibility = max(0.0f, TraceShadowCone(position, light.direction, coneShadowAperture, 1.0f / voxelScale));
    }else if(shadowMode == 3)
    {
        vec3 voxelPos = WorldToVoxel(position);  
        visibility = max(0.0f, texture(voxelVisibility, voxelPos).a);
    }else
    {
        visibility = 1.0f;
    }
    
    if(visibility <= 0.0f) return 0.0f;  

    return visibility;
}

vec3 CalculateDirectional(Light light, vec3 normal, vec3 position, vec3 albedo, vec4 specular,
    float depth, float roughness, float metallic, float ao)
{
    vec3 brdf = CookTorranceBRDF(light, normal, position, albedo, metallic, roughness, ao);
    return brdf;
}

vec3 CalculateDirectLighting(vec3 position, vec3 normal, vec3 albedo, vec4 specular, float depth,
    float roughness, float metallic, float ao)
{
    // calculate directional lighting
    vec3 directLighting = vec3(0.0f);

    // calculate lighting for directional light

    for (int i = 0; i < numDirectionalLight; ++i)
    {
        Light light = directionalLight[i];

        directLighting += CalculateDirectional(light, normal, position, 
            albedo, specular, depth, roughness, metallic, ao);
        directLighting += Ambient(light, albedo);        
    }


    return directLighting;
}

vec4 CalculateIndirectLighting(vec3 position, vec3 normal, vec3 albedo, vec4 specular, float roughness,
    float metallic, bool ambientOcclusion, float visibility)
{
    vec4 specularTrace = vec4(0.0f);
    vec4 diffuseTrace = vec4(0.0f);
    vec3 coneDirection = vec3(0.0f);

    if(any(greaterThan(specular.rgb, specularTrace.rgb)))
    {
        vec3 viewDirection = normalize(cameraPosition - position);
        vec3 coneDirection = reflect(-viewDirection, normal);
        coneDirection = normalize(coneDirection);
        float aperture = clamp(tan(HALF_PI * (1.0f - specular.a)), 0.0174533f, PI);
        specularTrace = TraceCone(position, normal, coneDirection, aperture, false);
        specularTrace.rgb *= specular.rgb;
    }

    if(any(greaterThan(albedo, diffuseTrace.rgb)))
    {
        const float aperture = 0.57735f;
        vec3 guide = vec3(0.0f, 1.0f, 0.0f);

        if (abs(dot(normal,guide)) == 1.0f)
        {
            guide = vec3(0.0f, 0.0f, 1.0f);
        }

        vec3 right = normalize(guide - dot(normal, guide) * normal);
        vec3 up = cross(right, normal);

        for(int i = 0; i < 6; i++)
        {
            coneDirection = normal;
            coneDirection += diffuseConeDirections[i].x * right + diffuseConeDirections[i].z * up;
            coneDirection = normalize(coneDirection);
            diffuseTrace += TraceCone(position, normal, coneDirection, aperture, ambientOcclusion) * diffuseConeWeights[i];
        }

        diffuseTrace.rgb *= albedo;
    }

    vec3 result = bounceStrength * (diffuseTrace.rgb + specularTrace.rgb);

    float ao = clamp((1.0f - diffuseTrace.a + aoAlpha), 0.08f, 1.0f);

    ao = pow(ao, 2.0f);

    // Distanct IBL Specular Occulision
    vec3 V = normalize(cameraPosition - position);
    float spec_occ = specOcculision(dot(normal, V), ao, roughness);

    float spec_gi_alpha = 1 - clamp(dot(specularTrace.rgb, vec3(0.34)), 0, 1);
    float diff_gi_alpha = 1 - clamp(dot(diffuseTrace.rgb, vec3(0.44)), 0, 1);
    float gi_alpha = spec_gi_alpha * diff_gi_alpha;

    float visibility_alpha = clamp(visibility * 4, 0, 1);

    vec3 distance_ibl = DistanceIBL(normal, position, albedo, metallic, roughness, ao);
    result = result * (1.0 - ibl_factor) + vec3(spec_occ * gi_alpha * visibility_alpha) * distance_ibl * ibl_factor;
    return vec4(result, ambientOcclusion ? ao : 1.0f);
}

void main()
{
    if(texture(gNormal, TexCoords).a == 0) {
        discard;
    }

    // world-space position
    vec3 position = texture(gPosition, TexCoords).rgb;
    // Depth (Log)
    float depth = texture(gPosition, TexCoords).a; 
    // world-space normal
    vec3 normal = -normalize(texture(gNormal, TexCoords).rgb);
    
    float metallic = texture(gPBR, TexCoords).r;
    float roughness = texture(gPBR, TexCoords).g;
    float height = texture(gPBR, TexCoords).b;
    float ao = texture(gPBR, TexCoords).a;
    vec4 specular = vec4(1,1,1, clamp(1.0 - roughness,0,1));
    vec3 baseColor = texture(gAlbedoSpec, TexCoords).rgb;

    vec3 albedo = pow(baseColor, vec3(2.2f));
    vec3 emissive = vec3(0,0,0);

    float visibility = 1.0f;
    vec3 directLighting = vec3(1.0f);
    vec4 indirectLighting = vec4(1.0f);
    vec3 compositeLighting = vec3(1.0f);

    // if(texture(gNormal, TexCoords).a < 1)
    //     discard;

    // if(linearize(depth) > 0.6)
    //     mode = 5;

    if(mode == 0)   // direct + indirect + ao
    {
        visibility = CaculateDirectionalShadow(normal, position, depth);
        indirectLighting = CalculateIndirectLighting(position, normal, baseColor, specular, 
            roughness, metallic, true, visibility);
        directLighting = CalculateDirectLighting(position, normal, albedo, specular, depth,
            roughness, metallic, ao) * visibility;
    }
    // else if(mode == 1)  // direct + indirect
    // {
    //     visibility = CaculateDirectionalShadow(normal, position, depth);
    //     indirectLighting = CalculateIndirectLighting(position, normal, baseColor, specular, 
    //         roughness, metallic, false, visibility);
    //     directLighting = CalculateDirectLighting(position, normal, albedo, specular, depth,
    //         roughness, metallic, ao) * visibility + albedo * 0.15;
    // }
    // else if(mode == 2) // direct only
    // {
    //     visibility = CaculateDirectionalShadow(normal, position, depth);
    //     indirectLighting = vec4(0.0f, 0.0f, 0.0f, 1.0f);
    //     directLighting = CalculateDirectLighting(position, normal, albedo, specular, depth,
    //         roughness, metallic, ao) * visibility;
    // }
    // else if(mode == 3) // indirect only
    // {
    //     visibility = CaculateDirectionalShadow(normal, position, depth);
    //     directLighting = vec3(0.0f);
    //     baseColor.rgb = specular.rgb = vec3(1.0f);
    //     indirectLighting = CalculateIndirectLighting(position, normal, baseColor, specular,
    //         roughness, metallic, false, visibility);
    // }
    else if(mode == 10) // ambient occlusion only
    {
        directLighting = vec3(0.0f);
        specular = vec4(0.0f);
        indirectLighting = CalculateIndirectLighting(position, normal, baseColor, specular,
            roughness, metallic, true, 1.0f);
        indirectLighting.rgb = vec3(1.0f);
    }
    else if(mode == 5)
    {
        visibility = CaculateDirectionalShadow(normal, position, depth);
        indirectLighting = vec4(0.0f, 0.0f, 0.0f, 1.0f);
        directLighting = BRDF(directionalLight[0], normal, position, albedo, specular) *
                visibility + albedo * 0.2f;
    }
    else if(mode == 6)
    {
        visibility = CaculateDirectionalShadow(normal, position, depth);
        indirectLighting = vec4(0.0f, 0.0f, 0.0f, 1.0f);
        directLighting = CookTorranceBRDF(directionalLight[0], normal, position, albedo,
                metallic, roughness, ao) + albedo * 0.2f;
    }

    // visibility = CaculateDirectionalShadow(normal, position, depth);
    //     directLighting = vec3(0.0f);
    //     baseColor.rgb = specular.rgb = vec3(1.0f);
    //     indirectLighting = CalculateIndirectLighting(position, normal, baseColor, specular,
    //         roughness, metallic, true, visibility);

    // visibility = CaculateDirectionalShadow(normal, position, depth);
    //     directLighting = vec3(0.0f);
    //     baseColor.rgb = specular.rgb = vec3(1.0f);
    //     indirectLighting = CalculateIndirectLighting(position, normal, baseColor, specular,
    //         roughness, metallic, true, visibility);

    // vec3 debug = debug_cascade(depth);
    // directLighting =  debug;

    // visibility = CaculateDirectionalShadow(normal, position, depth);
    //     directLighting = vec3(0.0f);
    //     baseColor.rgb = specular.rgb = vec3(1.0f);
    //     indirectLighting = CalculateIndirectLighting(position, normal, baseColor, specular,
    //         roughness, metallic, true, visibility);

    // visibility = CaculateDirectionalShadow(normal, position, depth);
    //     directLighting = vec3(0.0f);
    //     baseColor.rgb = specular.rgb = vec3(1.0f);
    //     indirectLighting = CalculateIndirectLighting(position, normal, baseColor, specular,
    //         roughness, metallic, true, visibility);

    // visibility = CaculateDirectionalShadow(normal, position, depth);
    //     directLighting = vec3(0.0f);
    //     baseColor.rgb = specular.rgb = vec3(1.0f);
    //     indirectLighting = CalculateIndirectLighting(position, normal, baseColor, specular,
    //         roughness, metallic, true, visibility);

    // visibility = CaculateDirectionalShadow(normal, position, depth);
    //     directLighting = vec3(0.0f);
    //     baseColor.rgb = specular.rgb = vec3(1.0f);
    //     indirectLighting = CalculateIndirectLighting(position, normal, baseColor, specular,
    //         roughness, metallic, true, visibility);


    indirectLighting.rgb = pow(indirectLighting.rgb, vec3(2.2f));
    compositeLighting = (directLighting + indirectLighting.rgb) * indirectLighting.a;
    compositeLighting += emissive;


    // Reinhard tone mapping
    //compositeLighting = compositeLighting / (compositeLighting + 1.0f);

    compositeLighting = compositeLighting;

    // convert to gamma space
    const float gamma = 2.2;
    compositeLighting = pow(compositeLighting, vec3(1.0 / gamma));

    fragColor = vec4(compositeLighting, texture(gNormal, TexCoords).a);

    if(depth * (zFar - zNear) < 2.0f)
        LabelAndDepth = vec4(id_color, 1);
    else
        LabelAndDepth = vec4(1,1,1,1);
}