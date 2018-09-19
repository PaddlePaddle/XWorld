#version 430
#extension GL_ARB_shader_image_load_store : require

in GeometryOut
{
    vec3 wsPosition;
    vec3 position;
    vec3 normal;
    vec3 texCoord;
    flat vec4 triangleAABB;
} In;

layout (location = 0) out vec4 fragColor;
layout (pixel_center_integer) in vec4 gl_FragCoord;

layout(binding = 0, r32ui) uniform volatile coherent uimage3D voxelAlbedo;
layout(binding = 1, r32ui) uniform volatile coherent uimage3D voxelNormal;
layout(binding = 2, r32ui) uniform volatile coherent uimage3D voxelEmission;
// layout(binding = 3, r8) uniform image3D staticVoxelFlag;

layout(binding = 4) uniform sampler2D texture_diffuse0;
// layout(binding = 5) uniform sampler2D texture_specular0;
// layout(binding = 6) uniform sampler2D texture_normal0;
// layout(binding = 7) uniform sampler2D texture_height0;
// layout(binding = 8) uniform sampler2D texture_ao0;
layout(binding = 9) uniform sampler2D opacityMap0;
layout(binding = 10) uniform sampler2D emissiveMap0;

uniform struct Material
{
    vec3 diffuse;
    vec3 emissive;
} material;

uniform int volumeDimension;
uniform int flagStaticVoxels = 1;
uniform int linear = 1;

// MTL
uniform float d = 1;
uniform vec3 kD = vec3(1,0,0);
uniform int diffuseMap = 1;

vec4 convRGBA8ToVec4(uint val)
{
    return vec4(float((val & 0x000000FF)), 
    float((val & 0x0000FF00) >> 8U), 
    float((val & 0x00FF0000) >> 16U), 
    float((val & 0xFF000000) >> 24U));
}

uint convVec4ToRGBA8(vec4 val)
{
    return (uint(val.w) & 0x000000FF) << 24U | 
    (uint(val.z) & 0x000000FF) << 16U | 
    (uint(val.y) & 0x000000FF) << 8U | 
    (uint(val.x) & 0x000000FF);
}

void imageAtomicRGBA8Avg(layout(r32ui) volatile coherent uimage3D grid, ivec3 coords, vec4 value)
{
    value.rgb *= 255.0;                 // optimize following calculations
    uint newVal = convVec4ToRGBA8(value);
    uint prevStoredVal = 0;
    uint curStoredVal;
    uint numIterations = 0;

    while((curStoredVal = imageAtomicCompSwap(grid, coords, prevStoredVal, newVal)) 
            != prevStoredVal
            && numIterations < 255)
    {
        prevStoredVal = curStoredVal;
        vec4 rval = convRGBA8ToVec4(curStoredVal);
        rval.rgb = (rval.rgb * rval.a); // Denormalize
        vec4 curValF = rval + value;    // Add
        curValF.rgb /= curValF.a;       // Renormalize
        newVal = convVec4ToRGBA8(curValF);

        ++numIterations;
    }
}

// void imageAtomicRGBA8Avg(layout(r32ui) volatile coherent uimage3D grid, ivec3 coords, vec4 value)
// {
//     uint nextUint = packUnorm4x8(vec4(value.rgb, 1.0f / 255.0f));
//     uint prevUint = 0;
//     uint currUint;

//     vec4 currVec4;

//     vec3 average;
//     uint count;

//     while((currUint = imageAtomicCompSwap(grid, coords, prevUint, nextUint)) != prevUint)
//     {
//         prevUint = currUint;
//         currVec4 = unpackUnorm4x8(currUint);

//         average = currVec4.rgb;
//         count = uint(currVec4.a * 255.0f);

//         average = (average * count + value.rgb) / (count + 1);

//         nextUint = packUnorm4x8(vec4(average, (count + 1) / 255.0f));
//     }
// }

vec3 EncodeNormal(vec3 normal)
{
    return normal * 0.5f + vec3(0.5f);
}

vec3 DecodeNormal(vec3 normal)
{
    return normal * 2.0f - vec3(1.0f);
}

void main()
{
    if( In.position.x < In.triangleAABB.x || In.position.y < In.triangleAABB.y || 
		In.position.x > In.triangleAABB.z || In.position.y > In.triangleAABB.w )
	{
		discard;
	}


    ivec3 position = ivec3(In.wsPosition);
    vec4 albedo = vec4(kD, 1.0f);
    if(diffuseMap == 1)
    {
        albedo.rgb = texture(texture_diffuse0, In.texCoord.xy).rgb;
    }

    // alpha cutoff
    if(albedo.a > 0.0f)
    {
        // albedo is in srgb space, bring back to linear
        if(linear == 1)
            albedo.rgb = pow(albedo.rgb, vec3(2.2f));
        // premultiplied alpha
        albedo.rgb *= albedo.a;
        albedo.a = 1.0f;
        // emission value
        vec4 emissive = vec4(albedo.rgb * 0.01f, 1.0f);

        vec4 normal = vec4(EncodeNormal(normalize(In.normal)), 1.0f);

        imageAtomicRGBA8Avg(voxelNormal, position, normal);
        imageAtomicRGBA8Avg(voxelAlbedo, position, albedo);
        imageAtomicRGBA8Avg(voxelEmission, position, emissive);
        
        // if(flagStaticVoxels == 1)
        // {
        //     imageStore(staticVoxelFlag, position, vec4(1.0));
        // }
    }
}
