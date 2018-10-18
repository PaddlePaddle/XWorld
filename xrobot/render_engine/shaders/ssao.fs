#version 330 core

#pragma optionNV(unroll all)

out float FragColor;

in vec2 TexCoords;

uniform sampler2D gPosition;
uniform sampler2D gNormal;
uniform sampler2D texNoise;

uniform vec3 samples[64];

int kernelSize = 64;
float radius = 0.5;
float bias = 0.025;
float scale = 1.2f;

const vec2 noiseScale = vec2(640.0/4.0, 480.0/4.0); 

uniform mat4 projection;
uniform mat4 view;

void main()
{
    vec3 fragPos = vec3(view * vec4(texture(gPosition, TexCoords).xyz, 1));
    vec3 normal = -normalize(mat3(view) * texture(gNormal, TexCoords).xyz);
    vec3 randomVec = normalize(texture(texNoise, TexCoords * noiseScale).xyz);

    vec3 tangent = normalize(randomVec - normal * dot(randomVec, normal));
    vec3 bitangent = cross(normal, tangent);
    mat3 TBN = mat3(tangent, bitangent, normal);

    float occlusion = 0.0;
    for(int i = 0; i < kernelSize; ++i)
    {
        vec3 sample = TBN * samples[i];
        sample = fragPos + sample * radius; 
        
        vec4 offset = vec4(sample, 1.0);
        offset = projection * offset;
        offset.xyz /= offset.w;
        offset.xyz = offset.xyz * 0.5 + 0.5;
        
        float sampleDepth = vec3(view * vec4(texture(gPosition, offset.xy).xyz, 1)).z;

        if(texture(gNormal, offset.xy).a > 0) {
            float rangeCheck = smoothstep(0.0, 1.0, radius / abs(fragPos.z - sampleDepth));
            occlusion += (sampleDepth >= sample.z + bias ? 1.0 : 0.0) * rangeCheck;      
        }     
    }
    
    occlusion = 1.0 - clamp(occlusion / kernelSize, 0.0, 1.0);

    FragColor = pow(occlusion, scale);
}