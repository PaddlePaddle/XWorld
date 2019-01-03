#version 330 core

out float FragColor;
in vec2 TexCoords;

uniform sampler2D gPosition;
uniform sampler2D gNormal;
uniform sampler2D texNoise;
uniform float height;
uniform float width;
uniform vec3 samples[64];

int kernelSize = 64;
float radius = 0.5;
float bias = 0.025;
float scale = 2.0f;

uniform mat4 projection;
uniform mat4 view;

void main()
{
    vec2 noiseScale = vec2(width/4.0, height/4.0); 
    vec3 fragPos = texture(gPosition, TexCoords).xyz;
    vec3 normal = -normalize(mat3(view) * texture(gNormal, TexCoords).xyz);
    vec3 randomVec = normalize(texture(texNoise, TexCoords * noiseScale).xyz);

    vec3 tangent = normalize(randomVec - normal * dot(randomVec, normal));
    vec3 bitangent = cross(normal, tangent);
    mat3 TBN = mat3(tangent, bitangent, normal);

    vec4 proj_coord = projection * vec4(fragPos, 1.0);
    float depth = (proj_coord.z / proj_coord.w) * 0.5f + 0.5f;

    FragColor = 1.0;
    float occlusion = 0.0;
    if(depth < 1) {
        for(int i = 0; i < kernelSize; ++i)
        {
            vec3 sample = TBN * samples[i]; // view-space
            sample = fragPos + sample * radius; 
            
            vec4 offset = vec4(sample, 1.0);
            offset = projection * offset;
            offset.xyz /= offset.w;
            offset.xyz = offset.xyz * 0.5 + 0.5;
            
            float sampleDepth = texture(gPosition, offset.xy).z;
            float rangeCheck = smoothstep(0.0, 1.0, radius / abs(fragPos.z - sampleDepth));
            occlusion += (sampleDepth >= sample.z + bias ? 1.0 : 0.0) * rangeCheck;   
        }
    
        occlusion = 1.0 - (occlusion / kernelSize);
        FragColor = pow(occlusion, scale);
    }
}