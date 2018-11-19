#version 330 core

out float FragColor;
in vec2 TexCoords;

uniform sampler2D gPosition;
uniform sampler2D gNormal;
uniform sampler2D texNoise;
uniform float height = 640;
uniform float width = 480;
uniform mediump vec3 samples[64];

mediump int kernelSize = 64;
highp float radius = 0.3;
highp float bias = 0.025;
highp float scale = 1.2f;

uniform mat4 projection;
uniform mat4 view;

void main()
{

    if(texture(gNormal, TexCoords).a == 0) {
        discard;
    }

    vec2 noiseScale = vec2(width/4.0, height/4.0); 
    vec3 fragPos = vec3(view * vec4(texture(gPosition, TexCoords).xyz, 1));
    vec3 normal = -normalize(mat3(view) * texture(gNormal, TexCoords).xyz);
    vec3 randomVec = normalize(texture(texNoise, TexCoords * noiseScale).xyz);

    vec3 tangent = normalize(randomVec - normal * dot(randomVec, normal));
    vec3 bitangent = cross(normal, tangent);
    mat3 TBN = mat3(tangent, bitangent, normal);

    highp float occlusion = 0.0;
    for(lowp int i = 0; i < kernelSize; ++i)
    {
        vec3 sample = TBN * samples[i];
        sample = fragPos + sample * radius; 
        
        vec4 offset = vec4(sample, 1.0);
        offset = projection * offset;
        offset.xyz /= offset.w;
        offset.xyz = offset.xyz * 0.5 + 0.5;
        
        highp float sampleDepth = vec3(view * vec4(texture(gPosition, offset.xy).xyz, 1)).z;

        if(texture(gNormal, offset.xy).a > 0) {
            float rangeCheck = clamp(smoothstep(0.0, 1.0, radius / abs(fragPos.z - sampleDepth)), 0, 1);
            occlusion += (sampleDepth >= sample.z + bias ? 1.0 : 0.0) * rangeCheck;      
        }     
    }
    
    occlusion = 1.0 - clamp(occlusion / kernelSize, 0.0, 1.0);

    FragColor = pow(occlusion, scale);
}