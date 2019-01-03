// SSR
// Adapted from JoeyDeVries

#version 330 core
 
#extension GL_ARB_texture_query_lod : enable

uniform sampler2D renderedTexture;
uniform sampler2D gPosition;
uniform sampler2D gNormal;

uniform mat4 projection;
uniform mat4 view;

in vec2 TexCoords;
out vec3 FragColor;

const float step = 0.1;
const float minRayStep = 0.1;
const float maxSteps = 60;
const int numBinarySearchSteps = 10;

vec3 binarySearch(inout vec3 dir, inout vec3 hitCoord, inout float dDepth);

vec4 rayMarch(vec3 dir, inout vec3 hitCoord, out float dDepth);

void main()
{
    vec4 g0        = texture(gPosition, TexCoords);
    vec3 viewPos   = g0.xyz;
    vec3 worldNorm = normalize(texture(gNormal, TexCoords).xyz);
    vec3 viewNorm  = normalize(mat3(view) * worldNorm);
    float rough    = g0.w;

    vec3 N = normalize(viewNorm);
    vec3 V = normalize(-viewPos);
    vec3 R = reflect(-V, N);
    vec3 hitPos = viewPos;
    float dDepth;
    
    vec4 coords = rayMarch(R * max(minRayStep, -viewPos.z), hitPos, dDepth);
    vec2 dCoords = smoothstep(0.2, 0.6, abs(vec2(0.5, 0.5) - coords.xy));
    float error = coords.w;
    float screenEdgefactor = clamp(1.0 - (dCoords.x + dCoords.y), 0.0, 1.0);
    
    vec3 color = textureLod(renderedTexture, coords.xy, rough * 3.5f).rgb;
    vec3 ssr = color * clamp(screenEdgefactor, 0.0, 1.0);
    
    error *= clamp(screenEdgefactor, 0.0, 0.9);
    
    float backfacingFactor = 1.0 - smoothstep(-1.0, -0.8, R.z);
    ssr *= backfacingFactor;
    error *= backfacingFactor;
    FragColor = rough * 0.7 * mix(ssr, vec3(0), max(1.0 - error, 0.0));
}
 
vec3 binarySearch(inout vec3 dir, inout vec3 hitCoord, inout float dDepth)
{
    float depth;
    
    vec4 projectedCoord;
    
    for(int i = 0; i < numBinarySearchSteps; i++)
    {
        
        projectedCoord = projection * vec4(hitCoord, 1.0);
        projectedCoord.xy /= projectedCoord.w;
        projectedCoord.xy = projectedCoord.xy * 0.5 + 0.5;
        
        depth = texture(gPosition, projectedCoord.xy).z;
        dDepth = hitCoord.z - depth;
        
        dir *= 0.5;
        if(dDepth > 0.0)
            hitCoord += dir;
        else
            hitCoord -= dir;
    }

    projectedCoord = projection * vec4(hitCoord, 1.0);
    projectedCoord.xy /= projectedCoord.w;
    projectedCoord.xy = projectedCoord.xy * 0.5 + 0.5;

    return vec3(projectedCoord.xy, depth);
}

vec4 rayMarch(vec3 dir, inout vec3 hitCoord, out float dDepth)
{
    dir *= step;
    float error = 1.0;
    float depth;
    int steps;
    vec4 projectedCoord;
    
    for(int i = 0; i < maxSteps; i++)
    {
        hitCoord += dir;
        projectedCoord = projection * vec4(hitCoord, 1.0);
        projectedCoord.xy /= projectedCoord.w;
        projectedCoord.xy = projectedCoord.xy * 0.5 + 0.5;
        depth = texture(gPosition, projectedCoord.xy).z;
        if(depth > 1000.0)
            continue;
        
        dDepth = hitCoord.z - depth;
        
        if((dir.z - dDepth) < 1.2)
        {
            if(dDepth <= 0.0)
            {
                error = 0.0;
                vec4 Result;
                Result = vec4(binarySearch(dir, hitCoord, dDepth), 1.0);
                return Result;

            }
        }
        steps++;
    }
    return vec4(projectedCoord.xy, depth, error);
}