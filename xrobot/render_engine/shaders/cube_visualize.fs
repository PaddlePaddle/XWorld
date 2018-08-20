#version 330 core

out vec4 FragColor;

in vec2 TexCoords;

uniform samplerCube tex;
uniform int dir = 0;
uniform float zNear = 0.01;
uniform float zFar = 20.0;

const float fov = 3.14159  * 90.0 / 360.0;

float linearize(float depth)
{
    return (2 * zNear) / (zFar + zNear - depth * (zFar - zNear));
}

vec3 packFloatInto8BitVec3(float v, float min, float max) {
   float zeroToOne = (v - min) / (max - min);
   int zeroTo24Bit = int(zeroToOne * 256.0 * 256.0 * 255.0);
   return vec3(zeroTo24Bit % 256, int(zeroTo24Bit / 256.0) % 256, zeroTo24Bit / 256.0 / 256.0);
}

vec4 visualize(vec3 dir, float phi, float theta)
{
	float depth_lg = texture(tex, normalize(dir)).r;
	float depth_linear = linearize(depth_lg);
	float distance = depth_linear / abs(cos(phi) * cos(theta));
	return vec4(vec3(distance), 1.0f);
}

void main()
{	
	vec2 mapCoord = 2.0 * TexCoords - 1.0;

	float phi = fov * mapCoord.x;
	float theta = fov * mapCoord.y;

	float tanfov = tan(fov);

	mapCoord.x = tan(phi) / tanfov;
	mapCoord.y = tan(theta) / (cos(phi) * tanfov);

	if(dir == 0)
    	FragColor = visualize(vec3( 1.0, mapCoord.y, mapCoord.x), phi, theta);
    else if(dir == 1)
    	FragColor = visualize(vec3(-1.0, mapCoord.y, -mapCoord.x), phi, theta);
    // else if(dir == 2)
    // 	FragColor = visualize(vec3(mapCoord.x, 1.0, mapCoord.y), phi, theta);
    // else if(dir == 3)
    // 	FragColor = visualize(vec3(mapCoord.x,-1.0, mapCoord.y), phi, theta);
    else if(dir == 4)
    	FragColor = visualize(vec3(-mapCoord.x, mapCoord.y, 1.0), phi, theta);
    else if(dir == 5)
    	FragColor = visualize(vec3(mapCoord.xy, -1.0), phi, theta);
}

