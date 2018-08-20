#version 330 core

out vec4 FragColor;

out VS_OUT {
	vec3 color;
} fs_in;

void main()
{
	vec2 mapCoord = 2.0 * TexCoords - 1.0;

	float phi = fov * mapCoord.x;
	float theta = fov * mapCoord.y;

	float tanfov = tan(fov);

	mapCoord.x = tan(phi) / tanfov;
	mapCoord.y = tan(theta) / (cos(phi) * tanfov);
	
	if(dir == 0) {

		float depth_lg = texture(tex, normalize(dir)).r;
		float depth_linear = linearize(depth_lg);
		float distance = depth_linear / abs(cos(phi) * cos(theta));
    	
		float x = sin(phi) * cos(theta);
		float y = sin(phi) * sin(theta);
		float z = cos(phi);

    	vec3 point = eye + distance * normalize(vec3(x,y,z));

    	vec4 proj = projection * view * vec4(point, 1);
    	proj /= proj.w;


	}


	FragColor = vec4(fs_in.color, 1);
}