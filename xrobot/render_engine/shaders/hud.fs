#version 330 core

out vec4 FragColor;
in vec2 TexCoords;

const highp float ep = 0.005f;

// Disk
uniform lowp float show_disk = 1.0f;
uniform highp float aspect_ratio = 1.0f;
uniform highp float disk_radius = 0.01f;
uniform highp float alpha = 1.0f;
uniform vec3 disk_color = vec3(0.9f, 0.0f, 0.0f);
uniform vec2 disk_center = vec2(0.5f, 0.5f);

uniform mat4 projection;
uniform float near;
uniform float far;

// Inventory
uniform float start_u = 0;
uniform float end_u = 0;

uniform sampler2D rgb;
uniform sampler2D depth;

float linearize(float depth)
{
    return (2 * near) / (far + near - depth * (far - near));
}

void main()
{

	FragColor.rgb = texture(rgb, TexCoords).rgb;
	vec3 position = texture(depth, TexCoords).xyz;

	vec4 proj_coord = projection * vec4(position, 1.0);
    float depth = (proj_coord.z / proj_coord.w) * 0.5f + 0.5f;
    FragColor.a = linearize(depth);

	if(depth >= 1) 
		FragColor.a = 1;

	// Draw Disk
	vec2 uv_offset = (TexCoords - disk_center) * vec2(aspect_ratio, 1);
	float dist = sqrt(dot(uv_offset, uv_offset));
	float t = smoothstep(disk_radius + ep, disk_radius - ep, dist) * alpha;
	FragColor.rgb = mix(FragColor.rgb, disk_color, t); 

	// Draw Inventory
	if(TexCoords.x > start_u && TexCoords.x < end_u && TexCoords.y < 0.1) 
	{
		FragColor.rgb = vec3(0.3);
	}
}