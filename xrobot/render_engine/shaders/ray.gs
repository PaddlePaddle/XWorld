#version 330 core
layout (lines) in;
layout (line_strip, max_vertices = 8) out;

out GS_OUT {
	vec4 Color;
} gs_out;

in VS_OUT {
	bool first;
} gs_in[];

uniform mat4 projection;
uniform mat4 view;

void draw_crosshair(in vec4 center)
{
	vec4 pos;

	pos = center + vec4(0.05, 0, 0, 0);
	gl_Position = projection * view * pos;
	EmitVertex();

	pos = center - vec4(0.05, 0, 0, 0);
	gl_Position = projection * view * pos;
	EmitVertex();

	EndPrimitive();


	pos = center + vec4(0, 0.05, 0, 0);
	gl_Position = projection * view * pos;
	EmitVertex();

	pos = center - vec4(0, 0.05, 0, 0);
	gl_Position = projection * view * pos;
	EmitVertex();

	EndPrimitive();


	pos = center + vec4(0, 0, 0.05, 0);
	gl_Position = projection * view * pos;
	EmitVertex();

	pos = center - vec4(0, 0, 0.05, 0);
	gl_Position = projection * view * pos;
	EmitVertex();

	EndPrimitive();
}

void main()
{
	float dist01 = length(vec3(gl_in[0].gl_Position - gl_in[1].gl_Position)) / 5.0;

	// Draw Ray
	gs_out.Color = vec4(0,0,1,0.1);
	gl_Position = projection * view * gl_in[0].gl_Position;
	EmitVertex();

	gs_out.Color = vec4(mix(vec3(0,0,1), vec3(1,0,0), dist01), 0.95);
	gl_Position = projection * view * gl_in[1].gl_Position;
	EmitVertex();

	EndPrimitive();

	// Draw Crosshair
	if(gl_in[0].gl_Position != gl_in[1].gl_Position)
	{
		gs_out.Color = vec4(0,1,0,1);
		if(gs_in[0].first)
			gs_out.Color = vec4(1,1,1,1);
		draw_crosshair(gl_in[1].gl_Position);
	}
}