// MIT License

// Copyright (c) 2017 José Villegas

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#version 430
#extension GL_ARB_shader_image_load_store : require

out vec4 albedo;

layout(binding = 0, rgba8) uniform readonly image3D voxelRadiance;

uniform int volumeDimension;
uniform vec4 colorChannels;

void main()
{
	float volumeDimensionF = float(volumeDimension);

	vec3 position = vec3
	(
		gl_VertexID % volumeDimension,
		(gl_VertexID / volumeDimension) % volumeDimension,
		gl_VertexID / (volumeDimension * volumeDimension)
	);

	ivec3 texPos = ivec3(position);
	albedo = imageLoad(voxelRadiance, texPos);

	uvec4 channels = uvec4(floor(colorChannels));

	albedo = vec4(albedo.rgb * channels.rgb, albedo.a);
	// if no color channel is enabled but alpha is one, show alpha as rgb
	if(all(equal(channels.rgb, uvec3(0))) && channels.a > 0) 
	{
		albedo = vec4(albedo.a);
	}

	gl_Position = vec4(position, 1.0f);
}