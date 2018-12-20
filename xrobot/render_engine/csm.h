// MIT License

// Copyright (c) 2018 Dihara Wijetunga

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

#pragma once
#include <memory>
#include "camera.h"

#define MAX_FRUSTUM_SPLITS 8

namespace xrobot {
namespace render_engine {

struct FrustumSplit
{
	float near_plane;
	float far_plane;
	float ratio;
	float fov;
	glm::vec3 center;
	glm::vec3 corners[8];
};

struct CSMUniforms
{
	glm::vec4 direction;
    glm::vec4 options;
    int num_cascades;
    float far_bounds[8];
    glm::mat4 texture_matrices[8];
};

struct CSM
{
	GLuint m_shadow_maps[MAX_FRUSTUM_SPLITS];
	GLuint m_shadow_fbos[MAX_FRUSTUM_SPLITS];
	float m_lambda;
	float m_near_offset;
	int   m_split_count;
	int   m_shadow_map_size;
	FrustumSplit m_splits[MAX_FRUSTUM_SPLITS];
    float m_far_bounds[MAX_FRUSTUM_SPLITS];
	glm::vec3 m_light_direction;
    glm::mat4 m_bias;
	glm::mat4 m_light_view;
	glm::mat4 m_crop_matrices[MAX_FRUSTUM_SPLITS]; // crop * proj * view
	glm::mat4 m_proj_matrices[MAX_FRUSTUM_SPLITS]; // crop * proj * light_view * inv_view
    glm::mat4 m_texture_matrices[MAX_FRUSTUM_SPLITS];
	bool m_stable_pssm = true;

	CSM();
	~CSM();
	void initialize(float lambda, float near_offset, int split_count, int shadow_map_size, float fov, int _width, int _height, glm::vec3 dir);
	void shutdown();
	void update(Camera* camera, glm::vec3 dir);
	void update_splits(Camera* camera);
	void update_frustum_corners(Camera* camera);
	void update_crop_matrices(glm::mat4 t_modelview, Camera* camera);
    void update_texture_matrices(Camera* camera);
    void update_far_bounds(Camera* camera);
	
    inline FrustumSplit* frustum_splits() { return &m_splits[0]; }
    inline glm::mat4 split_view_proj(int i) { return m_crop_matrices[i]; }
    inline glm::mat4 texture_matrix(int i) { return m_texture_matrices[i]; }
    inline float far_bound(int i) { return m_far_bounds[i]; }
	inline GLuint* shadow_map() { return m_shadow_maps; }
	inline GLuint* framebuffers() { return &m_shadow_fbos[0]; }
	inline uint32_t frustum_split_count() { return m_split_count; }
	inline uint32_t near_offset() { return m_near_offset; }
	inline uint32_t lambda() { return m_lambda; }
	inline uint32_t shadow_map_size() { return m_shadow_map_size; }
};

}
}