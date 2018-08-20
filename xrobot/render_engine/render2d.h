#ifndef RENDER_ENGINE_RENDER_2D_H
#define RENDER_ENGINE_RENDER_2D_H

#include <chrono>
#include <ctime>
#include <ratio>
#include <stdio.h>
#include <string.h>
#include <vector>

#include "glm/gtc/type_ptr.hpp"

#include "gl_context.h"
#include "../map.h"
#include "shader.h"

using namespace glm;

namespace xrobot {
namespace render_engine {

class RenderMap {
public:
	RenderMap() : aabb_vao_(0), aabb_vbo_(0) {
		shader_ = Shader("./shaders/visualize_grid.vs",
						"./shaders/visualize_grid.fs");
	}

	~RenderMap();

	void renderAABB(vec3 min, vec3 max);

	void visualize(Map* s);

private:
	// Quad
	GLuint aabb_vao_;
	GLuint aabb_vbo_;
	Shader shader_;
};

} } // xrobot::render_engine

#endif
