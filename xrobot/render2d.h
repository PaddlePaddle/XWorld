#pragma once

#include "gl_context.h"
#include "shader.h"
#include "map.h"

#include "glm/gtc/type_ptr.hpp"

#include <stdio.h>
#include <string.h>
#include <vector>
#include <ctime>
#include <ratio>
#include <chrono>

using namespace glm;

namespace xrobot {


class RenderMap
{
public:

	// Quad
	GLuint aabbVAO = 0;
	GLuint aabbVBO;

	Shader shader;

	RenderMap()
	{
		shader = Shader("./shaders/visualize_grid.vs",
						"./shaders/visualize_grid.fs");
	}


	~RenderMap();

	void renderAABB(vec3 min, vec3 max)
	{
		float quadVertices[] = {
			min.x,  -max.z, 0.0f,
			min.x,  -min.z, 0.0f,
			max.x,  -max.z, 0.0f,
			max.x,  -min.z, 0.0f,
	    };

	    if (aabbVAO == 0)
	    {
	        // setup plane VAO
	        glGenVertexArrays(1, &aabbVAO);
	        glGenBuffers(1, &aabbVBO);
	        glBindVertexArray(aabbVAO);
	        glBindBuffer(GL_ARRAY_BUFFER, aabbVBO);
	        glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_DYNAMIC_DRAW);
	        glEnableVertexAttribArray(0);
	        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	    }
	    glBindBuffer(GL_ARRAY_BUFFER, aabbVBO);
	    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(quadVertices), &quadVertices);

	    glBindVertexArray(aabbVAO);
	    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	    glBindVertexArray(0);
	}

	void visualize(Map * s)
	{
		glViewport(0, 480 - 256, 256, 256);
		glDisable(GL_DEPTH_TEST);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable( GL_POLYGON_OFFSET_FILL );

		shader.use();
		// shader.setVec3("scale", vec3(0.2f));

        for (int i = 0; i < s->sections_map_.size(); ++i)
        {
            vec3 aabb_min = s->sections_map_[i].first;
            vec3 aabb_max = s->sections_map_[i].second;

            renderAABB(aabb_min, aabb_max);
        }

        glPolygonOffset(1.0f, 1.0f);
        glEnable( GL_POLYGON_OFFSET_FILL );
        glDisable(GL_BLEND);
	}

};

}