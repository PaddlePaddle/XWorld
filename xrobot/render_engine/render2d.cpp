#include "render2d.h"

namespace xrobot {
namespace render_engine {

void RenderMap::renderAABB(vec3 min, vec3 max) {
    float quadVertices[] = {
        min.x,  -max.z, 0.0f,
        min.x,  -min.z, 0.0f,
        max.x,  -max.z, 0.0f,
        max.x,  -min.z, 0.0f,
    };

    if (aabb_vao_ == 0) {
        // setup plane VAO
        glGenVertexArrays(1, &aabb_vao_);
        glGenBuffers(1, &aabb_vao_);
        glBindVertexArray(aabb_vao_);
        glBindBuffer(GL_ARRAY_BUFFER, aabb_vbo_);
        glBufferData(GL_ARRAY_BUFFER,
                     sizeof(quadVertices),
                     &quadVertices,
                     GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0,
                              3,
                              GL_FLOAT,
                              GL_FALSE,
                              3 * sizeof(float),
                              (void*)0);
    }
    glBindBuffer(GL_ARRAY_BUFFER, aabb_vbo_);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(quadVertices), &quadVertices);

    glBindVertexArray(aabb_vao_);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glBindVertexArray(0);
}

void RenderMap::visualize(Map * s) {
    glViewport(0, 480 - 256, 256, 256);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable( GL_POLYGON_OFFSET_FILL );

    shader_.use();
    // shader.setVec3("scale", vec3(0.2f));

    for (int i = 0; i < s->sections_map_.size(); ++i) {
        vec3 aabb_min = s->sections_map_[i].first;
        vec3 aabb_max = s->sections_map_[i].second;

        renderAABB(aabb_min, aabb_max);
    }

    glPolygonOffset(1.0f, 1.0f);
    glEnable( GL_POLYGON_OFFSET_FILL );
    glDisable(GL_BLEND);
}

} } // xrobot::render_engine
