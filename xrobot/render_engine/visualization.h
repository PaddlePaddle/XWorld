#ifndef RENDER_ENGINE_VISUALIZATION_H_
#define RENDER_ENGINE_VISUALIZATION_H_

#include <chrono>
#include <ctime>
#include <ratio>
#include <stdio.h>
#include <string>
#include <vector>

#include "common.h"
#include "gl_context.h"
#include "shader.h"
#include "texture2d.h"
#include "render_target.h"
#include "render_world.h"

namespace xrobot {
namespace render_engine {

typedef std::chrono::high_resolution_clock::time_point TimeFrame;

enum VisualizationShader {
	kAABB,
	kRay,
	kLambert,
	kVisualizationShaders
};

class Visualization {	
public:
	Visualization(const int width,
				  const int height,
				  GLContext* ctx);
	~Visualization();

	void Visualize(RenderWorld* world);
	GLuint GetTexture() const { return visualization_->texture_id(0); }

	void InitShaders();
	void RenderAABB();
	void DrawRootAABB(RenderWorld* world, const Shader& shader);
	void DrawSubTiles(RenderWorld* world, const Shader& shader);
	void DrawWorldAABB(RenderWorld* world, const Shader& shader);
	void DrawBatchRays();
	void UpdateRay(const int offset, const glm::vec3 from, const glm::vec3 to);
	void InitDrawBatchRays(const int rays);
	void Draw(RenderWorld* world, const Shader& shader);

	void GetDeltaTime();
    void ProcessInput();
    void ProcessMouse();

private:
    Camera free_camera_;
    float last_x_, last_y_, delta_time_;
    bool first_mouse_;
    TimeFrame last_frame_;

    GLContext* ctx_;
    int width_, height_;
    std::vector<Shader> shaders_;

    bool lidar_;
    int num_rays_;
    GLuint aabb_vao_, aabb_vbo_;
    GLuint batch_ray_vao_, batch_ray_vbo_;

    std::shared_ptr<RenderTarget> visualization_;
};

}
}

#endif // RENDER_ENGINE_RENDER_H_
