#ifndef RENDER_ENGINE_CAPTURE_H_
#define RENDER_ENGINE_CAPTURE_H_

#include <chrono>
#include <ctime>
#include <ratio>
#include <stdio.h>
#include <string>
#include <vector>

#include "gl_context.h"
#include "common.h"
#include "shader.h"
#include "csm.h"
#include "texture2d.h"
#include "texture3d.h"
#include "render_target.h"
#include "render_world.h"

namespace xrobot {
namespace render_engine {

enum CaptureShaders {
    kCapture,
    kCubemap,
    kCaptureShaders
};

class Capture {
public:
	Capture(const int resolution);
	~Capture();

	void Stitch(GLuint& rgb, Image<float>& lidar_image);
	void RenderCubemap(RenderWorld* world, Camera* camera);

	GLuint GetRawCubeMap() const { return raw_capture_cubemap_; }

private:
	void InitPBOs();
	void InitShaders();
	void InitCapture();
	void Draw(RenderWorld* world, const Shader& shader);
	void RenderQuad();

	int resolution_;
	GLuint raw_capture_fb_;
	GLuint raw_capture_cubemap_;
	GLuint quad_vao_, quad_vbo_;
	GLuint lidar_pbo_;

	std::shared_ptr<RenderTarget> capture_;

	std::vector<Shader> shaders_;
};

}
}

#endif // RENDER_ENGINE_CAPTURE_H_
