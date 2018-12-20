#ifndef RENDER_ENGINE_POSTPROCESS_H_
#define RENDER_ENGINE_POSTPROCESS_H_

#include <string>
#include <vector>

#include "gl_context.h"
#include "shader.h"
#include "texture2d.h"
#include "render_target.h"

namespace xrobot {
namespace render_engine {

class PostProcessing {
public:
	PostProcessing(const float width, const float height);
	virtual ~PostProcessing();
	virtual void InitShaders();
	void RenderQuad();

protected:
	std::vector<Shader> shaders_;
	GLuint quad_vao_, quad_vbo_;
	float width_, height_;
};


}}

#endif // RENDER_ENGINE_POSTPROCESS_H_
