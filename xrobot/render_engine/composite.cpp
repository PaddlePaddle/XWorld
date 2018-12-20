#include "composite.h"

namespace xrobot {
namespace render_engine {

Composite::Composite(const float width, 
					 const float height) : PostProcessing(width, height) {

	composite_ = std::make_shared<RenderTarget>(width, height);
	composite_->append_rgb_uint8();
	composite_->init();

	InitShaders();
}

Composite::~Composite() {}

void Composite::InitShaders() {
	std::string pwd = get_pwd(__FILE__);
	shaders_.resize((int) kCompositeShaders);

	shaders_[kComposite] = Shader(pwd+"/shaders/quad.vs",
							      pwd+"/shaders/composite.fs");
}

void Composite::Draw(std::vector<GLuint>& in, std::vector<GLuint>& out,
		const bool ssao, const bool ssr, const bool shading) {
	glDisable(GL_BLEND);
	glDisable(GL_DEPTH_TEST);
	glClearColor(0, 0, 0, 0);

	GLuint base       = in[0];
	GLuint ao         = in[1];
	GLuint ref        = in[2];

	auto composite = shaders_[kComposite];
	glBindFramebuffer(GL_FRAMEBUFFER, composite_->id());
	glViewport(0, 0, width_, height_);

	glClear(GL_COLOR_BUFFER_BIT);

	composite.use();
	composite.setFloat("shading", (float) shading);
	composite.setFloat("use_ao",  (float) ssao);
	composite.setFloat("use_ref", (float) ssr);
	composite.setInt("lighting", 0);
	composite.setInt("ssao", 1);
	composite.setInt("ssr", 2);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, base);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, ao);
	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, ref);
	RenderQuad();

	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	out.clear();
	out.push_back(composite_->texture_id(0));
}

}
}