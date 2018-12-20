#include "fxaa.h"

namespace xrobot {
namespace render_engine {

FXAA::FXAA(const float width, const float height) : PostProcessing(width, height) {

	aa_ = std::make_shared<RenderTarget>(width, height);
	aa_->append_rgb_uint8();
	aa_->init();

	InitShaders();
}

FXAA::~FXAA() {}

void FXAA::InitShaders() {
	std::string pwd = get_pwd(__FILE__);
	shaders_.resize((int) kFXAAShaders);

	shaders_[kFXAA] = Shader(pwd+"/shaders/quad.vs",
							 pwd+"/shaders/fxaa.fs");
}

void FXAA::Draw(std::vector<GLuint>& in,
		        std::vector<GLuint>& out) {

	glDisable(GL_BLEND);
	glDisable(GL_DEPTH_TEST);
	glClearColor(0, 0, 0, 0);

	GLuint color = in[0];

	FXAAPass(color);

	out.clear();
	out.push_back(aa_->texture_id(0));
}

void FXAA::FXAAPass(const GLuint color) {
	auto aa = shaders_[kFXAA];

	glBindFramebuffer(GL_FRAMEBUFFER, aa_->id());
	glViewport(0, 0, width_, height_);
	glClear(GL_COLOR_BUFFER_BIT);

	aa.use();
	aa.setInt("tex", 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, color);
	RenderQuad();
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

}
}