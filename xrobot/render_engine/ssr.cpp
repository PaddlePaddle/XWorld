#include "ssr.h"

namespace xrobot {
namespace render_engine {

SSR::SSR(const float width, const float height) : PostProcessing(width, height) {

	ssr_ = std::make_shared<RenderTarget>(width, height);
	ssr_->append_rgb_float32();
	ssr_->init();

	InitShaders();

	glGenFramebuffers(1, &color_mipmaps_fb_);
	glBindFramebuffer(GL_FRAMEBUFFER, color_mipmaps_fb_);
	glGenTextures(1, &color_mipmaps_);
	glBindTexture(GL_TEXTURE_2D, color_mipmaps_);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, 
			width_, height_, 0, GL_RGBA, GL_FLOAT, 0);
	glGenerateMipmap(GL_TEXTURE_2D); 
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, 
			GL_TEXTURE_2D, color_mipmaps_, 0);

	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
		std::cout << "Framebuffer not complete!" << std::endl;
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

SSR::~SSR() {
	glDeleteFramebuffers(1, &color_mipmaps_fb_);
	glDeleteTextures(1, &color_mipmaps_);
}

void SSR::InitShaders() {
	std::string pwd = get_pwd(__FILE__);
	shaders_.resize((int) kSSRShaders);

	shaders_[kSSR] = Shader(pwd+"/shaders/quad.vs",
							pwd+"/shaders/ssr.fs");

	shaders_[kSSRCopy] = Shader(pwd+"/shaders/quad.vs",
							    pwd+"/shaders/flat.fs");
}

void SSR::Draw(std::vector<GLuint>& in,
		        std::vector<GLuint>& out,
		        const glm::mat4 view,
		        const glm::mat4 projection) {

	glDisable(GL_BLEND);
	glDisable(GL_DEPTH_TEST);
	glClearColor(0, 0, 0, 0);

	GLuint color      = in[0];
	GLuint g_position = in[1];
	GLuint g_normal   = in[2];

	CopyTexture(color);

	glBindTexture(GL_TEXTURE_2D, color_mipmaps_);
	glGenerateMipmap(GL_TEXTURE_2D);

	SSRPass(color_mipmaps_, g_position, g_normal, view, projection);

	out.clear();
	out.push_back(ssr_->texture_id(0));
}

void SSR::CopyTexture(const GLuint color) {
	auto copy = shaders_[kSSRCopy];

	glBindFramebuffer(GL_FRAMEBUFFER, color_mipmaps_fb_);
	glViewport(0, 0, width_, height_);
	glClear(GL_COLOR_BUFFER_BIT);

	copy.use();
	copy.setInt("tex", 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, color);

	RenderQuad();
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void SSR::SSRPass(const GLuint color, 
				  const GLuint g_position,
				  const GLuint g_normal,
				  const glm::mat4 view,
				  const glm::mat4 projection) {

	auto ssr = shaders_[kSSR];

	glBindFramebuffer(GL_FRAMEBUFFER, ssr_->id());
	glViewport(0, 0, width_, height_);
	glClear(GL_COLOR_BUFFER_BIT);

	ssr.use();
	ssr.setMat4("view", view);
	ssr.setMat4("projection", projection);
	ssr.setInt("renderedTexture", 0);
	ssr.setInt("gPosition", 1);
	ssr.setInt("gNormal", 2);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, color);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, g_position);
	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, g_normal);
	RenderQuad();
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}


}
}