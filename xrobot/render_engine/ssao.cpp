#include "ssao.h"

namespace xrobot {
namespace render_engine {

SSAO::SSAO(const float width, const float height) : PostProcessing(width, height),
													ssao_kernel_(0),
												    noise_(0) {

	ao_ = std::make_shared<RenderTarget>(width, height);
	ao_->append_r_float();
	ao_->init();

	blur_ = std::make_shared<RenderTarget>(width, height);
	blur_->append_r_float();
	blur_->init();

	InitShaders();

	GenerateNoise1DTexture();
	GenerateSSAOKernel();
}

SSAO::~SSAO() {
	glDeleteTextures(1, &noise_);
}

void SSAO::InitShaders() {
	std::string pwd = get_pwd(__FILE__);
	shaders_.resize((int) kSSAOShaders);

	shaders_[kSSAO] = Shader(pwd+"/shaders/quad.vs",
							 pwd+"/shaders/ssao.fs");

	shaders_[kBlur] = Shader(pwd+"/shaders/quad.vs",
							 pwd+"/shaders/blur.fs");
}

void SSAO::Draw(std::vector<GLuint>& in,
		        std::vector<GLuint>& out,
		        const glm::mat4 view,
		        const glm::mat4 projection) {

	glDisable(GL_BLEND);
	glDisable(GL_DEPTH_TEST);
	glClearColor(0, 0, 0, 0);

	GLuint g_position = in[0];
	GLuint g_normal   = in[1];

	AOPass(g_position, g_normal, view, projection);
	BlurPass(g_position);

	out.clear();
	out.push_back(blur_->texture_id(0));
}

void SSAO::AOPass(const GLuint g_position, const GLuint g_normal,
		const glm::mat4 view, const glm::mat4 projection) {
	auto ssao = shaders_[kSSAO];

	glBindFramebuffer(GL_FRAMEBUFFER, ao_->id());
	glViewport(0, 0, width_, height_);
	glClear(GL_COLOR_BUFFER_BIT);

	ssao.use();
	for (GLuint i = 0; i < 64; ++i)
		ssao.setVec3("samples[" + std::to_string(i) + "]", ssao_kernel_[i]);
	ssao.setMat4("view", view);
	ssao.setMat4("projection", projection);
	ssao.setInt("gPosition", 0);
	ssao.setInt("gNormal", 1);
	ssao.setInt("texNoise", 2);
	ssao.setFloat("height", height_);
	ssao.setFloat("width", width_);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, g_position);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, g_normal);
	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, noise_);
	RenderQuad();
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void SSAO::BlurPass(const GLuint g_position) {
	auto blur = shaders_[kBlur];

	glBindFramebuffer(GL_FRAMEBUFFER, blur_->id());
	glViewport(0, 0, width_, height_);
	glClear(GL_COLOR_BUFFER_BIT);

	blur.use();
	blur.setInt("ssaoInput", 0);
	blur.setInt("gPosition", 1);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, ao_->texture_id(0));
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, g_position);
	RenderQuad();
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void SSAO::GenerateNoise1DTexture() {
	std::vector<glm::vec3> ssaoNoise;
	std::uniform_real_distribution<GLfloat> randomFloats(0.0, 1.0);
	std::default_random_engine generator;
	for (unsigned int i = 0; i < 16; i++) {
		glm::vec3 noise(randomFloats(generator) * 2.0 - 1.0,
						randomFloats(generator) * 2.0 - 1.0,
						0.0f);
		ssaoNoise.push_back(noise);
	}

	glGenTextures(1, &noise_);
	glBindTexture(GL_TEXTURE_2D, noise_);
	glTexImage2D(GL_TEXTURE_2D, 0, 
			GL_RGB32F, 4, 4, 0, GL_RGB, GL_FLOAT, &ssaoNoise[0]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glBindTexture(GL_TEXTURE_2D, 0);
}

void SSAO::GenerateSSAOKernel() {
	std::uniform_real_distribution<GLfloat> randomFloats(0.0, 1.0);
	std::default_random_engine generator;
	for (unsigned int i = 0; i < 64; ++i) {
		glm::vec3 sample(randomFloats(generator) * 2.0 - 1.0,
						 randomFloats(generator) * 2.0 - 1.0,
						 randomFloats(generator));
		sample = glm::normalize(sample);
		sample *= randomFloats(generator);
		float scale = float(i) / 64.0;
		scale = glm::mix(0.1f, 1.0f, scale * scale);
		sample *= scale;
		ssao_kernel_.push_back(sample);
	}
}




}
}