#include "render_target.h"

namespace xrobot {
namespace render_engine {

void RenderTarget::active(const int shader, const std::string sampler,
		const int target, const int unit) {
	glActiveTexture(GL_TEXTURE0 + unit);
	glBindTexture(GL_TEXTURE_2D, texture_ids_[target]);
	glUniform1f(glGetUniformLocation(shader, sampler.c_str()), unit);
}

void RenderTarget::active_depth(const int shader, const std::string sampler,
		const int unit) {
	glActiveTexture(GL_TEXTURE0 + unit);
	glBindTexture(GL_TEXTURE_2D, depth_buffer_);
	glUniform1f(glGetUniformLocation(shader, sampler.c_str()), unit);
}

RenderTarget::RenderTarget(const int width, 
						   const int height) : depth_(0),
											   width_(width),
						   					   height_(height),
						   					   texture_ids_(0),
						   					   attachments_(0) {

	glGetIntegerv(GL_FRAMEBUFFER_BINDING, (GLint*) &bound_framebuffer_id_);		   					   	
    glGetIntegerv(GL_TEXTURE_BINDING_2D, (GLint*) &bound_texture_id_);

    glGenFramebuffers(1, &id_);
	glBindFramebuffer(GL_FRAMEBUFFER, id_);
}

RenderTarget::~RenderTarget() {
	glDeleteFramebuffers(1, &id_);

	if(depth_ == 1)
		glDeleteRenderbuffers(1, &depth_buffer_);
	if(depth_ == 2)
		glDeleteTextures(1, &depth_buffer_);

	for(auto& texture_id_ : texture_ids_)
		glDeleteTextures(1, &texture_id_);
}

void RenderTarget::append_rgba_uint8() {
	append(GL_NEAREST, GL_NEAREST, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE,
			GL_CLAMP_TO_EDGE);
}

void RenderTarget::append_bgra_uint8() {
	append(GL_NEAREST, GL_NEAREST, GL_RGBA8, GL_BGRA, GL_UNSIGNED_BYTE,
			GL_CLAMP_TO_EDGE);
}

void RenderTarget::append_rgba_float32() {
	append(GL_NEAREST, GL_NEAREST, GL_RGBA32F, GL_RGBA, GL_FLOAT,
			GL_CLAMP_TO_EDGE);
}

void RenderTarget::append_rgba_float16() {
	append(GL_NEAREST, GL_NEAREST, GL_RGBA16F, GL_RGBA, GL_FLOAT,
			GL_CLAMP_TO_EDGE);
}

void RenderTarget::append_rgb_float32() {
	append(GL_NEAREST, GL_NEAREST, GL_RGB32F, GL_RGB, GL_FLOAT,
			GL_CLAMP_TO_EDGE);
}

void RenderTarget::append_rgb_float16() {
	append(GL_NEAREST, GL_NEAREST, GL_RGB16F, GL_RGB, GL_FLOAT,
			GL_CLAMP_TO_EDGE);
}

void RenderTarget::append_rgb_uint8() {
	append(GL_NEAREST, GL_NEAREST, GL_RGB8, GL_RGB, GL_UNSIGNED_BYTE,
			GL_CLAMP_TO_EDGE);
}

void RenderTarget::append_rgb_float() {
	append(GL_NEAREST, GL_NEAREST, GL_RGB32F, GL_RGB, GL_FLOAT,
			GL_CLAMP_TO_EDGE);
}

void RenderTarget::append_r_float() {
	append(GL_NEAREST, GL_NEAREST, GL_R32F, GL_RGB, GL_FLOAT,
			GL_CLAMP_TO_EDGE);
}

void RenderTarget::append_r_uint8() {
	append(GL_NEAREST, GL_NEAREST, GL_RED, GL_RGB, GL_UNSIGNED_BYTE,
			GL_CLAMP_TO_EDGE);
}

void RenderTarget::append_depth() {
	if(depth_ > 0)
		return;

	depth_ = 2;
	glGenTextures(1, &depth_buffer_);
	glBindTexture(GL_TEXTURE_2D, depth_buffer_);
	glTexImage2D(GL_TEXTURE_2D,
                 0,
                 GL_DEPTH_COMPONENT24,
                 width_,
                 height_,
                 0,
                 GL_DEPTH_COMPONENT,
                 GL_FLOAT,
                 0);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_ALPHA);
	glFramebufferTexture2D(GL_FRAMEBUFFER,
                           GL_DEPTH_ATTACHMENT,
                           GL_TEXTURE_2D,
                           depth_buffer_,
                           0);
}

void RenderTarget::append_depth_readonly() {
	if(depth_ > 0)
		return;

	depth_ = 1;
	glGenRenderbuffers(1, &depth_buffer_);
	glBindRenderbuffer(GL_RENDERBUFFER, depth_buffer_);
	glRenderbufferStorage(GL_RENDERBUFFER, 
			GL_DEPTH_COMPONENT24, width_, height_);

	glFramebufferRenderbuffer(GL_FRAMEBUFFER, 
			GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_buffer_);
}

void RenderTarget::append(const GLenum mag_filter,
					      const GLenum min_filter,
						  const GLint internal_format,
					      const GLenum format,
					      const GLenum type,
						  const GLint border) {

	GLuint texture_id;
	glGenTextures(1, &texture_id);
    glBindTexture(GL_TEXTURE_2D, texture_id);

    glTexImage2D(GL_TEXTURE_2D,
                 0,
                 internal_format,
                 width_,
                 height_,
                 0,
                 format,
                 type,
                 NULL);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, min_filter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, mag_filter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, border);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, border);

    glFramebufferTexture2D(GL_FRAMEBUFFER,
                           GL_COLOR_ATTACHMENT0 + texture_ids_.size(),
                           GL_TEXTURE_2D,
                           texture_id,
                           0);

    attachments_.push_back(GL_COLOR_ATTACHMENT0 + texture_ids_.size());
    texture_ids_.push_back(texture_id);
}

void RenderTarget::init() {
    glDrawBuffers(attachments_.size(), attachments_.data());

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
		std::cout << "FBO failed to initialize correctly." << std::endl;

	glBindFramebuffer(GL_FRAMEBUFFER, bound_framebuffer_id_);
    glBindTexture(GL_TEXTURE_2D, bound_texture_id_);
}

}
}