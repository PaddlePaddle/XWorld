#include "fbo.h"

namespace xrobot {
namespace render_engine {

FBO::FBO(GLuint w,
         GLuint h,
         bool depthBuffer,
         bool sigBuffer,
         GLenum magFilter,
         GLenum minFilter,
         GLint internalFormat,
         GLint format,
         GLint wrap) {
	this->width = w;
	this->height = h;
	this->needDepthBuffer = depthBuffer;
    this->needSigBuffer = sigBuffer;

	GLint previousFrameBuffer;
	glGetIntegerv(GL_FRAMEBUFFER_BINDING, &previousFrameBuffer);

	glGenFramebuffers(1, &frameBuffer);
	glBindFramebuffer(GL_FRAMEBUFFER, frameBuffer);

	glGenTextures(1, &textureColorBuffer);
	glBindTexture(GL_TEXTURE_2D, textureColorBuffer);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, magFilter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap);

	glTexImage2D(GL_TEXTURE_2D,
                 0,
                 internalFormat,
                 w,
                 h,
                 0,
                 GL_RGBA,
                 format,
                 NULL);
	glFramebufferTexture2D(GL_FRAMEBUFFER,
                           GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D,
                           textureColorBuffer,
                           0);

	if (depthBuffer == false) {
		glGenRenderbuffers(1, &rbo);
		glBindRenderbuffer(GL_RENDERBUFFER, rbo);
        // Use a single rbo for both depth and stencil buffer.
		glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, w, h);
		glFramebufferRenderbuffer(
                GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rbo);

		unsigned int attachments[1] = { GL_COLOR_ATTACHMENT0 };
		glDrawBuffers(1, attachments);
	} else {
		glGenTextures(1, &textureDepthBuffer);
		glBindTexture(GL_TEXTURE_2D, textureDepthBuffer);
		glTexImage2D(GL_TEXTURE_2D,
                     0,
                     GL_DEPTH_COMPONENT24,
                     w,
                     h,
                     0,
                     GL_DEPTH_COMPONENT,
                     GL_FLOAT,
                     0);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_ALPHA);
		glFramebufferTexture2D(GL_FRAMEBUFFER,
                               GL_DEPTH_ATTACHMENT,
                               GL_TEXTURE_2D,
                               textureDepthBuffer,
                               0);
	}
    
    if (sigBuffer) {
        glGenTextures(1, &textureSigBuffer);
        glBindTexture(GL_TEXTURE_2D, textureSigBuffer);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, magFilter);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap);
        glTexImage2D(GL_TEXTURE_2D,
                     0,
                     internalFormat,
                     w,
                     h,
                     0,
                     GL_RGBA,
                     format,
                     NULL);
        glFramebufferTexture2D(GL_FRAMEBUFFER,
                               GL_COLOR_ATTACHMENT1,
                               GL_TEXTURE_2D,
                               textureSigBuffer,
                               0);
        
        unsigned int attach[2] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
        glDrawBuffers(2, attach);
    }

	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
		std::cout << "FBO failed to initialize correctly." << std::endl;
	}
}

FBO::~FBO() {
	glDeleteTextures(1, &textureColorBuffer);
	glDeleteFramebuffers(1, &frameBuffer);
	if (needDepthBuffer == false) {
        glDeleteRenderbuffers(1, &rbo);
    } else {
        glDeleteTextures(1, &textureDepthBuffer);
    }
    if (needSigBuffer) {
        glDeleteTextures(1, &textureSigBuffer);
    }
}

void FBO::ActivateSgAsTexture(const int shaderProgram,
                              const std::string& glSamplerName,
                              const int textureUnit) {
    if (!this->needSigBuffer) { return; }
    glActiveTexture(GL_TEXTURE0 + textureUnit);
    glBindTexture(GL_TEXTURE_2D, textureSigBuffer);
    GLuint loc = glGetUniformLocation(shaderProgram, glSamplerName.c_str());
    glUniform1i(loc, textureUnit);
}


void FBO::ActivateAsTexture(const int shaderProgram,
                            const std::string& glSamplerName,
                            const int textureUnit) {
	glActiveTexture(GL_TEXTURE0 + textureUnit);
	glBindTexture(GL_TEXTURE_2D, textureColorBuffer);
    GLuint loc = glGetUniformLocation(shaderProgram, glSamplerName.c_str());
	glUniform1i(loc, textureUnit);
}

void FBO::ActivateDepthAsTexture(const int shaderProgram,
                                 const std::string& glSamplerName,
                                 const int textureUnit) {
	if (!this->needDepthBuffer) { return; }
	glActiveTexture(GL_TEXTURE0 + textureUnit);
	glBindTexture(GL_TEXTURE_2D, textureDepthBuffer);
    GLuint loc = glGetUniformLocation(shaderProgram, glSamplerName.c_str());
	glUniform1i(loc, textureUnit);
}

} } // xrobot::render_engine
