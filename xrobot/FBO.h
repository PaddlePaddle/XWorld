#pragma once

#include <vector>
#include <string>
#include <iostream>
#include "gl_header.h"
class FBO
{
public:
	GLuint width, height;
	GLuint frameBuffer;
	GLuint textureColorBuffer, textureDepthBuffer, textureSigBuffer, rbo;
	GLuint attachement;
	bool needDepthBuffer;
    bool needSigBuffer;

	void ActivateAsTexture(const int shaderProgram, const std::string glSamplerName, const int textureUnit = 0);
	void ActivateDepthAsTexture(const int shaderProgram, const std::string glSamplerName, const int textureUnit = 0);
	void ActivateSgAsTexture(const int shaderProgram, const std::string glSamplerName, const int textureUnit = 0);
    
	FBO(
		GLuint w, GLuint h, bool depthBuffer = false, bool sigBuffer = false,
		GLenum magFilter = GL_NEAREST, GLenum minFilter = GL_NEAREST,
		GLint internalFormat = GL_RGBA, GLint format = GL_UNSIGNED_BYTE,
		GLint wrap = GL_REPEAT
	);

	~FBO();
};
