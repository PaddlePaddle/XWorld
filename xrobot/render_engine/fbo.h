#ifndef RENDER_ENGINE_FBO_H
#define RENDER_ENGINE_FBO_H

#include <iostream>
#include <string>
#include <vector>

#define INCLUDE_GL_CONTEXT_HEADERS
#include "gl_header.h"

namespace xrobot {
namespace render_engine {

class FBO {
public:
	GLuint width;
    GLuint height;

	GLuint frameBuffer;

	GLuint textureColorBuffer;
    GLuint textureDepthBuffer;
    GLuint textureSigBuffer;

    GLuint rbo;
    
	GLuint attachement;

	bool needDepthBuffer;

    bool needSigBuffer;

public:
    FBO(GLuint w,
        GLuint h,
        bool depthBuffer = false,
        bool sigBuffer = false,
		GLenum magFilter = GL_NEAREST,
        GLenum minFilter = GL_NEAREST,
		GLint internalFormat = GL_RGBA,
        GLint format = GL_UNSIGNED_BYTE,
		GLint wrap = GL_REPEAT);

	~FBO();

	void ActivateAsTexture(const int shaderProgram,
                           const std::string& glSamplerName,
                           const int textureUnit = 0);

	void ActivateDepthAsTexture(const int shaderProgram,
                                const std::string& glSamplerName,
                                const int textureUnit = 0);

	void ActivateSgAsTexture(const int shaderProgram,
                             const std::string& glSamplerName,
                             const int textureUnit = 0);
    
};

} } // xrobot::render_engine

#endif
