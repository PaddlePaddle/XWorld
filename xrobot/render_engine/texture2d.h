#ifndef RENDER_ENGINE_TEXTURE_2D_H_
#define RENDER_ENGINE_TEXTURE_2D_H_

#include <vector>
#include <string>
#include "glm/glm.hpp"
#include "gl_header.h"

namespace xrobot {
namespace render_engine {

class Texture2D
{
public:

	GLuint id() const { return id_; }
	void active(const int shader,
			const std::string sampler, const int unit = 0);

	Texture2D(
		const int width, 
		const int height,
		const GLenum mag_filter = GL_NEAREST,
	    const GLenum min_filter = GL_NEAREST,
		const GLint internal_format = GL_RGBA8,
	    const GLenum format = GL_RGBA,
	    const GLenum type = GL_UNSIGNED_BYTE,
		const GLint border = GL_CLAMP_TO_EDGE);
	~Texture2D();

private:
	int width_, height_;
	GLuint id_;
	GLenum min_filter_, mag_filter_, format_, type_;
	GLint internal_format_, border_;
};
}
}

#endif // RENDER_ENGINE_TEXTURE_2D_H_