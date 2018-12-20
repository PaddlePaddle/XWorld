#ifndef RENDER_ENGINE_TEXTURE_3D_H_
#define RENDER_ENGINE_TEXTURE_3D_H_

#include <vector>
#include <string>
#include "glm/glm.hpp"
#include "gl_header.h"

namespace xrobot {
namespace render_engine {

class Texture3D
{
public:

	GLuint id() const { return id_; }
	void set_clear_data(const std::vector<unsigned char>& data) {
		clear_data_ = data;
	}

	void clear();
	void active(const int shader,
			const std::string sampler, const int unit = 0);

	Texture3D(
		const int width, 
		const int height,
		const int depth,
		const int levels = 1,
		const GLint internal_format = GL_RGBA8,
	    const GLenum format = GL_RGBA,
	    const GLenum type = GL_UNSIGNED_BYTE,
		const GLint border = GL_CLAMP_TO_EDGE);
	~Texture3D();

private:
	int width_, height_, depth_, levels_;
	GLuint id_;
	GLenum min_filter_, mag_filter_, format_, type_;
	GLint internal_format_, border_;
	std::vector<unsigned char> clear_data_;
};
}
}

#endif // RENDER_ENGINE_TEXTURE_3D_H_