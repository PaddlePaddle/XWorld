#include "texture2d.h"

namespace xrobot {
namespace render_engine {

void Texture2D::active(const int shader, 
		const std::string sampler, const int unit) {
	glActiveTexture(GL_TEXTURE0 + unit);
	glBindTexture(GL_TEXTURE_2D, id_);
	glUniform1f(glGetUniformLocation(shader, sampler.c_str()), unit);
}

Texture2D::Texture2D(
		const int width, 
		const int height,
		const GLenum mag_filter,
	    const GLenum min_filter,
		const GLint internal_format,
	    const GLenum format,
	    const GLenum type,
		const GLint border) :  width_(width),
							   height_(height),
							   min_filter_(min_filter),
							   mag_filter_(mag_filter),
							   internal_format_(internal_format),
							   format_(format),
							   type_(type),
							   border_(border) {

	// TODO: Upgrade to Direct State Access
	GLuint bound_id = 0;
    glGetIntegerv(GL_TEXTURE_BINDING_2D, (GLint*) &bound_id);

	glGenTextures(1, &id_);
	glBindTexture(GL_TEXTURE_2D, id_);

	glTexImage2D(GL_TEXTURE_2D,
				 0,
				 internal_format_,
				 width_,
				 height_,
				 0,
				 format_,
				 type_,
				 NULL);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, min_filter_);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, mag_filter_);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, border_);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, border_);

	glBindTexture(GL_TEXTURE_2D, bound_id);
}

Texture2D::~Texture2D() {
	glDeleteTextures(1, &id_);
}

}
}