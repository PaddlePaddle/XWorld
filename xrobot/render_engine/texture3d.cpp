#include "texture3d.h"

namespace xrobot {
namespace render_engine {

void Texture3D::active(const int shader, 
		const std::string sampler, const int unit) {
	glActiveTexture(GL_TEXTURE0 + unit);
	glBindTexture(GL_TEXTURE_3D, id_);
	glUniform1f(glGetUniformLocation(shader, sampler.c_str()), unit);
}

void Texture3D::clear() {
	glBindTexture(GL_TEXTURE_3D, id_);
	for(GLint i = 0; i < levels_; ++i)
		glClearTexImage(id_, i, format_, type_, &clear_data_[0]);
}

Texture3D::Texture3D(
		const int width, 
		const int height,
		const int depth,
		const int levels,
		const GLint internal_format,
	    const GLenum format,
	    const GLenum type,
		const GLint border) :  width_(width),
							   height_(height),
							   depth_(depth),
							   levels_(levels),
							   internal_format_(internal_format),
							   format_(format),
							   type_(type),
							   border_(border) {

	// RGBA8
	clear_data_ = std::vector<unsigned char>(width_ * height_ * depth_ * 4, 0);

	if(levels < 2) {
		min_filter_ = GL_LINEAR;
		mag_filter_ = GL_LINEAR;
	} else {
		min_filter_ = GL_LINEAR_MIPMAP_LINEAR;
		mag_filter_ = GL_LINEAR;
	}

	GLuint bound_id = 0;
    glGetIntegerv(GL_TEXTURE_BINDING_3D, (GLint*) &bound_id);

	glGenTextures(1, &id_);
	glBindTexture(GL_TEXTURE_3D, id_);

	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, min_filter_);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, mag_filter_);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, border_);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, border_);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, border_);

	glTexStorage3D(GL_TEXTURE_3D,
				   levels_, 
				   internal_format_,
				   width_,
				   height_,
				   depth_);

	int w = width_;
	int h = height_;
	int d = depth_;

	for(GLint i = 0; i < levels; ++i) {
		glTexImage3D(GL_TEXTURE_3D,
					 i,
					 internal_format_,
					 w,
					 h,
					 d,
					 0,
					 format_,
					 type_,
					 &clear_data_[0]);

		w = glm::max(1, (w >> 1));
		h = glm::max(1, (h >> 1));
		d = glm::max(1, (d >> 1));
	}

	if (levels > 1) 
		glGenerateMipmap(GL_TEXTURE_3D);

	glBindTexture(GL_TEXTURE_3D, bound_id);
}

Texture3D::~Texture3D() {
	glDeleteTextures(1, &id_);
}

}
}