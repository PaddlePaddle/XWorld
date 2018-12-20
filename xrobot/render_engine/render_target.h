#ifndef RENDER_ENGINE_RENDER_TARGET_H_
#define RENDER_ENGINE_RENDER_TARGET_H_

#include <vector>
#include <string>
#include <iostream>
#include "glm/glm.hpp"
#include "gl_header.h"

namespace xrobot {
namespace render_engine {

class RenderTarget {
public:
	GLuint id() const { return id_; }
	GLuint texture_id(const int target) const { return texture_ids_[target]; }

	void append(const GLenum mag_filter,
			    const GLenum min_filter,
				const GLint internal_format,
			    const GLenum format,
			    const GLenum type,
				const GLint border);


	void append_rgba_uint8();
	void append_bgra_uint8();
	void append_rgba_float32();
	void append_rgba_float16();
	void append_rgb_float32();
	void append_rgb_float16();
	void append_rgb_uint8();
	void append_rgb_float();
	void append_r_float();
	void append_r_uint8();
	void append_depth();
	void append_depth_readonly();

	void init();
	void active_depth(const int shader, const std::string sampler,
		const int unit = 0);
	void active(const int shader, const std::string sampler,
		const int target, const int unit = 0);

	RenderTarget(const int width, const int height);
	~RenderTarget();

private:
	GLuint bound_texture_id_;
	GLuint bound_framebuffer_id_;

	int depth_;
	GLuint depth_buffer_;

	int width_, height_;
	GLuint id_;
	std::vector<GLuint> texture_ids_;
	std::vector<GLuint> attachments_;
};

}
}

#endif // RENDER_ENGINE_RENDER_TARGET_H_