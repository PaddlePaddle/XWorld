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

	GLuint textureID;
	unsigned char * textureBuffer = nullptr;

	void Activate(const int shaderProgram, const std::string glSamplerName, const int textureUnit = 0);
	void Clear();

	Texture3D(
		const std::vector<unsigned char> & textureBuffer,
		const int width, const int height, const int depth,
		const bool generateMipmaps = false, const int level = 1
	);
	~Texture3D();

private:
	int width, height, depth;
	int levels;
	std::vector<unsigned char> clearData;
};
}
}

#endif // RENDER_ENGINE_TEXTURE_3D_H_