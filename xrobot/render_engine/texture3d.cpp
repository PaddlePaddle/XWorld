#include "texture3d.h"

namespace xrobot {
namespace render_engine {

Texture3D::Texture3D(
	const std::vector<unsigned char> & textureBuffer,
	const int width, const int height, const int depth,
	const bool generateMipmaps, const int level
)
{
	this->width = width;
	this->height = height;
	this->depth = depth;
	this->levels = level;
	this->clearData = std::vector<unsigned char>(width * height * depth * 4, 0);

	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_3D, textureID);

	const auto wrap = GL_CLAMP_TO_EDGE;
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, wrap);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, wrap);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, wrap);

	if(level < 1)
	{
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	}
	else
	{
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	}
	
	glTexStorage3D(GL_TEXTURE_3D, level, GL_RGBA8, width, height, depth);
	int w = width;
	int h = height;
	int d = depth;

	for (size_t i = 0; i < level; ++i)
	{
		glTexImage3D(GL_TEXTURE_3D, i, GL_RGBA8, w, h, d, 0, GL_RGBA, GL_UNSIGNED_BYTE, &clearData[0]);
		w = glm::max(1, (w >> 1));
		h = glm::max(1, (h >> 1));
		d = glm::max(1, (d >> 1));
	}

	if (generateMipmaps) glGenerateMipmap(GL_TEXTURE_3D);

	//glBindTexture(GL_TEXTURE_3D, 0);
}

Texture3D::~Texture3D()
{
	glDeleteTextures(1, &textureID);
	delete textureBuffer;
	clearData.clear();
}

void Texture3D::Activate(const int shaderProgram, const std::string glSamplerName, const int textureUnit)
{
	glActiveTexture(GL_TEXTURE0 + textureUnit);
	glBindTexture(GL_TEXTURE_3D, textureID);
	glUniform1f(glGetUniformLocation(shaderProgram, glSamplerName.c_str()), textureUnit);
}

void Texture3D::Clear()
{
	glBindTexture(GL_TEXTURE_3D, textureID);
	//glTexSubImage3D(GL_TEXTURE_3D, 0, 0, 0, 0, width, height, depth, GL_RGBA, GL_UNSIGNED_BYTE, &clearData[0]);
	for(int i = 0; i < levels; ++i)
		glClearTexImage(textureID, i, GL_RGBA, GL_UNSIGNED_BYTE, &clearData[0]);

}

}
}