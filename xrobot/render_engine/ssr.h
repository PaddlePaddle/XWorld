#ifndef RENDER_ENGINE_SSR_H_
#define RENDER_ENGINE_SSR_H_

#include <string>
#include <vector>
#include <random>

#include "common.h"
#include "shader.h"
#include "texture2d.h"
#include "render_target.h"
#include "postprocess.h"

namespace xrobot {
namespace render_engine {

enum SSRShaders{
	kSSR,
	kSSRCopy,
	kSSRShaders
};

class SSR : public PostProcessing {
public:
	SSR(const float width, const float height);
	~SSR();

	void Draw(std::vector<GLuint>& in,
		      std::vector<GLuint>& out, 
		      const glm::mat4 view,
		      const glm::mat4 projection);

private:

	void InitShaders() override;

	void CopyTexture(const GLuint color);

	void SSRPass(const GLuint color, 
				 const GLuint g_position,
				 const GLuint g_normal,
				 const glm::mat4 view,
				 const glm::mat4 projection);

	std::shared_ptr<RenderTarget> ssr_;
	GLuint color_mipmaps_;
	GLuint color_mipmaps_fb_;
};


}}

#endif // RENDER_ENGINE_SSR_H_
