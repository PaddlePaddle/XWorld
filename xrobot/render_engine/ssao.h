#ifndef RENDER_ENGINE_SSAO_H_
#define RENDER_ENGINE_SSAO_H_

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

enum SSAOShaders{
	kSSAO,
	kBlur,
	kSSAOShaders
};

class SSAO : public PostProcessing {
public:
	SSAO(const float width, const float height);
	~SSAO();

	void Draw(std::vector<GLuint>& in,
		      std::vector<GLuint>& out,
		      const glm::mat4 view,
		      const glm::mat4 projection);

private:

	void InitShaders() override;

	void GenerateNoise1DTexture();
	void GenerateSSAOKernel();

	void AOPass(const GLuint g_position, const GLuint g_normal, 
			const glm::mat4 view, const glm::mat4 projection);
	void BlurPass(const GLuint g_position);

	std::vector<glm::vec3> ssao_kernel_;
	GLuint noise_;

	std::shared_ptr<RenderTarget> ao_;
	std::shared_ptr<RenderTarget> blur_;
};

}}

#endif // RENDER_ENGINE_SSAO_H_
