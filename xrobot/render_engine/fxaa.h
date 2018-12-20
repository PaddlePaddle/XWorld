#ifndef RENDER_ENGINE_FXAA_H_
#define RENDER_ENGINE_FXAA_H_

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

enum FXAAShaders{
	kFXAA,
	kFXAAShaders
};

class FXAA : public PostProcessing {
public:
	FXAA(const float width, const float height);
	~FXAA();

	void Draw(std::vector<GLuint>& in,
		      std::vector<GLuint>& out);

private:

	void InitShaders() override;

	void FXAAPass(const GLuint color);

	std::shared_ptr<RenderTarget> aa_;
};


}}

#endif // RENDER_ENGINE_FXAO_H_
