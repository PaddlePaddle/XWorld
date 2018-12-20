#ifndef RENDER_ENGINE_BLOOM_H_
#define RENDER_ENGINE_BLOOM_H_

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

enum BloomShaders{
	kHighlightExtract,
	kGaussianBlur,
	kBloomShaders
};

class Bloom : public PostProcessing {
public:
	Bloom(const float width, const float height);
	~Bloom();

	void Draw(std::vector<GLuint>& in,
		      std::vector<GLuint>& out);

private:

	void InitShaders() override;

	void ExtractHighlightPass();

	void GaussianBlurPass();

	std::shared_ptr<RenderTarget> bloom_;
};


}}

#endif // RENDER_ENGINE_BLOOM_H_
