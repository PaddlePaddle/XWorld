#ifndef RENDER_ENGINE_COMPOSITE_H_
#define RENDER_ENGINE_COMPOSITE_H_

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

enum CompositeShaders{
	kComposite,
	kCompositeShaders
};

class Composite : public PostProcessing {
public:
	Composite(const float width, const float height);
	~Composite();

	void Draw(std::vector<GLuint>& in,
		      std::vector<GLuint>& out,
		      const bool ssao,
		      const bool ssr,
		      const bool shading);

private:
	void InitShaders() override;

	std::shared_ptr<RenderTarget> composite_;
};

}}

#endif // RENDER_ENGINE_COMPOSITE_H_
