#ifndef RENDER_ENGINE_VCT_H_
#define RENDER_ENGINE_VCT_H_

#include <chrono>
#include <ctime>
#include <ratio>
#include <stdio.h>
#include <string>
#include <vector>

#include "gl_context.h"
#include "common.h"
#include "shader.h"
#include "csm.h"
#include "texture2d.h"
#include "texture3d.h"
#include "render_target.h"
#include "render_world.h"

namespace xrobot {
namespace render_engine {

constexpr int kVoxelDimension = 256;
constexpr float kScenePadding = 1.0f;
constexpr float kSceneHeightMin = -2.0f;
constexpr float kSceneHeightMax = 10.0f;
constexpr float kTraceShadowDistance = 0.3f;
constexpr float kPropagationDistance = 0.5f;
constexpr float kConeTracingDistance = 1.0f;
constexpr float kSampleFactor = 0.5f;
constexpr float kAOFalloff = 1000.0f;
constexpr float kGIStrength = 0.4f;

enum VCTShaders {
    kVoxelize,
    kRadianceInjection,
    kRadiancePropagation,
    kMipMapBase,
    kMipMapVolume,
    kClear,
    kConeTracing,
    kVCTShaders
};


class VCT {
public:
	VCT(const int width, const int height);
	~VCT();

    void BakeGI(RenderWorld* world,
                const DirectionalLight& light,
                const bool force = false);

    void ConeTracing(RenderWorld* world,
                     Camera* camera,
                     DirectionalLight& light,
                     PSSM& shadow,
                     const float exposure,
                     std::shared_ptr<RenderTarget> gbuffer,
                     GLuint& out);
    void Visualize(const int level) =delete;

private:
    void InitShaders();
    void Draw(RenderWorld* world, const Shader& shader);
    void InjectRadiance(const DirectionalLight& light);
    void PropagateRadiance();
    void GenerateMipmapBase();
    void GenerateMipmapVolume();
    void ClearTextures();
    void UpdateProjectionMatrices(RenderWorld* world);
    void InitVoxelization();
    void VoxelizeScene(RenderWorld* world);
    void RenderQuad();

public:
    // Lighting
    float ambient_;

	// Tracing Area
    glm::vec3 scene_min_;
    glm::vec3 scene_max_;

    // Basic
    int width_, height_;
    bool baked_;
    int voxel_count_;
    int volume_dimension_;
    float volume_grid_size_;
    float voxel_size_;
    glm::mat4 view_projection_matrix_[3];
    glm::mat4 view_projection_matrix_inv_[3];
    GLuint voxel_vao_;
    GLuint quad_vao_, quad_vbo_;
    std::vector<Shader> shaders_;
    std::shared_ptr<RenderTarget> vct_out_;

    // Buffers
    std::shared_ptr<Texture3D> voxel_albedo_;
    std::shared_ptr<Texture3D> voxel_normal_;
    std::shared_ptr<Texture3D> voxel_emissive_;
    std::shared_ptr<Texture3D> voxel_radiance_;
    std::shared_ptr<Texture3D> voxel_mipmaps_[6];
};

}
}

#endif // RENDER_ENGINE_VCT_H_
