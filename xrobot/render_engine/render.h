#ifndef RENDER_ENGINE_RENDER_H_
#define RENDER_ENGINE_RENDER_H_

#define EXPERIMENTAL_LIDAR

#include <chrono>
#include <ctime>
#include <ratio>
#include <stdio.h>
#include <string>
#include <vector>

#include "gl_context.h"
#include "shader.h"
#include "texture2d.h"
#include "render_target.h"
#include "csm.h"
#include "vct.h"
#include "composite.h"
#include "ssao.h"
#include "ssr.h"
#include "fxaa.h"
#include "frustum.h"
#include "capture.h"
#include "visualization.h"
#include "render_world.h"

namespace xrobot {
namespace render_engine {

enum RenderShaders {
    kGeometry,
    kPSSM,
    kFlat,
    kAlpha,
    kBlinn,
    kHUD,
    kRenderShaders
};

class Render {
public:
    Render(const int width,
           const int height,
           const Profile profile,
           const bool headless = false,
           const int device = 0);

    ~Render();

    GLContext* GetContext() { return ctx_; }

    // Render
    void StepRender(RenderWorld* world);

    // Visualization
    void InitDrawBatchRays(const int rays);
    void UpdateRay(const int offset, const glm::vec3 from, const glm::vec3 to);

    // IO
    Image<unsigned char> GetRenderedImage() const { return render_image_; }
    Image<float>         GetLidarImage() const { return lidar_image_; };

    // Other
    void BakeGI(RenderWorld* world);
    void UpdateExposure(const float exposure) { lighting_.exposure = exposure; }
    void UpdateBackgroundColor(const glm::vec3 rgb) { lighting_.bg = rgb; }
    void UpdateAmbientColor(const glm::vec3 ambient) { 
        dir_light_.ambient = ambient; 
    }
    void UpdateLightColor(const glm::vec3 color) {
        dir_light_.diffuse = color;
    }
    void UpdateLightDirection(const glm::vec3 direction) {
        dir_light_.direction = direction;
    }

private:

    // PostProcessing
    void InitPostProcessing();
    void RenderPostProcessingPasses(Camera* camera, 
            const std::vector<GLuint>& in, GLuint& out);

    // VCT
    void InitVCT();

    // PBOs
    void InitPBOs();

    // Shaders
    void InitShaders();

    // Deferred
    void RenderQuad();
    void RenderGeometryPass(RenderWorld* world, Camera* camera);
    void RenderLightingPass(RenderWorld* world, Camera* camera, GLuint& rgb);
    void RenderNormalShading(Camera* camera);
    void Draw(RenderWorld* world, const Shader& shader, const bool cull = false);

    // HUD
    void RenderHUD(RenderWorld* world, 
            Camera* camera, const GLuint rgb, const GLuint depth);

    // Shadow
    void InitShadowMaps(const float fov);
    void RenderShadowMaps(RenderWorld* world, Camera* camera);

    // Visualization
    void InitVisualization();
    void RenderVisualization(RenderWorld* world, Camera* camera);
    void RenderTexture(const GLuint id, const int width, const int height,
            const int x = 0, const int y = 0, const bool display_alpha = false,
            const int channel = 3);

    // Capture
    void InitLidarCapture();
    void RenderLidarCapture(RenderWorld* world, Camera* camera, GLuint& depth);

    // Basic
    Profile profile_;
    Lighting lighting_;

    GLContext * ctx_;
    std::vector<Shader> shaders_;
    float width_, height_;
    GLuint camera_pbos_[2];
    GLuint lidar_pbo_;
    GLuint quad_vao_, quad_vbo_;

    // Culling
    std::shared_ptr<CameraFrustum> frustum_;

    // IO
    Image<unsigned char> render_image_;
    Image<float> lidar_image_;

    // Lighting
    DirectionalLight dir_light_;

    // Deferred
    std::shared_ptr<RenderTarget> geomtry_pass_;
    std::shared_ptr<RenderTarget> lighting_pass_;
    std::shared_ptr<RenderTarget> hud_pass_;

    // Shadow
    PSSM shadow_;

    // Others
    std::shared_ptr<VCT>  vct_;
    std::shared_ptr<SSAO> ssao_;
    std::shared_ptr<SSR>  ssr_;
    std::shared_ptr<FXAA> fxaa_;
    std::shared_ptr<Composite> composite_;

    // Visualization
    std::shared_ptr<Visualization> visualize_;

    // Experimental
    std::shared_ptr<Capture> capture_;
};

}
}

#endif // RENDER_ENGINE_RENDER_H_
