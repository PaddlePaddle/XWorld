#ifndef RENDER_ENGINE_RENDER_H_
#define RENDER_ENGINE_RENDER_H_

#include <chrono>
#include <ctime>
#include <ratio>
#include <stdio.h>
#include <string>
#include <vector>

#include "fbo.h"
#include "gl_context.h"
#include "shader.h"
#include "texture3d.h"
#include "csm.h"
#include "render_world.h"

namespace xrobot {
namespace render_engine {

//
// Low Quality
// Lambert + Blinn-Phong + Ambient + Cubemap Reflection
//
// Normal Quality
// Lambert + Blinn-Phong + Ambient 
// Normal / Parrallex Mapping
// Shadow: 3 Cascades
//
// High Quality
// Cook-Torrance + IBL
// Normal / Parrallex Mapping
// VXGI + VXAO 256
// Shadow: 4 Cascades
//
// Very High Quality (Baked)
// Cook-Torrance + IBL
// Normal / Parrallex Mapping
// VXGI + VXAO 512
// Shadow: 5 Cascades
//

constexpr unsigned int kLowQuality = 0;
constexpr unsigned int kNormalQuality = 1;
constexpr unsigned int kHighQuality   = 2;
constexpr unsigned int kVeryHighQuality = 3;
constexpr float kMin = -1000.0f;
constexpr float kMax =  1000.0f;

typedef std::chrono::high_resolution_clock::time_point TimeFrame;

struct Image {
    Image() : data(0), camera_id(-1) {}

    std::vector<unsigned char> data;
    int camera_id;
};

struct RenderSettings {
    RenderSettings(const unsigned int quality = kLowQuality) : use_deferred(quality < 1 ? 0 : 1),
                                                               use_vct(quality < 2 ? 0 : 1),
                                                               vct_bake_before_simulate(quality < 3 ? false : true),
                                                               vct_resolution(quality < 3 ? 256 : 512),
                                                               use_shadow(quality < 1 ? 0 : 1),
                                                               shadow_split(quality + 3),
                                                               shadow_lamda(0.7f),
                                                               shadow_near_offset(50.0f),
                                                               shadow_resolution(2048),
                                                               use_basic_frustrum_culling(true),
                                                               use_free_camera(true) {}

    // Deferred Shading                                                           
    bool use_deferred;

    // Voxel Cone Tracing
    // Voxel Cone Tracing (VCT) is not very stable, especially in large scene
    bool  use_vct;
    bool  vct_bake_before_simulate; // Only for fully static scene
    int   vct_resolution; // Recommend 128 or 256

    // Physically Based Rendering
    bool  use_pbr;

    // Shadow
    bool  use_shadow;
    int   shadow_split;
    float shadow_lamda;
    float shadow_near_offset;
    int   shadow_resolution;

    // Culling
    bool  use_basic_frustrum_culling;

    // Others
    bool use_free_camera;
};

struct Lighting {
    Lighting() : exposure(1.3f),
                 indirect_strength(0.4f) {}
    // Lighting
    // exposure: change the brightness of the scene
    // indirect_strength: change the contribution of the indirect lighting
    float exposure; 
    float indirect_strength;

    // Shadow
    // shadow_bias: increase to reduce the shadow arc effect however 
    //              this could also detach the shadow
    // force_disable_shadow: disable directional shadow during rendering
    float shadow_bias_clamp = 0.0005f;
    float shadow_bias_scale = 0.0002f;
    bool force_disable_shadow = false;

    // VXGI
    // propagation_distance: increase to make directional light lit more area
    //                       however this could also leak the light
    // conetracing_distance: increase to make indirectional light lit more area
    // traceshadow_distance: shadow ray distance
    // boost_ambient: force the albedo contribute ambient lighting in VXGI
    // sample_factor: inc/dec for different size of scene
    // ibl_factor: control the contribution between IBL and VXGI
    // force_disable_progagation: disable the light progagation algorithm
    // linear_voxelize: convert albedo into linear space before voxelize
    float propagation_distance = 0.5f;
    float conetracing_distance = 1.0f;
    float traceshadow_distance = 0.3f;
    float boost_ambient = 0.02f;
    float sample_factor = 0.4f;
    float ao_falloff = 1000.0f;
    float ibl_factor = 0.0f;
    bool force_disable_propagation = false;
    bool linear_voxelize = false;
};

struct DirectionalLight {
    DirectionalLight() : direction(glm::vec3(1,2,1)),
                         diffuse(glm::vec3(0.8f, 0.8f, 0.8f)),
                         specular(glm::vec3(0.1f, 0.1f, 0.1f)),
                         ambient(glm::vec3(0.2, 0.2, 0.2)) {}

    glm::vec3  direction;
    glm::vec3  diffuse;
    glm::vec3  specular;
    glm::vec3  ambient;
};

class Render {
    enum ShaderTypes {
        kLambert = 0,
        kDepth = 1,
        kColor = 2,
        kLine = 3,
        kCrossHair = 4,
        kMultiply = 5,
        kVoxelize = 6,
        kVoxelVisualization = 7,
        kRadianceInjection = 8,
        kRadiancePropagation = 9,
        kMipMapBase = 10,
        kMipMapVolume = 11,
        kVoxelConeTracing = 12,
        kGBuffer = 13,
        kClearBuffer = 14,
        kPSSM = 15,
        kConvertToCubemap = 16,
        kIrrandianceConv = 17,
        kPrefilter = 18,
        kBRDF = 19,
        kCaptureCubemap = 20,
        kCubemapVisualization = 21,
        kCombineCubemap = 22,
        kRay = 23,
        kSkybox = 24,
        kFXAA = 25,
        kSSAO = 26,
        kBlur = 27,
        kComposite = 28,
        kShaderTypeSize
    };

public:
    // Framerate
    // This Could Be Removed In the Future...
    float current_framerate_;
    float max_framerate_;
    double avg_framerate_;
    double all_rendered_frames_;

    // Free Camera
    Camera free_camera_;
    float last_x_, last_y_;
    float delta_time_;
    bool first_mouse_;
    TimeFrame last_frame_;

    // Buffers
    GLuint crosshair_vao_;
    GLuint crosshair_vbo_;
    GLuint cube_vao_;
    GLuint cube_vbo_;
    GLuint quad_vao_;
    GLuint quad_vbo_;
    GLuint box_vao_;
    GLuint box_vbo_;

    // Deferred Shading
    GLuint g_buffer_fbo_;
    GLuint g_buffer_rbo_;
    GLuint g_position_;
    GLuint g_normal_;
    GLuint g_albedospec_;
    GLuint g_pbr_;

    // Directional Light
    bool use_sunlight_;
    DirectionalLight sunlight_;
    Lighting lighting_;

    // Cone Tracing
    glm::vec3 vct_bbox_min_ = glm::vec3(-2, -2, -2);
    glm::vec3 vct_bbox_max_ = glm::vec3(10, 10, 10);
    int volume_dimension_ = 256;
    float volume_grid_size_;
    float voxel_size_;
    int voxel_count_;
    glm::mat4 view_projection_matrix_[3];
    glm::mat4 view_projection_matrix_inv_[3];
    GLuint voxel_vao_;
    bool need_voxelize_;

    // Cone Tracing Textures
    Texture3D * voxel_albedo_;
    Texture3D * voxel_normal_;
    Texture3D * voxel_emissive_;
    Texture3D * voxel_radiance_;
    Texture3D * voxel_mipmaps_[6];

    // PSSM
    CSMUniforms csm_uniforms_;
    CSM csm_;
    int shadow_map_size_;
    int cascade_count_;
    float pssm_lamda_;
    float near_offset_;
    bool has_init_;

    // Shaders and Other Basic Rendering Variables
	std::vector<Shader> shaders_;
	float width_, height_;
    glm::vec3 background_color_;
    GLContext * ctx_;    
    RenderSettings settings_;

    // Frame Buffers
    FBO * free_camera_framebuffer_;
    std::vector<FBO*> camera_framebuffer_list_;
    std::vector<FBO*> multiplied_framebuffer_list_;

    // Cube Map
    GLuint reflection_map_;

    // I/O
    int pixel_buffer_index_;
    int pixel_buffer_next_index_;
    GLuint pixel_buffers_[2];
    int num_frames_;
    std::vector<Image> img_buffers_;


    // Post-Processing
    std::vector<glm::vec3> ssaoKernel;
    GLuint ssao_composite_fbo_;
    GLuint ssao_composite_tex_;
    GLuint noise_tex_;
    GLuint ssao_fbo_;
    GLuint ssao_blur_fbo_;
    GLuint ssao_tex_;
    GLuint ssao_tex_blur_;

    // High Res Lidar
    GLuint cubemap_capture_fbo_;
    GLuint cubemap_capture_rbo_;
    GLuint cubemap_texture_;

    GLuint combine_capture_fbo_;
    GLuint combine_texture_;

    // Batch Ray
    int num_rays;
    GLuint batch_ray_vao_;
    GLuint batch_ray_vbo_;

public: 
    Render(const int width,
           const int height,
           const int num_cameras,
           const RenderSettings render_profile,
           const bool headless);

    ~Render();

    void ProcessInput();

    void ProcessMouse();

    void RenderMarker();

    void RenderCube();

    void RenderBox();
    
    void RenderQuad();
    
    void InitFramebuffers(const unsigned int num_cameras);
    
    void Draw(RenderWorld* world, 
              const Shader& shader,
              const glm::vec3 aabb_min = glm::vec3(kMin),
              const glm::vec3 aabb_max = glm::vec3(kMax),
              const bool skip_robot = false);

    void DrawRootAABB(RenderWorld* world, const Shader& shader);

    void InitDrawBatchRay(const int rays);

    void UpdateRay(const int offset, const glm::vec3 from, const glm::vec3 to);

    void DrawBatchRay();

    //
    //void DrawEmptyAABB(Map* map, const Shader& shader);
    
    //void DrawRoomAABB(Map* map, const Shader& shader); 

    void StepRenderFreeCamera(RenderWorld* world);

    void StepRenderAllCameras(
            RenderWorld* world, std::vector<int>& picks, const bool color_pick);

    void StepRenderAllCamerasDeferred(
            RenderWorld* world, std::vector<int>& picks, const bool color_pick);

    void StepRenderGetDepthAllCameras();

    void StepRenderShadowMaps(RenderWorld * world, Camera * camera);

    void Visualization();

    void VisualizeVoxels(const int draw_mip_level = 0);

    int StepRender(RenderWorld* map, int pick_camera_id = -1);

    int StepRender2(RenderWorld* map, int pick_camera_id = -1);
    
    std::vector<Image> GetRenderedImages() {
        return img_buffers_;
    }

    void InitSSAO();

    void SSAO(RenderWorld* world);

    void InitShaders();

    void Init(Camera * camera);
 
    void InitCombineTexture();

    void StepCombineTexture();

    void StepLidarVisualization();

    void InitCaptureCubemap();

    void StepRenderCubemap(RenderWorld* world);

    void BakeScene(RenderWorld* world);

private:

    void VoxelConeTracing(RenderWorld* world, Camera * camera);

    void InjectRadiance();

    void PropagateRadiance();

    void GenerateMipmapBase();

    void GenerateMipmapVolume();

    void ClearTextures();

    void UpdateProjectionMatrices(RenderWorld* world);

    void VoxelizeScene(RenderWorld* world);

    void VoxelizeLocalScene(RenderWorld* world, Camera * camera);

    void InitVoxelization();

    void InitGBuffer();

    void StepRenderGBuffer(RenderWorld* world, Camera * camera);

    void GetViewFrusrumBoundingVolume(const glm::vec3 eye,
        const glm::vec3 front, const float far, const float aspect,
        const float fov, const glm::mat4 view_matrix,
        glm::vec3& aabb_min, glm::vec3& aabb_max);

    void GetViewFrusrumBoundingVolume(Camera* camera, 
        glm::vec3& aabb_min, glm::vec3& aabb_max);

    void UpdateCascadeShadowMap();

    void InitCascadeShadowMap(Camera * camera);

    void InitPBO();

    void InitReflectionMap();

    void IdToColor(const int id, int& r, int& g, int& b) {
        r = (id & 0x000000FF) >> 0;
        g = (id & 0x0000FF00) >> 8;
        b = (id & 0x00FF0000) >> 16;
    }

    int ColorToId(const int r, const int g, const int b) {
        return r + g * 256 + b * 256 * 256;
    }

    void GetDeltaTime();

    void GetFrameRate();

    std::string get_pwd(const std::string& file) {
        size_t p = file.find_last_of("/");
        return file.substr(0, p);
    }
};

} } // xrobot::render_engine

#endif // RENDER_ENGINE_RENDER_H_
