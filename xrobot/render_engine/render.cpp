//#define DEBUG

#include <unistd.h>

#include "glm/glm.hpp"

#include "render.h"
#include "render_utils.h"

namespace xrobot {
namespace render_engine {

Render::Render(const int width,
               const int height,
               const int num_cameras,
               const RenderSettings render_profile,
               const bool headless) : current_framerate_(0.0f),
                                      max_framerate_(0.0f),
                                      avg_framerate_(0.0),
                                      all_rendered_frames_(0.0),
                                      free_camera_(),
                                      last_x_(0.0f),
                                      last_y_(0.0f),
                                      delta_time_(0.0f),
                                      first_mouse_(true), 
                                      last_frame_(),
                                      crosshair_vao_(0),
                                      crosshair_vbo_(0),
                                      cube_vao_(0),
                                      cube_vbo_(0),
                                      quad_vao_(0),
                                      quad_vbo_(0),
                                      box_vao_(0),
                                      box_vbo_(0),
                                      g_buffer_fbo_(0),
                                      g_buffer_rbo_(0),
                                      g_position_(0),
                                      g_normal_(0),
                                      g_albedospec_(0),
                                      g_pbr_(0),
                                      capture_fbo_(0),
                                      capture_rbo_(0),
                                      hdr_map_(0),
                                      environment_map_(0),
                                      irradiance_map_(0),
                                      prefilter_map_(0),
                                      brdf_lut_map_(0),
                                      use_sunlight_(true),
                                      sunlight_(),
                                      lighting_(),
                                      vct_bbox_min_(glm::vec3(-2, -2, -2)),
                                      vct_bbox_max_(glm::vec3(10, 10, 10)),
                                      volume_dimension_(render_profile.vct_resolution),
                                      volume_grid_size_(0),
                                      voxel_size_(0),
                                      voxel_count_(0),
                                      voxel_vao_(0),
                                      voxel_albedo_(nullptr),
                                      voxel_normal_(nullptr),
                                      voxel_emissive_(nullptr),
                                      voxel_radiance_(nullptr),
                                      voxel_mipmaps_{nullptr, nullptr, nullptr, nullptr, nullptr, nullptr},
                                      csm_uniforms_(),
                                      csm_(),
                                      shadow_map_size_(render_profile.shadow_resolution),
                                      cascade_count_(render_profile.shadow_split),
                                      pssm_lamda_(render_profile.shadow_lamda),
                                      near_offset_(render_profile.shadow_near_offset),
                                      settings_(render_profile),
                                      shaders_(0),
                                      width_(width),
                                      height_(height), 
                                      background_color_(glm::vec3(0.5f, 0.5f, 0.5f)),
                                      ctx_(nullptr),
                                      free_camera_framebuffer_(nullptr),
                                      camera_framebuffer_list_(num_cameras),
                                      multiplied_framebuffer_list_(num_cameras),
                                      reflection_map_(0),
                                      pixel_buffer_index_(0),
                                      pixel_buffer_next_index_(1),
                                      pixel_buffers_{0, 0},
                                      num_frames_(0),
                                      img_buffers_(0),
                                      has_init_(false),
                                      need_voxelize_(true) {

    if(headless) {
        ctx_ = render_engine::CreateHeadlessContext(height, width);
    } else {
        #ifdef DEBUG
        ctx_ = render_engine::CreateContext(height, width * 2);
        #else
        ctx_ = render_engine::CreateContext(height, width);
        #endif
    }

    free_camera_framebuffer_ = new FBO(width, height, false, false);

    free_camera_ = Camera(glm::vec3(0.0f, 0.0f, 0.0f));
    free_camera_.Front = glm::vec3(1, 0, 0);
    free_camera_.Aspect = (float)width / height;
    free_camera_.Near = 0.05f;
    free_camera_.Far = 120.0f;
    
    last_x_ = width / 2.0f;
    last_y_ = height / 2.0f;

    view_projection_matrix_[0] = glm::mat4(1);
    view_projection_matrix_[1] = glm::mat4(1);
    view_projection_matrix_[2] = glm::mat4(1);
    view_projection_matrix_inv_[0] = glm::mat4(1);
    view_projection_matrix_inv_[1] = glm::mat4(1);
    view_projection_matrix_inv_[2] = glm::mat4(1);
    
    InitFXAA();
    InitSSAO();
    InitPBO();
    InitShaders();
    InitFramebuffers(num_cameras);

    // Lidar Simulation...
    // InitCaptureCubemap();
    // InitCombineTexture();

    if(!settings_.use_deferred)
    {
        InitReflectionMap(); // Forward
    }
    else
    {
        InitGBuffer(); // Deferred

        if(settings_.use_vct)
        {
            InitVoxelization(); // VCT
        }

        if(settings_.use_pbr || settings_.use_vct)
        {
            if(settings_.use_ibl)
            {
                InitIBL(settings_.ibl_path);
            }
        }
    }
    
}

void Render::Init(Camera * camera)
{
    if(settings_.use_shadow && !has_init_ && !lighting_.force_disable_shadow)
    {
        InitCascadeShadowMap(camera);
        has_init_ = true;
    }
}

void Render::RenderBox()
{
    if(box_vao_ == 0)
    {
        constexpr float vertices[] = {
            // back face
            -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 0.0f, // bottom-left
            1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 1.0f, // top-right
            1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 0.0f, // bottom-right
            1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 1.0f, // top-right
            -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 0.0f, // bottom-left
            -1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 1.0f, // top-left
            // front face
            -1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 0.0f, // bottom-left
            1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f, 0.0f, // bottom-right
            1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f, 1.0f, // top-right
            1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f, 1.0f, // top-right
            -1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 1.0f, // top-left
            -1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 0.0f, // bottom-left
            // left face
            -1.0f,  1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-right
            -1.0f,  1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 1.0f, 1.0f, // top-left
            -1.0f, -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-left
            -1.0f, -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-left
            -1.0f, -1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 0.0f, 0.0f, // bottom-right
            -1.0f,  1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-right
            // right face
            1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-left
            1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-right
            1.0f,  1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 1.0f, 1.0f, // top-right
            1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-right
            1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-left
            1.0f, -1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 0.0f, 0.0f, // bottom-left
            // bottom face
            -1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 1.0f, // top-right
            1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 1.0f, 1.0f, // top-left
            1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 1.0f, 0.0f, // bottom-left
            1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 1.0f, 0.0f, // bottom-left
            -1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 0.0f, // bottom-right
            -1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 1.0f, // top-right
            // top face
            -1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f, // top-left
            1.0f,  1.0f , 1.0f,  0.0f,  1.0f,  0.0f, 1.0f, 0.0f, // bottom-right
            1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 1.0f, 1.0f, // top-right
            1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f, 1.0f, 0.0f, // bottom-right
            -1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f, // top-left
            -1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f, 0.0f, 0.0f  // bottom-left
        };
        glGenVertexArrays(1, &box_vao_);
        glGenBuffers(1, &box_vbo_);
        glBindBuffer(GL_ARRAY_BUFFER, box_vbo_);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
        glBindVertexArray(box_vao_);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }
    glBindVertexArray(box_vao_);
    glDrawArrays(GL_TRIANGLES, 0, 36);
    glBindVertexArray(0);
}

void Render::StepRenderShadowMaps(RenderWorld * world, Camera * camera)
{
    csm_.update(camera, csm_uniforms_.direction);
    UpdateCascadeShadowMap();

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glCullFace(GL_FRONT);

    auto shader_csm = shaders_[kPSSM];
    shader_csm.use();

    glm::mat4 projection = camera->GetProjectionMatrix();
    glm::mat4 view = camera->GetViewMatrix();

    for (int i = 0; i < csm_.frustum_split_count(); ++i)
    {
        shader_csm.setMat4("projection", projection);
        shader_csm.setMat4("view", view);
        shader_csm.setMat4("crop", csm_.split_view_proj(i));

        glBindFramebuffer(GL_FRAMEBUFFER, csm_.framebuffers()[i]);
        glViewport(0, 0, csm_.shadow_map_size(), csm_.shadow_map_size());

        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // glm::vec3 minAABB, maxAABB;
        // GetViewFrusrumBoundingVolume(camera, minAABB, maxAABB);
        //Draw(world, shader_csm, minAABB - glm::vec3(3), maxAABB + glm::vec3(3));
        Draw(world, shader_csm);

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
    glCullFace(GL_BACK);
}

void Render::UpdateCascadeShadowMap()
{
    csm_uniforms_.num_cascades = csm_.frustum_split_count();
    for (int i = 0; i < csm_uniforms_.num_cascades; ++i)
    {
        csm_uniforms_.far_bounds[i] = csm_.far_bound(i);
        csm_uniforms_.texture_matrices[i] = csm_.texture_matrix(i);
    }
}

void Render::InitCascadeShadowMap(Camera * camera)
{
    csm_uniforms_.direction = glm::vec4(-1.0f * sunlight_.direction, 0.0f);
    csm_uniforms_.direction = glm::normalize(csm_uniforms_.direction);
    csm_uniforms_.options.x = 1;
    csm_uniforms_.options.y = 0;
    csm_uniforms_.options.z = 1;

    csm_.initialize(
        settings_.shadow_lamda,
        settings_.shadow_near_offset,
        settings_.shadow_split,
        settings_.shadow_resolution,
        camera,
        width_,
        height_,
        csm_uniforms_.direction
    );
}

void Render::InitIBL(const std::string& hdr_path)
{
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);
    
    glGenFramebuffers(1, &capture_fbo_);
    glGenRenderbuffers(1, &capture_rbo_);
    
    glBindFramebuffer(GL_FRAMEBUFFER, capture_fbo_);
    glBindRenderbuffer(GL_RENDERBUFFER, capture_rbo_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, 512, 512);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, capture_rbo_);

    std::string pwd = get_pwd(__FILE__);
    std::string temp_path;
    if(hdr_path.empty())
        temp_path = pwd + "/envmaps/Ridgecrest_Road_Ref.hdr";
    else
        temp_path = pwd + hdr_path;

    printf("hdri: %s\n", temp_path.c_str());

    GLuint hdr_map_ = HDRTextureFromFile(temp_path.c_str());

    glGenTextures(1, &environment_map_);
    glBindTexture(GL_TEXTURE_CUBE_MAP, environment_map_);
    for (unsigned int i = 0; i < 6; ++i)
    {
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB16F, 512, 512, 0, GL_RGB, GL_FLOAT, nullptr);
    }
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glm::mat4 captureProjection = glm::perspective(glm::radians(90.0f), 1.0f, 0.1f, 10.0f);
    glm::mat4 captureViews[] =
    {
        glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3( 1.0f,  0.0f,  0.0f), glm::vec3(0.0f, -1.0f,  0.0f)),
        glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(-1.0f,  0.0f,  0.0f), glm::vec3(0.0f, -1.0f,  0.0f)),
        glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3( 0.0f,  1.0f,  0.0f), glm::vec3(0.0f,  0.0f,  1.0f)),
        glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3( 0.0f, -1.0f,  0.0f), glm::vec3(0.0f,  0.0f, -1.0f)),
        glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3( 0.0f,  0.0f,  1.0f), glm::vec3(0.0f, -1.0f,  0.0f)),
        glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3( 0.0f,  0.0f, -1.0f), glm::vec3(0.0f, -1.0f,  0.0f))
    };
    
    auto toCubemapShader = shaders_[kConvertToCubemap];
    toCubemapShader.use();
    toCubemapShader.setInt("equirectangularMap", 0);
    toCubemapShader.setMat4("projection", captureProjection);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, hdr_map_);
    
    glViewport(0, 0, 512, 512);
    glBindFramebuffer(GL_FRAMEBUFFER, capture_fbo_);
    for (unsigned int i = 0; i < 6; ++i)
    {
        toCubemapShader.setMat4("view", captureViews[i]);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, environment_map_, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        RenderBox();
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);


    // Convolution
    // Convolute Cubemap to Irradiance Map
    glBindTexture(GL_TEXTURE_CUBE_MAP, environment_map_);
    glGenerateMipmap(GL_TEXTURE_CUBE_MAP);

    glGenTextures(1, &irradiance_map_);
    glBindTexture(GL_TEXTURE_CUBE_MAP, irradiance_map_);
    for (unsigned int i = 0; i < 6; ++i)
    {
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB16F, 32, 32, 0,
                     GL_RGB, GL_FLOAT, nullptr);
    }
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    
    glBindFramebuffer(GL_FRAMEBUFFER, capture_fbo_);
    glBindRenderbuffer(GL_RENDERBUFFER, capture_rbo_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, 32, 32);
    
    auto irradianceConvShader = shaders_[kIrrandianceConv];
    irradianceConvShader.use();
    irradianceConvShader.setInt("environmentMap", 0);
    irradianceConvShader.setMat4("projection", captureProjection);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_CUBE_MAP, environment_map_);
    
    glViewport(0, 0, 32, 32);
    glBindFramebuffer(GL_FRAMEBUFFER, capture_fbo_);
    for (unsigned int i = 0; i < 6; ++i)
    {
        irradianceConvShader.setMat4("view", captureViews[i]);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, irradiance_map_, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        RenderBox();
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Pre-Filter
    // This is for the BRDF Specular Look Up
    glGenTextures(1, &prefilter_map_);
    glBindTexture(GL_TEXTURE_CUBE_MAP, prefilter_map_);
    for (unsigned int i = 0; i < 6; ++i) {
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB16F, 128, 128, 0, GL_RGB, GL_FLOAT, nullptr);
    }
    
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    
    glGenerateMipmap(GL_TEXTURE_CUBE_MAP);
    
    auto prefilterShader = shaders_[kPrefilter];
    prefilterShader.use();
    prefilterShader.setInt("enviromentMap", 0);
    prefilterShader.setMat4("projection", captureProjection);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_CUBE_MAP, environment_map_);
    
    glBindFramebuffer(GL_FRAMEBUFFER, capture_fbo_);
    unsigned int maxMipLevels = 5;
    for (unsigned int mip = 0; mip < maxMipLevels; ++mip) {
        unsigned int mipWidth  = 128 * std::pow(0.5, mip);
        unsigned int mipHeight = 128 * std::pow(0.5, mip);
        glBindRenderbuffer(GL_RENDERBUFFER, capture_rbo_);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, mipWidth, mipHeight);
        glViewport(0, 0, mipWidth, mipHeight);
        
        float roughness = (float)mip / (float)(maxMipLevels - 1);
        prefilterShader.setFloat("roughness", roughness);
        for (unsigned int i = 0; i < 6; ++i)
        {
            prefilterShader.setMat4("view", captureViews[i]);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, prefilter_map_, mip);
            
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            RenderBox();
        }
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);


    // BRDF LUT
    glGenTextures(1, &brdf_lut_map_);
    glBindTexture(GL_TEXTURE_2D, brdf_lut_map_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RG16F, 512, 512, 0, GL_RG, GL_FLOAT, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glBindFramebuffer(GL_FRAMEBUFFER, capture_fbo_);
    glBindRenderbuffer(GL_RENDERBUFFER, capture_rbo_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, 512, 512);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, brdf_lut_map_, 0);
    
    glViewport(0, 0, 512, 512);
    auto brdfShader = shaders_[kBRDF];
    brdfShader.use();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    RenderQuad();

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Render::GetViewFrusrumBoundingVolume(const glm::vec3 eye,
        const glm::vec3 front, const float far, const float aspect,
        const float fov, const glm::mat4 view_matrix,
        glm::vec3& aabb_min, glm::vec3& aabb_max) 
{
    glm::mat4 view2world_mat4 = glm::inverse(view_matrix);

    float tanHalfHHOV = tanf(glm::radians(fov / 2.0f));
    float tanHalfVHOV = tanf(glm::radians(fov * aspect / 2.0f));

    float xn = 0 * tanHalfHHOV;
    float xf = -far * tanHalfHHOV;
    float yn = 0 * tanHalfVHOV;
    float yf = -far * tanHalfVHOV;
    
    glm::vec4 frustumCorners[8] = 
    {
        glm::vec4(xn, yn, 0, 1),
        glm::vec4(-xn, yn, 0, 1),
        glm::vec4(xn, -yn, 0, 1),
        glm::vec4(-xn, -yn, 0, 1),

        glm::vec4(xf, yf, -far, 1),
        glm::vec4(-xf, yf, -far, 1),
        glm::vec4(xf, -yf, -far, 1),
        glm::vec4(-xf, -yf, -far, 1) 
    };

    float minX = 1e+5;
    float maxX = -1e+5;
    float minY = 1e+5;
    float maxY = -1e+5;
    float minZ = 1e+5;
    float maxZ = -1e+5;

    for (unsigned int i = 0; i < 8; ++i)
    {
        glm::vec4 vW = view2world_mat4 * frustumCorners[i];

        minX = glm::min(minX, vW.x);
        maxX = glm::max(maxX, vW.x);
        minY = glm::min(minY, vW.y);
        maxY = glm::max(maxY, vW.y);
        minZ = glm::min(minZ, vW.z);
        maxZ = glm::max(maxZ, vW.z);
    }

    aabb_min = glm::vec3(minX, minY, minZ);
    aabb_max = glm::vec3(maxX, maxY, maxZ);
}

void Render::GetViewFrusrumBoundingVolume(Camera* camera, 
        glm::vec3& aabb_min, glm::vec3& aabb_max)
{
    glm::mat4 view2world_mat4 = glm::inverse(camera->GetViewMatrix());

    float far = camera->Far;
    float ar = camera->Aspect;
    float tanHalfHHOV = tanf(glm::radians((camera->Zoom + 0.0f) / 2.0f));
    float tanHalfVHOV = tanf(glm::radians((camera->Zoom + 0.0f) * ar / 2.0f));

    float xn = 0 * tanHalfHHOV;
    float xf = -far / 1.1f * tanHalfHHOV;
    float yn = 0 * tanHalfVHOV;
    float yf = -far / 1.1f * tanHalfVHOV;
    
    glm::vec4 frustumCorners[8] = 
    {
        glm::vec4(xn, yn, 0, 1),
        glm::vec4(-xn, yn, 0, 1),
        glm::vec4(xn, -yn, 0, 1),
        glm::vec4(-xn, -yn, 0, 1),

        glm::vec4(xf, yf, -far, 1),
        glm::vec4(-xf, yf, -far, 1),
        glm::vec4(xf, -yf, -far, 1),
        glm::vec4(-xf, -yf, -far, 1) 
    };

    float minX = 1e+5;
    float maxX = -1e+5;
    float minY = 1e+5;
    float maxY = -1e+5;
    float minZ = 1e+5;
    float maxZ = -1e+5;

    for (unsigned int i = 0; i < 8; ++i)
    {
        glm::vec4 vW = view2world_mat4 * frustumCorners[i];

        minX = glm::min(minX, vW.x);
        maxX = glm::max(maxX, vW.x);
        minY = glm::min(minY, vW.y);
        maxY = glm::max(maxY, vW.y);
        minZ = glm::min(minZ, vW.z);
        maxZ = glm::max(maxZ, vW.z);
    }

    aabb_min = glm::vec3(minX, minY, minZ);
    aabb_max = glm::vec3(maxX, maxY, maxZ);
}

void Render::UpdateProjectionMatrices(RenderWorld * world)
{
    float min_x;
    float min_z;
    float max_x;
    float max_z;
    world->get_world_size(min_x, min_z, max_x, max_z);

    vct_bbox_min_ = glm::vec3(min_x - 1, -2, min_z - 1);
    vct_bbox_max_ = glm::vec3(max_x + 1,  6, max_z + 1);

    glm::vec3 center = (vct_bbox_min_ + vct_bbox_max_) * 0.5f;
    glm::vec3 axis_size = vct_bbox_max_ - vct_bbox_min_;
    volume_grid_size_ = glm::max(glm::max(axis_size.x, axis_size.y), axis_size.z);
    voxel_size_ = (float) volume_grid_size_ / (float) volume_dimension_;
    voxel_count_ = volume_dimension_ * volume_dimension_ * volume_dimension_;

    float half_size = volume_grid_size_ * 0.5f;
    glm::mat4 projection = glm::ortho(-half_size, half_size, -half_size, half_size, 0.0f, volume_grid_size_);


    view_projection_matrix_[0] = glm::lookAt(center + glm::vec3(half_size, 0.0f, 0.0f),
                                     center, glm::vec3(0.0f, 1.0f, 0.0f));
    view_projection_matrix_[1] = glm::lookAt(center + glm::vec3(0.0f, half_size, 0.0f),
                                     center, glm::vec3(0.0f, 0.0f, -1.0f));
    view_projection_matrix_[2] = glm::lookAt(center + glm::vec3(0.0f, 0.0f, half_size),
                                     center, glm::vec3(0.0f, 1.0f, 0.0f));

    int i = 0;
    for (auto &matrix : view_projection_matrix_)
    {
        matrix = projection * matrix;
        view_projection_matrix_inv_[i++] = glm::inverse(matrix);
    }
}

void Render::VoxelConeTracing(RenderWorld* world, Camera * camera)
{
    auto shader_vct = shaders_[kVoxelConeTracing];
    shader_vct.use();

    glColorMask(true, true, true, true);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glViewport(0, 0, width_, height_);
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);

    glm::mat4 projection = camera->GetProjectionMatrix();
    glm::mat4 view = camera->GetViewMatrix();

    shader_vct.setInt("shadowMode", lighting_.force_disable_shadow ? 4 : 0);
    shader_vct.setFloat("samplingFactor", lighting_.sample_factor);
    shader_vct.setFloat("ibl_factor", lighting_.ibl_factor);
    
    if(settings_.use_shadow && !lighting_.force_disable_shadow) {
        shader_vct.setVec4("direction", csm_uniforms_.direction);
        shader_vct.setVec4("options", csm_uniforms_.options);
        shader_vct.setInt("num_cascades", csm_uniforms_.num_cascades);
        shader_vct.setFloat("bias_scale", lighting_.shadow_bias_scale);
        shader_vct.setFloat("bias_clamp", lighting_.shadow_bias_clamp);

        for (unsigned int i = 0; i < csm_uniforms_.num_cascades - 1; ++i)
        {
            std::string far_bounds_str = "far_bounds[" + std::to_string(i) + "]";
            shader_vct.setFloat(far_bounds_str.c_str(), csm_uniforms_.far_bounds[i]);

            std::string mat_str = "texture_matrices[" + std::to_string(i) + "]";
            shader_vct.setMat4(mat_str.c_str(), csm_uniforms_.texture_matrices[i]);
        }

        for (unsigned int i = 0; i < csm_uniforms_.num_cascades; ++i)
        {
            glActiveTexture(GL_TEXTURE15 + i);
            glBindTexture(GL_TEXTURE_2D, csm_.shadow_map()[i]);
            std::string temp = "s_ShadowMap[" + std::to_string(i) + "]";
            glUniform1i(glGetUniformLocation(shader_vct.id(), temp.c_str()), 15 + i);
        }
    } else {
        shader_vct.setInt("shadowMode", 4);
    }


    if(use_sunlight_) {
        shader_vct.setInt("numDirectionalLight", 1);
        shader_vct.setVec3("directionalLight[0].specular", sunlight_.specular);
        shader_vct.setVec3("directionalLight[0].ambient", sunlight_.ambient);
        shader_vct.setVec3("directionalLight[0].diffuse", sunlight_.diffuse);
        shader_vct.setVec3("directionalLight[0].direction", sunlight_.direction);
    } else {
        shader_vct.setInt("numDirectionalLight", 0);
    }

    shader_vct.setFloat("exposure", lighting_.exposure);
    shader_vct.setFloat("bounceStrength", lighting_.indirect_strength);
    shader_vct.setFloat("maxTracingDistanceGlobal", lighting_.conetracing_distance);

    if(settings_.use_vct) {
        shader_vct.setFloat("voxelScale", 1.0f / volume_grid_size_);
        shader_vct.setVec3("worldMinPoint", vct_bbox_min_);
        shader_vct.setVec3("worldMaxPoint", vct_bbox_max_);
        shader_vct.setInt("volumeDimension", volume_dimension_);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_3D, voxel_normal_->textureID);
        glUniform1i(glGetUniformLocation(shader_vct.id(), "voxelVisibility"), 0);

        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_3D, voxel_radiance_->textureID);
        glUniform1i(glGetUniformLocation(shader_vct.id(), "voxelTex"), 1);
    }

    if(settings_.use_pbr) {
        glActiveTexture(GL_TEXTURE2);
        glBindTexture(GL_TEXTURE_CUBE_MAP, irradiance_map_);
        glUniform1i(glGetUniformLocation(shader_vct.id(), "irradianceMap"), 2);

        glActiveTexture(GL_TEXTURE3);
        glBindTexture(GL_TEXTURE_CUBE_MAP, prefilter_map_);
        glUniform1i(glGetUniformLocation(shader_vct.id(), "prefilterMap"), 3);

        glActiveTexture(GL_TEXTURE4);
        glBindTexture(GL_TEXTURE_2D, brdf_lut_map_);
        glUniform1i(glGetUniformLocation(shader_vct.id(), "brdfLUT"), 4);
    }

    glActiveTexture(GL_TEXTURE5);
    glBindTexture(GL_TEXTURE_2D, g_albedospec_);
    glUniform1i(glGetUniformLocation(shader_vct.id(), "gAlbedoSpec"), 5);

    glActiveTexture(GL_TEXTURE6);
    glBindTexture(GL_TEXTURE_2D, g_normal_);
    glUniform1i(glGetUniformLocation(shader_vct.id(), "gNormal"), 6);

    glActiveTexture(GL_TEXTURE7);
    glBindTexture(GL_TEXTURE_2D, g_position_);
    glUniform1i(glGetUniformLocation(shader_vct.id(), "gPosition"), 7);

    glActiveTexture(GL_TEXTURE8);
    glBindTexture(GL_TEXTURE_2D, g_pbr_);
    glUniform1i(glGetUniformLocation(shader_vct.id(), "gPBR"), 8);

    if(settings_.use_vct) {
        for (unsigned int i = 0; i < 6; ++i)
        {
            glActiveTexture(GL_TEXTURE9 + i);
            glBindTexture(GL_TEXTURE_3D, voxel_mipmaps_[i]->textureID);
            std::string temp = "voxelTexMipmap[" + std::to_string(i) + "]";
            glUniform1i(glGetUniformLocation(shader_vct.id(), temp.c_str()), 9 + i);
        }
    }

    if(settings_.use_vct)
        shader_vct.setInt("mode", 0);
    else if(!settings_.use_vct && settings_.use_pbr)
        shader_vct.setInt("mode", 6);
    else if(!settings_.use_vct && !settings_.use_pbr)
        shader_vct.setInt("mode", 5);

    shader_vct.setVec3("cameraPosition", camera->Position);

    RenderQuad();
}

void Render::InjectRadiance()
{
    auto shader_injection = shaders_[kRadianceInjection];
    shader_injection.use();

    auto vSize = volume_grid_size_ / volume_dimension_;

    if(use_sunlight_) {
        shader_injection.setInt("numDirectionalLight", 1);
        shader_injection.setVec3("directionalLight[0].diffuse", sunlight_.diffuse);
        shader_injection.setVec3("directionalLight[0].direction", sunlight_.direction);
    }
    else
    {
        shader_injection.setInt("numDirectionalLight", 0);
    }

    // if(settings_.use_shadow) {
    //     //shader_injection.setInt("shadowMode", 1);
    //     shader_injection.setVec4("direction", csm_uniforms_.direction);
    //     shader_injection.setVec4("options", csm_uniforms_.options);
    //     shader_injection.setInt("num_cascades", csm_uniforms_.num_cascades);

    //     shader_injection.setVec3("camera_position", free_camera_.Position);

    //     for (unsigned int i = 0; i < csm_uniforms_.num_cascades - 1; ++i)
    //     {
    //         std::string far_bounds_str = "far_bounds[" + std::to_string(i) + "]";
    //         shader_injection.setFloat(far_bounds_str.c_str(), csm_uniforms_.far_bounds[i]);

    //         std::string mat_str = "texture_matrices[" + std::to_string(i) + "]";
    //         shader_injection.setMat4(mat_str.c_str(), csm_uniforms_.texture_matrices[i]);
    //     }

    //     for (unsigned int i = 0; i < csm_uniforms_.num_cascades; ++i)
    //     {
    //         glActiveTexture(GL_TEXTURE4 + i);
    //         glBindTexture(GL_TEXTURE_2D, csm_.shadow_map()[i]);
    //         std::string temp = "s_ShadowMap[" + std::to_string(i) + "]";
    //         glUniform1i(glGetUniformLocation(shader_injection.id(), temp.c_str()), 4 + i);
    //     }
    // }

    shader_injection.setFloat("traceShadowHit", lighting_.traceshadow_distance);
    shader_injection.setFloat("voxelSize", vSize);
    shader_injection.setFloat("voxelScale", 1.0f / volume_grid_size_);
    shader_injection.setVec3("worldMinPoint", vct_bbox_min_);
    shader_injection.setInt("volumeDimension", volume_dimension_);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_3D, voxel_albedo_->textureID);
    glUniform1i(glGetUniformLocation(shader_injection.id(), "voxelAlbedo"), 0);

    glBindImageTexture(1, voxel_normal_->textureID, 0, GL_TRUE, 0, GL_READ_WRITE, GL_RGBA8);
    glBindImageTexture(2, voxel_radiance_->textureID, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA8);
    glBindImageTexture(3, voxel_emissive_->textureID, 0, GL_TRUE, 0, GL_READ_ONLY, GL_RGBA8);

    auto workGroups = static_cast<unsigned>(glm::ceil(volume_dimension_ / 8.0f));

    glDispatchCompute(workGroups, workGroups, workGroups);

    glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

    GenerateMipmapBase();
    GenerateMipmapVolume();

    if(!lighting_.force_disable_propagation) {
        PropagateRadiance();
        GenerateMipmapBase();
        GenerateMipmapVolume();
    }
}

void Render::GenerateMipmapBase()
{
    auto shader_mipbase = shaders_[kMipMapBase];
    shader_mipbase.use();

    float halfDimension = volume_dimension_ / 2;

    shader_mipbase.setInt("mipDimension", halfDimension);

    for (int i = 0; i < 6; ++i)
    {
        glBindImageTexture(i, voxel_mipmaps_[i]->textureID, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA8);   
    }

    glActiveTexture(GL_TEXTURE6);
    glBindTexture(GL_TEXTURE_3D, voxel_radiance_->textureID);
    glUniform1i(glGetUniformLocation(shader_mipbase.id(), "voxelBase"), 6);

    auto workGroups = static_cast<unsigned int>(glm::ceil(halfDimension / 8.0f));

    glDispatchCompute(workGroups, workGroups, workGroups);

    glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
}

void Render::GenerateMipmapVolume()
{
    auto shader_mipvol = shaders_[kMipMapVolume];

    auto mipDimension = volume_dimension_ / 4;
    auto mipLevel = 0;

    while(mipDimension >= 1)
    {
        shader_mipvol.use();

        auto volumeSize = glm::vec3(mipDimension, mipDimension, mipDimension);

        shader_mipvol.setVec3("mipDimension", volumeSize);
        shader_mipvol.setInt("mipLevel", mipLevel);


        for (int i = 0; i < 6; ++i)
        {
            glActiveTexture(GL_TEXTURE6 + i);
            glBindTexture(GL_TEXTURE_3D, voxel_mipmaps_[i]->textureID);
            std::string temp = "voxelMipmapSrc[" + std::to_string(i) + "]";
            glUniform1i(glGetUniformLocation(shader_mipvol.id(), temp.c_str()), 6 + i); 

            glBindImageTexture(i, voxel_mipmaps_[i]->textureID, mipLevel + 1, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA8);                 
        }

        auto workGroups = static_cast<unsigned>(glm::ceil(mipDimension / 8.0f));

        glDispatchCompute(workGroups, workGroups, workGroups);

        glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

        mipLevel++;
        mipDimension /= 2;
    }
}

void Render::PropagateRadiance()
{
    auto shader_propagation = shaders_[kRadiancePropagation];

    auto vSize = volume_grid_size_ / volume_dimension_;

    shader_propagation.use();

    shader_propagation.setInt("volumeDimension", volume_dimension_);
    shader_propagation.setFloat("maxTracingDistanceGlobal", lighting_.propagation_distance);

    glBindImageTexture(0, voxel_radiance_->textureID, 0, GL_TRUE, 0, GL_READ_WRITE, GL_RGBA8);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_3D, voxel_albedo_->textureID);
    glUniform1i(glGetUniformLocation(shader_propagation.id(), "voxelAlbedo"), 1);

    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_3D, voxel_normal_->textureID);
    glUniform1i(glGetUniformLocation(shader_propagation.id(), "voxelNormal"), 2);

    for (int i = 0; i < 6; ++i)
    {
        glActiveTexture(GL_TEXTURE3 + i);
        glBindTexture(GL_TEXTURE_3D, voxel_mipmaps_[i]->textureID);
        std::string temp = "voxelTexMipmap[" + std::to_string(i) + "]";
        glUniform1i(glGetUniformLocation(shader_propagation.id(), temp.c_str()), 3 + i);
    }

    auto workGroups = static_cast<unsigned>(glm::ceil(volume_dimension_ / 8.0f));

    glDispatchCompute(workGroups, workGroups, workGroups);

    glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
}

void Render::ClearTextures()
{
    auto shader_clear = shaders_[kClearBuffer];
    shader_clear.use();

    glBindImageTexture(0, voxel_albedo_->textureID, 0, GL_TRUE, 0, GL_READ_WRITE, GL_RGBA8);      
    glBindImageTexture(1, voxel_normal_->textureID, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA8);
    glBindImageTexture(2, voxel_emissive_->textureID, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA8);

    auto workGroups = static_cast<unsigned>(glm::ceil(volume_dimension_ / 8.0f));

    glDispatchCompute(workGroups, workGroups, workGroups);

    glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
}

void Render::BakeScene(RenderWorld* world)
{
    assert(world);
    if(settings_.use_vct)
        VoxelizeScene(world);
}

void Render::VoxelizeScene(RenderWorld* world)
{
    UpdateProjectionMatrices(world);
    ClearTextures();

    glColorMask(false, false, false, false);
    glViewport(0, 0, volume_dimension_, volume_dimension_);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);

    auto shader_voxelize = shaders_[kVoxelize];
    shader_voxelize.use();

    shader_voxelize.setMat4("viewProjections[0]", view_projection_matrix_[0]);
    shader_voxelize.setMat4("viewProjections[1]", view_projection_matrix_[1]);
    shader_voxelize.setMat4("viewProjections[2]", view_projection_matrix_[2]);
    shader_voxelize.setMat4("viewProjectionsI[0]", view_projection_matrix_inv_[0]);
    shader_voxelize.setMat4("viewProjectionsI[1]", view_projection_matrix_inv_[1]);
    shader_voxelize.setMat4("viewProjectionsI[2]", view_projection_matrix_inv_[2]);
    shader_voxelize.setInt("volumeDimension", volume_dimension_);
    shader_voxelize.setVec3("worldMinPoint", vct_bbox_min_);
    shader_voxelize.setFloat("voxelScale", 1.0f / volume_grid_size_);
    shader_voxelize.setInt("linear", lighting_.linear_voxelize);
    shader_voxelize.setFloat("ambient", lighting_.boost_ambient);

    voxel_radiance_->Clear();

    for (int i = 0; i < 6; ++i)
        voxel_mipmaps_[i]->Clear();

    glBindImageTexture(0, voxel_albedo_->textureID, 0, GL_TRUE, 0, GL_READ_WRITE, GL_R32UI);
    glBindImageTexture(1, voxel_normal_->textureID, 0, GL_TRUE, 0, GL_READ_WRITE, GL_R32UI);
    glBindImageTexture(2, voxel_emissive_->textureID, 0, GL_TRUE, 0, GL_READ_WRITE, GL_R32UI);


    assert(world->cameras_size() > 0);
    Camera* camera = world->camera(0);
    //Camera* camera = &free_camera_; 

    glm::vec3 minAABB, maxAABB;
    GetViewFrusrumBoundingVolume(camera, minAABB, maxAABB);

    Draw(world, shader_voxelize, minAABB - glm::vec3(5), maxAABB + glm::vec3(5), true);

    glMemoryBarrier(
        GL_TEXTURE_FETCH_BARRIER_BIT | 
        GL_SHADER_IMAGE_ACCESS_BARRIER_BIT |
        GL_ATOMIC_COUNTER_BARRIER_BIT
    );

    InjectRadiance();
}

void Render::VisualizeVoxels(const int draw_mip_level)
{
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glColorMask(true, true, true, true);
    glViewport(width_ / 3, 0, width_ / 3, height_ / 3);
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    auto shader_visualization = shaders_[kVoxelVisualization];
    shader_visualization.use();

    glm::mat4 proj = free_camera_.GetProjectionMatrix();
    glm::mat4 view = free_camera_.GetViewMatrix();

    auto vDimension = volume_dimension_ / pow(2.0f, draw_mip_level);
    auto vSize = volume_grid_size_ / vDimension;

    shader_visualization.setMat4("viewProjectionMatrix", proj);

    glBindImageTexture(0, voxel_radiance_->textureID, 0, GL_TRUE, 0, GL_READ_ONLY, GL_RGBA8);

    auto model = translate(glm::mat4(1), vct_bbox_min_) * glm::scale(glm::mat4(1), glm::vec3(vSize));

    shader_visualization.setInt("volumeDimension", vDimension);
    shader_visualization.setMat4("matrices.modelViewProjection", proj * view * model);
    shader_visualization.setFloat("voxelSize", vSize);
    shader_visualization.setVec3("minPoint", vct_bbox_min_);
    shader_visualization.setVec4("colorChannels", glm::vec4(1,1,1,1));

    glBindVertexArray(voxel_vao_);
    glDrawArrays(GL_POINTS, 0, voxel_count_);
    glBindVertexArray(0);
}

void Render::InitVoxelization()
{
    assert(volume_dimension_ > 63);

    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    const std::vector<unsigned char> clean_texture(pow(volume_dimension_, 3), 0);

    voxel_albedo_ = new Texture3D(clean_texture, volume_dimension_, volume_dimension_, volume_dimension_);
    voxel_normal_ = new Texture3D(clean_texture, volume_dimension_, volume_dimension_, volume_dimension_);
    voxel_emissive_ = new Texture3D(clean_texture, volume_dimension_, volume_dimension_, volume_dimension_);
    voxel_radiance_ = new Texture3D(clean_texture, volume_dimension_, volume_dimension_, volume_dimension_);

    for (unsigned int i = 0; i < 6; ++i)
    {
        voxel_mipmaps_[i] = new Texture3D(clean_texture,
            volume_dimension_ / 2, volume_dimension_ / 2, volume_dimension_ / 2,
            true, 6
        );
    }

    glGenVertexArrays(1, &voxel_vao_);
}

void Render::InitGBuffer()
{
    glGenFramebuffers(1, &g_buffer_fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, g_buffer_fbo_);

    glGenTextures(1, &g_position_);
    glBindTexture(GL_TEXTURE_2D, g_position_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width_, height_, 0, GL_RGBA, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, g_position_, 0);

    glGenTextures(1, &g_normal_);
    glBindTexture(GL_TEXTURE_2D, g_normal_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, width_, height_, 0, GL_RGBA, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, g_normal_, 0);

    glGenTextures(1, &g_albedospec_);
    glBindTexture(GL_TEXTURE_2D, g_albedospec_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width_, height_, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_2D, g_albedospec_, 0);

    glGenTextures(1, &g_pbr_);
    glBindTexture(GL_TEXTURE_2D, g_pbr_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width_, height_, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT3, GL_TEXTURE_2D, g_pbr_, 0);

    unsigned int attachments[4] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2, GL_COLOR_ATTACHMENT3};
    glDrawBuffers(4, attachments);

    glGenRenderbuffers(1, &g_buffer_rbo_);
    glBindRenderbuffer(GL_RENDERBUFFER, g_buffer_rbo_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width_, height_);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, g_buffer_rbo_);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        std::cout << "G-Buffer Not Complete!" << std::endl;

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Render::InitReflectionMap()
{
    std::string pwd = get_pwd(__FILE__);
    std::vector<std::string> faces {
        pwd + "/../data/cubemaps/right.tga",
        pwd + "/../data/cubemaps/left.tga",
        pwd + "/../data/cubemaps/top.tga",
        pwd + "/../data/cubemaps/bottom.tga",
        pwd + "/../data/cubemaps/front.tga",
        pwd + "/../data/cubemaps/back.tga",
    };
    reflection_map_ = LoadCubemap(faces);
}

void Render::InitPBO()
{
    glGenBuffers(2, pixel_buffers_);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, pixel_buffers_[0]);
    glBufferData(GL_PIXEL_PACK_BUFFER,
                 width_ * height_ * 4 * sizeof(unsigned char),
                 nullptr, GL_STREAM_READ);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, pixel_buffers_[1]);
    glBufferData(GL_PIXEL_PACK_BUFFER,
                 width_ * height_ * 4 * sizeof(unsigned char),
                 nullptr, GL_STREAM_READ);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
}

Render::~Render() {

    // Post
    glDeleteFramebuffers(1, &ssao_composite_fbo_);
    glDeleteFramebuffers(1, &ssao_blur_fbo_);
    glDeleteFramebuffers(1, &ssao_fbo_);
    glDeleteFramebuffers(1, &fxaa_fbo_);
    glDeleteTextures(1, &ssao_composite_tex_);
    glDeleteTextures(1, &noise_tex_);
    glDeleteTextures(1, &ssao_tex_);
    glDeleteTextures(1, &ssao_tex_blur_);
    glDeleteTextures(1, &fxaa_tex_);

    if(settings_.use_pbr)
    {
        glDeleteFramebuffers(1, &capture_fbo_);
        glDeleteRenderbuffers(1, &capture_rbo_);
    }

    if(settings_.use_ibl)
    {
        glDeleteTextures(1, &hdr_map_);
        glDeleteTextures(1, &environment_map_);
        glDeleteTextures(1, &irradiance_map_);
        glDeleteTextures(1, &prefilter_map_);
        glDeleteTextures(1, &brdf_lut_map_);
    }

    if(settings_.use_deferred)
    {
        glDeleteFramebuffers(1, &g_buffer_fbo_);
        glDeleteRenderbuffers(1, &g_buffer_rbo_);
        glDeleteTextures(1, &g_position_);
        glDeleteTextures(1, &g_normal_);
        glDeleteTextures(1, &g_albedospec_);
        glDeleteTextures(1, &g_pbr_);
    }

    if(settings_.use_vct)
    {
        delete voxel_albedo_;
        delete voxel_normal_;
        delete voxel_emissive_;
        delete voxel_radiance_;
        for (unsigned int i = 0; i < 6; ++i)
            delete voxel_mipmaps_[i];
        glDeleteVertexArrays(1, &voxel_vao_);
    }

    glDeleteTextures(1, &reflection_map_);
    glDeleteVertexArrays(1, &crosshair_vao_);
    glDeleteVertexArrays(1, &cube_vao_);
    glDeleteVertexArrays(1, &quad_vao_);
    glDeleteBuffers(1, &crosshair_vbo_);
    glDeleteBuffers(1, &cube_vbo_);
    glDeleteBuffers(1, &quad_vbo_);
    glDeleteBuffers(2, pixel_buffers_);
    
    for(int i = 0; i < camera_framebuffer_list_.size(); ++i) {
        delete camera_framebuffer_list_[i];
        delete multiplied_framebuffer_list_[i];
    }

    delete free_camera_framebuffer_;
    img_buffers_.clear();

    CloseContext(ctx_);
}

void Render::ProcessInput() {
    if(ctx_->GetKeyPressESC())
        ctx_->SetWindowShouldClose();
    if(ctx_->GetKeyPressW()) 
        free_camera_.ProcessKeyboard(Camera::FORWARD, delta_time_);
    if(ctx_->GetKeyPressS())
        free_camera_.ProcessKeyboard(Camera::BACKWARD, delta_time_);
    if(ctx_->GetKeyPressA())
        free_camera_.ProcessKeyboard(Camera::LEFT, delta_time_);
    if(ctx_->GetKeyPressD())
        free_camera_.ProcessKeyboard(Camera::RIGHT, delta_time_);
    if(ctx_->GetKeyPressQ())
        free_camera_.ProcessKeyboard(Camera::UP, delta_time_);
    if(ctx_->GetKeyPressE())
        free_camera_.ProcessKeyboard(Camera::DOWN, delta_time_);
}

void Render::ProcessMouse() {
    float x_position, y_position;
    ctx_->GetMouse(x_position, y_position);

    if(first_mouse_) {
        last_x_ = x_position;
        last_y_ = y_position;
        first_mouse_ = false;
    }

    float x_offset = x_position - last_x_;
    float y_offset = last_y_ - y_position;

    last_x_ = x_position;
    last_y_ = y_position;

    free_camera_.ProcessMouseMovement(x_offset, y_offset);
}

void Render::RenderMarker() {
    if(crosshair_vao_ == 0) {
        constexpr float vertices[] = {
            -0.075, 0, 0,
            0.075, 0, 0,
            0, -0.1, 0,
            0, 0.1, 0
        };

        glGenVertexArrays(1, &crosshair_vao_);
        glGenBuffers(1, &crosshair_vbo_);
        glBindBuffer(GL_ARRAY_BUFFER, crosshair_vbo_);
        glBufferData(
                GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
        glBindVertexArray(crosshair_vao_);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(
                0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

    glBindVertexArray(crosshair_vao_);
    glDrawArrays(GL_LINES, 0, 4);
    glBindVertexArray(0);
}

void Render::RenderCube() {
    if (cube_vao_ == 0) {
        constexpr float vertices[] = {
            // back face
            -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 0.0f, // bottom-left
             1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 0.0f, // bottom-right         
             1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 1.0f, // top-right
            -1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 1.0f, // top-left
            1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 0.0f, // bottom-right
            1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 1.0f, // top-right
            -1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 1.0f, // top-left
            -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 0.0f, // bottom-left
            // front face
            -1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 0.0f, // bottom-left
             1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f, 0.0f, // bottom-right
             1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f, 1.0f, // top-right
             -1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 1.0f, // top-left
             1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f, 0.0f, // bottom-right
              1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f, 1.0f, // top-right
            -1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 1.0f, // top-left
            -1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 0.0f, // bottom-left
            // left face
            -1.0f, -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-left
            -1.0f, -1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 0.0f, 0.0f, // bottom-right
            -1.0f,  1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-right
            -1.0f,  1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 1.0f, 1.0f, // top-left
            -1.0f, -1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 0.0f, 0.0f, // bottom-right
            -1.0f,  1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-right
            -1.0f,  1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 1.0f, 1.0f, // top-left
            -1.0f, -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-left
            // right face
            1.0f, -1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 0.0f, 0.0f, // bottom-left
             1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-right 
            1.0f,  1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 1.0f, 1.0f, // top-right   
             1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-left
             1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-right
             1.0f,  1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 1.0f, 1.0f, // top-right         
             1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-left
             1.0f, -1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 0.0f, 0.0f, // bottom-left     
            // bottom face
            1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 1.0f, 0.0f, // bottom-left
            -1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 0.0f, // bottom-right
            -1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 1.0f, // top-right
             1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 1.0f, 1.0f, // top-left
            -1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 0.0f, // bottom-right
            -1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 1.0f, // top-right
             1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 1.0f, 1.0f, // top-left
            1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 1.0f, 0.0f, // bottom-left
            // top face
            -1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f, 0.0f, 0.0f,  // bottom-left
             1.0f,  1.0f , 1.0f,  0.0f,  1.0f,  0.0f, 1.0f, 0.0f, // bottom-right
              1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 1.0f, 1.0f, // top-right  
            -1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f, // top-left
             1.0f,  1.0f , 1.0f,  0.0f,  1.0f,  0.0f, 1.0f, 0.0f, // bottom-right
             1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 1.0f, 1.0f, // top-right     
            -1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f, // top-left
            -1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f, 0.0f, 0.0f  // bottom-left        
        };
        glGenVertexArrays(1, &cube_vao_);
        glGenBuffers(1, &cube_vbo_);
        glBindBuffer(GL_ARRAY_BUFFER, cube_vbo_);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
        glBindVertexArray(cube_vao_);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(
                0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1,
                              3,
                              GL_FLOAT,
                              GL_FALSE,
                              8 * sizeof(float),
                              (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2,
                              2,
                              GL_FLOAT,
                              GL_FALSE,
                              8 * sizeof(float),
                              (void*)(6 * sizeof(float)));
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }
    // render Cube
    glBindVertexArray(cube_vao_);
    glDrawArrays(GL_LINES, 0, 48);
    glBindVertexArray(0);
}

void Render::RenderQuad() {
    if (quad_vao_ == 0) {
        constexpr float quadVertices[] = {
            // positions        // texture Coords
            -1.0f,  1.0f, 0.0f, 0.0f, 1.0f,
            -1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
            1.0f,  1.0f, 0.0f, 1.0f, 1.0f,
            1.0f, -1.0f, 0.0f, 1.0f, 0.0f,
        };
        // setup plane VAO
        glGenVertexArrays(1, &quad_vao_);
        glGenBuffers(1, &quad_vbo_);
        glBindVertexArray(quad_vao_);
        glBindBuffer(GL_ARRAY_BUFFER, quad_vbo_);
        glBufferData(GL_ARRAY_BUFFER,
                     sizeof(quadVertices),
                     &quadVertices,
                     GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(
                0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1,
                              2,
                              GL_FLOAT,
                              GL_FALSE,
                              5 * sizeof(float),
                              (void*)(3 * sizeof(float)));
    }
    glBindVertexArray(quad_vao_);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glBindVertexArray(0);
}

void Render::InitFramebuffers(const unsigned int num_cameras) {
    camera_framebuffer_list_.resize(num_cameras);
    multiplied_framebuffer_list_.resize(num_cameras);
    for(unsigned int i = 0; i < num_cameras; ++i) {
        camera_framebuffer_list_[i] = new FBO(width_, height_, true, true);
        multiplied_framebuffer_list_[i] = new FBO(width_, height_, false, false);
    }
}

void Render::InitDrawBatchRay(const int rays) {
    assert(rays > 0);
    num_rays = rays;

    if(batch_ray_vao_ == 0) {
        glGenVertexArrays(1, &batch_ray_vao_);
        glBindVertexArray(batch_ray_vao_);

        glGenBuffers(1, &batch_ray_vbo_);
        glBindBuffer(GL_ARRAY_BUFFER, batch_ray_vbo_);
        glBufferData(GL_ARRAY_BUFFER, num_rays * 6 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(
                    0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glBindVertexArray(0);
    }
}

void Render::UpdateRay(const int offset, const glm::vec3 from, const glm::vec3 to) {
    float sub_buffer_temp[6];
    sub_buffer_temp[0] = from[0];
    sub_buffer_temp[1] = from[1];
    sub_buffer_temp[2] = from[2];
    sub_buffer_temp[3] = to[0];
    sub_buffer_temp[4] = to[1];
    sub_buffer_temp[5] = to[2];

    glBindBuffer(GL_ARRAY_BUFFER, batch_ray_vbo_);
    glBufferSubData(GL_ARRAY_BUFFER, offset * 6 * sizeof(float), 6 * sizeof(float),
            sub_buffer_temp);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void Render::DrawBatchRay() {
    glEnable(GL_LINE_SMOOTH);
    glLineWidth(1.0f);

    glm::mat4 projection = free_camera_.GetProjectionMatrix();
    glm::mat4 view = free_camera_.GetViewMatrix();

    Shader ray_shader = shaders_[kRay];

    ray_shader.use();
    ray_shader.setMat4("projection", projection);
    ray_shader.setMat4("view", view);

    if(batch_ray_vao_) {
        glBindVertexArray(batch_ray_vao_);
        glDrawArrays(GL_LINES, 0, 2 * num_rays);
        glBindVertexArray(0);
    }

    glDisable(GL_LINE_SMOOTH);
}

void Render::DrawRootAABB(RenderWorld* world, const Shader& shader) {
    glDisable( GL_POLYGON_OFFSET_FILL );

    for (size_t i = 0; i < world->size(); i++)
    {
        RenderBody* body = world->render_body_ptr(i);
        RenderPart * part = body->render_root_ptr();
        if (part && !body->recycle())
        {
            glm::vec3 aabb_min, aabb_max;
            part->GetAABB(aabb_min, aabb_max);
            shader.setVec3("aabbMin", aabb_min);
            shader.setVec3("aabbMax", aabb_max);
            RenderCube();
        }
    }

    glPolygonOffset(1.0f, 1.0f);
    glEnable( GL_POLYGON_OFFSET_FILL );
}

// void Render::DrawEmptyAABB(Map* map, const Shader& shader) {
//    glDisable( GL_POLYGON_OFFSET_FILL );

//    for (int i = 0; i < map->empty_map_.size(); ++i)
//    {
//        vec3 aabb_min = map->empty_map_[i].first;
//        vec3 aabb_max = map->empty_map_[i].second;

//        shader.setVec3("aabbMin", aabb_min);
//        shader.setVec3("aabbMax", aabb_max);
//        RenderCube();
//    }

//    glPolygonOffset(1.0f, 1.0f);
//    glEnable( GL_POLYGON_OFFSET_FILL );
// }

//void Render::DrawRoomAABB(Map* map, const Shader& shader) {
//    glDisable( GL_POLYGON_OFFSET_FILL );
//
//    for (int i = 0; i < map->sections_map_.size(); ++i)
//    {
//        vec3 aabb_min = map->sections_map_[i].first;
//        vec3 aabb_max = map->sections_map_[i].second;
//
//        shader.setVec3("aabbMin", aabb_min);
//        shader.setVec3("aabbMax", aabb_max);
//        RenderCube();
//    }
//
//    glPolygonOffset(1.0f, 1.0f);
//    glEnable( GL_POLYGON_OFFSET_FILL );
//}

void Render::Draw(RenderWorld* world, 
                  const Shader& shader,
                  const glm::vec3 aabb_min,
                  const glm::vec3 aabb_max,
                  const bool skip_robot) 
{
    auto do_drawing = [&](const RenderPart* c, bool is_root) {
        int id_r, id_g, id_b;
        IdToColor(c->id(), id_r, id_g, id_b);
        glm::vec3 id_color(id_r/255.0f, id_g/255.0f, id_b/255.0f);

        for (size_t i = 0; i < c->size(); ++i) {
            ModelData* model = c->model_data(i);
            OriginTransformation* transform = c->transform(i);

            glm::mat4 translate = c->translation_matrix();
            glm::mat4 local_frame = 
                    glm::inverse(c->local_inertial_frame());
            glm::mat4 scale =  glm::scale(
                    glm::mat4(1), transform->local_scale);

            shader.setMat4("matrices.state", translate * local_frame);
            shader.setMat4("matrices.scale", scale);
            if(!is_root)
                shader.setMat4("matrices.model", transform->origin);
            else
                shader.setMat4("matrices.model", glm::mat4(1));
            shader.setFloat("flip", transform->flip);
            shader.setVec3("id_color", id_color);
            model->Draw(shader);
        }
    };

    for (size_t i = 0; i < world->size(); ++i) {
        RenderBody* body = world->render_body_ptr(i);

        // Skip Multibody
        if(skip_robot && body->size() > 0)
            continue;

        if(skip_robot && body->move())
            continue;

        // Root
        RenderPart* root = body->render_root_ptr();

        // Skip Root If Outside View Frustrum
        glm::vec3 root_aabb_min, root_aabb_max;
        root->GetAABB(root_aabb_min, root_aabb_max);
        if(!((aabb_min.x < root_aabb_max.x && aabb_max.x > root_aabb_min.x) &&
                (aabb_min.z < root_aabb_max.z && aabb_max.z > root_aabb_min.z))
        ) continue;

        if (root && !body->recycle()) {
            do_drawing(root, true);
            // Parts
            for (size_t j = 0; j < body->size(); ++j) {
                if (body->render_part_ptr(j)) {
                    do_drawing(body->render_part_ptr(j), false);
                }
            }
        }
    }
}

void Render::StepRenderFreeCamera(RenderWorld *world) {
    glm::mat4 projection = free_camera_.GetProjectionMatrix();
    glm::mat4 view = free_camera_.GetViewMatrix();

    Shader lambert_shader = shaders_[kLambert];
    Shader line_shader = shaders_[kLine];

    glBindFramebuffer(GL_FRAMEBUFFER, free_camera_framebuffer_->frameBuffer);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    lambert_shader.use();
    lambert_shader.setMat4("projection", projection);
    lambert_shader.setMat4("view", view);
    lambert_shader.setVec3("camPos", free_camera_.Position);
    lambert_shader.setInt("numDirectionalLight", use_sunlight_);
    lambert_shader.setVec3("light_directional", glm::vec3(-1) * sunlight_.direction);

    glActiveTexture(GL_TEXTURE5);
    glBindTexture(GL_TEXTURE_CUBE_MAP, reflection_map_);
    glUniform1i(glGetUniformLocation(lambert_shader.id(), "cmap"), 5);

    glm::vec3 minAABB, maxAABB;
    //GetViewFrusrumBoundingVolume( world->camera_list_[0], minAABB, maxAABB);
    GetViewFrusrumBoundingVolume(&free_camera_, minAABB, maxAABB);

    Draw(world, lambert_shader, minAABB - glm::vec3(1), maxAABB + glm::vec3(1));

    line_shader.use();
    line_shader.setMat4("projection", projection);
    line_shader.setMat4("view", view);

    line_shader.setVec3("color", glm::vec3(0.7,0.7,0.7));
    DrawRootAABB(world, line_shader);


    glEnable(GL_PROGRAM_POINT_SIZE);
    DrawBatchRay();

    //line_shader.setVec3("color", glm::vec3(0,1,0));
    //DrawEmptyAABB(map, line_shader);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

// Deferred Shading
void Render::StepRenderAllCamerasDeferred(RenderWorld* world,
                                  std::vector<int>& picks,
                                  const bool color_pick) {
    for (int i = 0; i < camera_framebuffer_list_.size(); ++i) {
        
        Camera* camera = world->camera(i);
        //Camera* camera = &free_camera_;

        if(settings_.use_shadow && !lighting_.force_disable_shadow) {
            StepRenderShadowMaps(world, camera);
        }

        if(settings_.use_vct && need_voxelize_) {
            VoxelizeScene(world);
        }

        if(settings_.vct_bake_before_simulate) {
            need_voxelize_ = false;
        }

        StepRenderGBuffer(world, camera);

        glBindFramebuffer(
                GL_FRAMEBUFFER, camera_framebuffer_list_[i]->frameBuffer);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        //DrawBackground(camera);

        VoxelConeTracing(world, camera);

        // glDepthFunc(GL_ALWAYS);
        // glDepthFunc(GL_LEQUAL);

        if(color_pick)
        {
            glFlush();
            glFinish();
            glReadBuffer(GL_COLOR_ATTACHMENT1); 
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

            unsigned char data[4];
            glReadPixels(
                    width_/2, height_/2, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, data); 
            picks[i] = ColorToId(data[0], data[1], data[2]);
        }

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
}

void Render::InitCombineTexture() {
    glGenFramebuffers(1, &combine_capture_fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, combine_capture_fbo_);

    glGenTextures(1, &combine_texture_);
    glBindTexture(GL_TEXTURE_2D, combine_texture_);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8,
            256, 64, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, 
            GL_TEXTURE_2D, combine_texture_, 0);

    glBindTexture(GL_TEXTURE_2D, 0);
}

void Render::StepCombineTexture() {
    glBindFramebuffer(GL_FRAMEBUFFER, combine_capture_fbo_);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glViewport(0, 0, 64, 64);
    shaders_[kCubemapVisualization].use();
    shaders_[kCubemapVisualization].setInt("dir", 5);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_CUBE_MAP, cubemap_texture_);
    glUniform1i(glGetUniformLocation(shaders_[kCubemapVisualization].id(), "tex"), 0);
    RenderQuad();

    glViewport(64, 0, 64, 64);
    shaders_[kCubemapVisualization].use();
    shaders_[kCubemapVisualization].setInt("dir", 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_CUBE_MAP, cubemap_texture_);
    glUniform1i(glGetUniformLocation(shaders_[kCubemapVisualization].id(), "tex"), 0);
    RenderQuad();

    glViewport(128, 0, 64, 64);
    shaders_[kCubemapVisualization].use();
    shaders_[kCubemapVisualization].setInt("dir", 4);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_CUBE_MAP, cubemap_texture_);
    glUniform1i(glGetUniformLocation(shaders_[kCubemapVisualization].id(), "tex"), 0);
    RenderQuad();

    glViewport(128 + 64, 0, 64, 64);
    shaders_[kCubemapVisualization].use();
    shaders_[kCubemapVisualization].setInt("dir", 1);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_CUBE_MAP, cubemap_texture_);
    glUniform1i(glGetUniformLocation(shaders_[kCubemapVisualization].id(), "tex"), 0);
    RenderQuad();

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

// Init Cubemap
void Render::InitCaptureCubemap2() {
    glGenFramebuffers(1, &cubemap_capture_fbo_);
    glGenRenderbuffers(1, &cubemap_capture_rbo_);

    glBindFramebuffer(GL_FRAMEBUFFER, cubemap_capture_fbo_);
    glBindRenderbuffer(GL_RENDERBUFFER, cubemap_capture_rbo_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, 128, 128);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, cubemap_capture_rbo_);

    glGenTextures(1, &cubemap_texture_);
    glBindTexture(GL_TEXTURE_CUBE_MAP, cubemap_texture_);

    for (unsigned int i = 0; i < 6; ++i) {
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGBA8,
                128, 128, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    }

    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
}

// This is the REAL ONE !
void Render::InitCaptureCubemap() {
    glGenFramebuffers(1, &cubemap_capture_fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, cubemap_capture_fbo_);

    glGenTextures(1, &cubemap_texture_);
    glBindTexture(GL_TEXTURE_CUBE_MAP, cubemap_texture_);

    for (unsigned int i = 0; i < 6; ++i) {
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_DEPTH_COMPONENT24,
                128, 128, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    }

    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
}

void Render::StepRenderCubemap(RenderWorld* world) {
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);

    // TODO 
    // Attach to Any Location

    Camera* camera = world->camera(0);
    glm::vec3 eye = camera->Position;
    glm::vec3 front = camera->Front;
    glm::vec3 up = camera->Up;

    glm::mat4 captureProjection = glm::perspective(glm::radians(90.0f), 1.0f, 0.01f, 20.0f);

    glm::mat4 captureViews[] = 
    {
       glm::lookAt(eye, eye + front, glm::vec3(0.0f, -1.0f,  0.0f)),
       glm::lookAt(eye, eye - front, glm::vec3(0.0f, -1.0f,  0.0f)),
       glm::lookAt(eye, eye + up, glm::vec3(0.0f,  0.0f,  1.0f)),
       glm::lookAt(eye, eye - up, glm::vec3(0.0f,  0.0f, -1.0f)),
       glm::lookAt(eye, eye + glm::cross(front, up), glm::vec3(0.0f, -1.0f,  0.0f)),
       glm::lookAt(eye, eye - glm::cross(front, up), glm::vec3(0.0f, -1.0f,  0.0f))
    };

    auto shader_capture = shaders_[kCaptureCubemap];

    shader_capture.use();
    shader_capture.setMat4("projection", captureProjection);

    // TODO
    // Changable Resolution

    glViewport(0, 0, 128, 128);
    glBindFramebuffer(GL_FRAMEBUFFER, cubemap_capture_fbo_);    
    for (unsigned int i = 0; i < 6; ++i)
    {
        if(i == 2 || i == 3) continue; // skip top and bottom
        shader_capture.setMat4("view", captureViews[i]);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, 
                GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, cubemap_texture_, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::vec3 minAABB, maxAABB;
        GetViewFrusrumBoundingVolume(eye, front, 20.0f,
                1.0f, 90.0f, captureViews[i], minAABB, maxAABB);

        Draw(world, shader_capture, minAABB, maxAABB);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);  
}

void Render::StepRenderAllCameras(RenderWorld* world,
                                  std::vector<int>& picks,
                                  const bool color_pick) {
    for (int i = 0; i < camera_framebuffer_list_.size(); ++i) {
        Shader lambert_shader = shaders_[kLambert];

        Camera* camera = world->camera(i);
        glm::mat4 projection = camera->GetProjectionMatrix();
        glm::mat4 view = camera->GetViewMatrix();


        // TODO
        // Shadow / SSAO in FS

        glBindFramebuffer(
                GL_FRAMEBUFFER, camera_framebuffer_list_[i]->frameBuffer); 
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


        lambert_shader.use();
        lambert_shader.setMat4("projection", projection);
        lambert_shader.setMat4("view", view);
        lambert_shader.setVec3("camPos", camera->Position);
        lambert_shader.setInt("numDirectionalLight", use_sunlight_);
        lambert_shader.setVec3("light_directional", glm::vec3(-1) * sunlight_.direction);


        glm::vec3 minAABB, maxAABB;
        GetViewFrusrumBoundingVolume(camera, minAABB, maxAABB);
        Draw(world, lambert_shader, minAABB - glm::vec3(3), maxAABB + glm::vec3(3));

        if(color_pick)
        {
            glFlush();
            glFinish();
            glReadBuffer(GL_COLOR_ATTACHMENT1); 
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

            unsigned char data[4];
            glReadPixels(
                    width_/2, height_/2, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, data); 
            picks[i] = ColorToId(data[0], data[1], data[2]);
        }

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
}

void Render::StepRenderGetDepthAllCameras() {
    glDisable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    img_buffers_.clear();

    for (int i = 0; i < camera_framebuffer_list_.size(); ++i) {     
        glBindFramebuffer(
                GL_FRAMEBUFFER, multiplied_framebuffer_list_[i]->frameBuffer);
        glViewport(0, 0, width_, height_);
        glClearColor(0,0,0,0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        Shader multiply_shader = shaders_[kMultiply];

        multiply_shader.use();

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, ssao_composite_tex_);
        glUniform1i(glGetUniformLocation(multiply_shader.id(), "tex"), 0);
        // camera_framebuffer_list_[i]->ActivateAsTexture(
        //     multiply_shader.id(), "tex", 0);

        camera_framebuffer_list_[i]->ActivateDepthAsTexture(
            multiply_shader.id(), "dep", 1);

        if(!settings_.use_deferred) {
            camera_framebuffer_list_[i]->ActivateDepthAsTexture(
                    multiply_shader.id(), "dep", 1);
        } else {
            multiply_shader.setFloat("deferred", 1);

            glActiveTexture(GL_TEXTURE2);
            glBindTexture(GL_TEXTURE_2D, g_normal_);
            glUniform1i(glGetUniformLocation(shaders_[kMultiply].id(), "mask"), 2);

            glActiveTexture(GL_TEXTURE1);
            glBindTexture(GL_TEXTURE_2D, g_position_);
            glUniform1i(glGetUniformLocation(shaders_[kMultiply].id(), "dep"), 1);
        }

        RenderQuad();

        pixel_buffer_index_ = (pixel_buffer_index_ + 1) % 2;
        pixel_buffer_next_index_ = (pixel_buffer_index_ + 1) % 2;


        glReadBuffer(GL_COLOR_ATTACHMENT0);           
        if(num_frames_ * (int)multiplied_framebuffer_list_.size() < 2) {
            glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB,
                            pixel_buffers_[pixel_buffer_index_]);
            glReadPixels(0, 0, width_, height_, GL_RGBA, GL_UNSIGNED_BYTE, 0);
            
            GLubyte* ptr = (GLubyte*)glMapBufferARB(
                    GL_PIXEL_PACK_BUFFER_ARB,GL_READ_ONLY_ARB);
            if (ptr) {
                Image image_temp;
                std::vector<unsigned char> temp(
                        ptr, ptr + (int)(width_*height_*4));
                image_temp.data = temp;
                image_temp.camera_id = i;
                img_buffers_.push_back(image_temp);
                
                glUnmapBufferARB(GL_PIXEL_PACK_BUFFER_ARB);
            }
            
        } else {
            glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB,
                            pixel_buffers_[pixel_buffer_next_index_]);
            GLubyte* ptr = (GLubyte*)glMapBufferARB(
                    GL_PIXEL_PACK_BUFFER_ARB,GL_READ_ONLY_ARB);
            if (ptr) {
                Image image_temp;
                std::vector<unsigned char> temp(
                        ptr, ptr + (int)(width_*height_*4));
                image_temp.data = temp;
                image_temp.camera_id = i;
                img_buffers_.push_back(image_temp);
                
                glUnmapBufferARB(GL_PIXEL_PACK_BUFFER_ARB);
            }
            glReadPixels(0, 0, width_, height_, GL_RGBA, GL_UNSIGNED_BYTE, 0);
        }
        glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, 0);

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
}

void Render::StepRenderGBuffer(RenderWorld* world, Camera * camera) {
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glDepthMask(GL_TRUE);
    glColorMask(true, true, true, true);
    glViewport(0, 0, width_, height_);
    glClearColor(0, 0, 0, 0);

    auto gShader = shaders_[kGBuffer];

    glm::mat4 projection = camera->GetProjectionMatrix();
    glm::mat4 view = camera->GetViewMatrix();

    glBindFramebuffer(GL_FRAMEBUFFER, g_buffer_fbo_);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    gShader.use();
    gShader.setMat4("projection", projection);
    gShader.setMat4("view", view);
    gShader.setVec3("viewPos", camera->Position);

    glm::vec3 minAABB, maxAABB;
    GetViewFrusrumBoundingVolume(camera, minAABB, maxAABB);

    Draw(world, gShader, minAABB - glm::vec3(3), maxAABB + glm::vec3(3));

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Render::DrawBackground(Camera * camera) {
    auto backgroundShader = shaders_[kSkybox];

    glm::mat4 projection = camera->GetProjectionMatrix();
    glm::mat4 view = camera->GetViewMatrix();

    backgroundShader.use();
    backgroundShader.setMat4("projection", projection);
    backgroundShader.setMat4("view", view);
    backgroundShader.setInt("environmentMap", 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_CUBE_MAP, environment_map_);
    RenderBox();
}

void Render::Visualization() {

    glEnable(GL_BLEND);
    glEnable(GL_LINE_SMOOTH);
    glDisable(GL_DEPTH_TEST);
    glClearColor(0.5, 0.5, 0.5, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);   

    // glViewport(0, 0, width_, height_);
    // shaders_[kColor].use();
    // camera_framebuffer_list_[0]->ActivateAsTexture(shaders_[kColor].id(), "tex", 0);
    // RenderQuad();

    // glViewport(0, 0, width_, height_);
    // shaders_[kColor].use();
    // glActiveTexture(GL_TEXTURE0);
    // glBindTexture(GL_TEXTURE_2D, ssao_composite_tex_);
    // glUniform1i(glGetUniformLocation(shaders_[kColor].id(), "tex"), 0);
    // RenderQuad();

    // glViewport(width_, 0, width_, height_);
    // shaders_[kColor].use();
    // free_camera_framebuffer_->ActivateAsTexture(shaders_[kColor].id(), "tex", 0);
    // RenderQuad();


    #ifdef DEBUG
        shaders_[kCrossHair].use();
        glViewport(0, 0, width_, height_);
        RenderMarker();

        glViewport(width_, 0, width_, height_);
        shaders_[kColor].use();
        free_camera_framebuffer_->ActivateAsTexture(shaders_[kColor].id(), "tex", 0);
        RenderQuad();
    #endif

    glViewport(0, 0, width_, height_);
    shaders_[kColor].use();
    multiplied_framebuffer_list_[0]->ActivateAsTexture(shaders_[kColor].id(), "tex", 0);
    RenderQuad();

    glViewport(2 * width_ / 3, 2 * height_ / 3, width_ / 3, height_ / 3);
    shaders_[kDepth].use();
    multiplied_framebuffer_list_[0]->ActivateAsTexture(shaders_[kDepth].id(), "tex", 0);
    RenderQuad();


    // glViewport(0, 0, 400, 100);
    // shaders_[kColor].use();
    // glActiveTexture(GL_TEXTURE0);
    // glBindTexture(GL_TEXTURE_2D, combine_texture_);
    // glUniform1i(glGetUniformLocation(shaders_[kColor].id(), "tex"), 0);
    // RenderQuad();

    // glViewport(width_, 0, 100, 100);
    // shaders_[kCubemapVisualization].use();
    // shaders_[kCubemapVisualization].setInt("dir", 5);
    // glActiveTexture(GL_TEXTURE0);
    // glBindTexture(GL_TEXTURE_CUBE_MAP, cubemap_texture_);
    // glUniform1i(glGetUniformLocation(shaders_[kCubemapVisualization].id(), "tex"), 0);
    // RenderQuad();

    // glViewport(width_ + 100, 0, 100, 100);
    // shaders_[kCubemapVisualization].use();
    // shaders_[kCubemapVisualization].setInt("dir", 0);
    // glActiveTexture(GL_TEXTURE0);
    // glBindTexture(GL_TEXTURE_CUBE_MAP, cubemap_texture_);
    // glUniform1i(glGetUniformLocation(shaders_[kCubemapVisualization].id(), "tex"), 0);
    // RenderQuad();

    // glViewport(width_ + 200, 0, 100, 100);
    // shaders_[kCubemapVisualization].use();
    // shaders_[kCubemapVisualization].setInt("dir", 4);
    // glActiveTexture(GL_TEXTURE0);
    // glBindTexture(GL_TEXTURE_CUBE_MAP, cubemap_texture_);
    // glUniform1i(glGetUniformLocation(shaders_[kCubemapVisualization].id(), "tex"), 0);
    // RenderQuad();

    // glViewport(width_ + 300, 0, 100, 100);
    // shaders_[kCubemapVisualization].use();
    // shaders_[kCubemapVisualization].setInt("dir", 1);
    // glActiveTexture(GL_TEXTURE0);
    // glBindTexture(GL_TEXTURE_CUBE_MAP, cubemap_texture_);
    // glUniform1i(glGetUniformLocation(shaders_[kCubemapVisualization].id(), "tex"), 0);
    // RenderQuad();

    // glViewport(width_, 0, 128, 128);
    // shaders_[kColor].use();
    // multiplied_framebuffer_list_[0]->ActivateAsTexture(shaders_[kColor].id(), "tex", 0);
    // RenderQuad();

    // glViewport(width_, 0, 128, 128);
    // shaders_[kColor].use();
    // multiplied_framebuffer_list_[0]->ActivateAsTexture(shaders_[kColor].id(), "tex", 0);
    // RenderQuad();

    // glViewport(width_, 0, 128, 128);
    // shaders_[kColor].use();
    // multiplied_framebuffer_list_[0]->ActivateAsTexture(shaders_[kColor].id(), "tex", 0);
    // RenderQuad();

    // glViewport(width_, 0, 128, 128);
    // shaders_[kColor].use();
    // multiplied_framebuffer_list_[0]->ActivateAsTexture(shaders_[kColor].id(), "tex", 0);
    // RenderQuad();

    // glViewport(width_, 0, 128, 128);
    // shaders_[kColor].use();
    // multiplied_framebuffer_list_[0]->ActivateAsTexture(shaders_[kColor].id(), "tex", 0);
    // RenderQuad();

    // glViewport(0, 0, width_, height_);
    // shaders_[kColor].use();
    // multiplied_framebuffer_list_[0]->ActivateAsTexture(shaders_[kColor].id(), "tex", 0);
    // RenderQuad();

    // glViewport(width_, 0, width_ / 2, height_ /2);
    // shaders_[kDepth].use();
    // multiplied_framebuffer_list_[0]->ActivateAsTexture(shaders_[kDepth].id(), "tex", 0);
    // RenderQuad();

    // glViewport(width_, 0, width_ / 4, height_ / 4);
    // shaders_[kDepth].use();
    // glActiveTexture(GL_TEXTURE0);
    // glBindTexture(GL_TEXTURE_2D, csm_.shadow_map()[0]);
    // glUniform1i(glGetUniformLocation(shaders_[kDepth].id(), "tex"), 0);
    // RenderQuad();

    // glViewport(width_, height_ / 4, width_ / 4,height_ / 4);
    // shaders_[kDepth].use();
    // glActiveTexture(GL_TEXTURE0);
    // glBindTexture(GL_TEXTURE_2D, csm_.shadow_map()[1]);
    // glUniform1i(glGetUniformLocation(shaders_[kDepth].id(), "tex"), 0);
    // RenderQuad();

    // glViewport(width_ * 1.25, 0, width_ / 4, height_ / 4);
    // shaders_[kDepth].use();
    // glActiveTexture(GL_TEXTURE0);
    // glBindTexture(GL_TEXTURE_2D, csm_.shadow_map()[2]);
    // glUniform1i(glGetUniformLocation(shaders_[kDepth].id(), "tex"), 0);
    // RenderQuad();

    // glViewport(width_ * 1.25, height_ / 4, width_ / 4,height_ / 4);
    // shaders_[kDepth].use();
    // glActiveTexture(GL_TEXTURE0);
    // glBindTexture(GL_TEXTURE_2D, csm_.shadow_map()[3]);
    // glUniform1i(glGetUniformLocation(shaders_[kDepth].id(), "tex"), 0);
    // RenderQuad();

    //VisualizeVoxels();      

    // glViewport(width_, 0, width_ / 3, height_ / 3);
    // shaders_[kColor].use();
    // glActiveTexture(GL_TEXTURE0);
    // glBindTexture(GL_TEXTURE_2D, g_albedospec_);
    // glUniform1i(glGetUniformLocation(shaders_[kColor].id(), "tex"), 0);
    // RenderQuad();

    // glViewport(width_ + width_ / 2, 0, width_ / 2, height_ / 2);
    // shaders_[kDepth].use();
    // glActiveTexture(GL_TEXTURE0);
    // glBindTexture(GL_TEXTURE_2D, g_position_);
    // glUniform1i(glGetUniformLocation(shaders_[kColor].id(), "tex"), 0);
    // RenderQuad();

    // glViewport(width_ * 2, 0, width_ / 2, height_ / 2);
    // shaders_[kColor].use();
    // glActiveTexture(GL_TEXTURE0);
    // glBindTexture(GL_TEXTURE_2D, g_normal_);
    // glUniform1i(glGetUniformLocation(shaders_[kColor].id(), "tex"), 0);
    // RenderQuad();

            // camera_framebuffer_list_[i]->ActivateAsTexture(
        //         shaders_[kColor].id(), "tex", 0);

    // for (int i = 0; i < multiplied_framebuffer_list_.size(); ++i) {
    //     glViewport(width_*2, i*height_, width_, height_);
    //     shaders_[kDepth].use();
    //     camera_framebuffer_list_[i]->ActivateDepthAsTexture(
    //             shaders_[kDepth].id(), "tex", 0);
    //     RenderQuad();
    // }
}

int Render::StepRender(RenderWorld* world, int pick_camera_id) {
    GetDeltaTime();
    //GetFrameRate();

    ProcessMouse();
    ProcessInput();

    int pick_result = -1;
    std::vector<int> picks(camera_framebuffer_list_.size(), -1);

    // StepRenderCubemap(world);
    // StepCombineTexture();
    glEnable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH);
    glDepthMask(GL_TRUE);
    glColorMask(true, true, true, true);
    glViewport(0, 0, width_, height_);
    glClearColor(
            background_color_.x, background_color_.y, background_color_.z, 1.0f); 

    #ifdef DEBUG
        StepRenderFreeCamera(world);
    #endif

    if(!settings_.use_deferred) {
        StepRenderAllCameras(world, picks, pick_camera_id >= 0);
    } else {
        StepRenderAllCamerasDeferred(world, picks, pick_camera_id >= 0);
    }

    if(pick_camera_id >= 0)
        pick_result = picks[pick_camera_id];

    SSAO(world);
    StepRenderGetDepthAllCameras();
    Visualization();  

    num_frames_++;
    return pick_result;
}

void Render::InitSSAO() {

    glGenFramebuffers(1, &ssao_composite_fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, ssao_composite_fbo_);
    glGenTextures(1, &ssao_composite_tex_);
    glBindTexture(GL_TEXTURE_2D, ssao_composite_tex_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, width_, height_, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, ssao_composite_tex_, 0);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        std::cout << "SSAO Framebuffer not complete!" << std::endl;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);


    glGenFramebuffers(1, &ssao_fbo_); 
    glGenFramebuffers(1, &ssao_blur_fbo_);

    glBindFramebuffer(GL_FRAMEBUFFER, ssao_fbo_);
    glGenTextures(1, &ssao_tex_);
    glBindTexture(GL_TEXTURE_2D, ssao_tex_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, width_, height_, 0, GL_RGB, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, ssao_tex_, 0);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        std::cout << "SSAO Framebuffer not complete!" << std::endl;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);


    glBindFramebuffer(GL_FRAMEBUFFER, ssao_blur_fbo_);
    glGenTextures(1, &ssao_tex_blur_);
    glBindTexture(GL_TEXTURE_2D, ssao_tex_blur_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, width_, height_, 0, GL_RGB, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, ssao_tex_blur_, 0);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        std::cout << "SSAO Blur Framebuffer not complete!" << std::endl;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);


    std::uniform_real_distribution<GLfloat> randomFloats(0.0, 1.0);
    std::default_random_engine generator;
    for (unsigned int i = 0; i < 64; ++i)
    {
        glm::vec3 sample(randomFloats(generator) * 2.0 - 1.0, randomFloats(generator) * 2.0 - 1.0, randomFloats(generator));
        sample = glm::normalize(sample);
        sample *= randomFloats(generator);
        float scale = float(i) / 64.0;

        scale = glm::mix(0.1f, 1.0f, scale * scale);
        sample *= scale;
        ssaoKernel.push_back(sample);
    }


    std::vector<glm::vec3> ssaoNoise;
    for (unsigned int i = 0; i < 16; i++)
    {
        glm::vec3 noise(randomFloats(generator) * 2.0 - 1.0, randomFloats(generator) * 2.0 - 1.0, 0.0f); // rotate around z-axis (in tangent space)
        ssaoNoise.push_back(noise);
    }

    glGenTextures(1, &noise_tex_);
    glBindTexture(GL_TEXTURE_2D, noise_tex_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, 4, 4, 0, GL_RGB, GL_FLOAT, &ssaoNoise[0]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Render::SSAO(RenderWorld* world) {
    // TODO
    // 1. Move All Post-Processing into a Class
    // 2. For Texture! Not for Camera

    glDisable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glClearColor(0,0,0,0);

    // SSAO Pass
    {
        auto shaderSSAO = shaders_[kSSAO];

        Camera* camera = world->camera(0);
        glm::mat4 projection = camera->GetProjectionMatrix();
        glm::mat4 view = camera->GetViewMatrix();

        glBindFramebuffer(GL_FRAMEBUFFER, ssao_fbo_);
            glViewport(0, 0, width_, height_);
            glClear(GL_COLOR_BUFFER_BIT);
            shaderSSAO.use();
            for (unsigned int i = 0; i < 64; ++i)
                shaderSSAO.setVec3("samples[" + std::to_string(i) + "]", ssaoKernel[i]);
            shaderSSAO.setMat4("projection", projection);
            shaderSSAO.setMat4("view", view);
            shaderSSAO.setInt("gPosition", 0);
            shaderSSAO.setInt("gNormal", 1);
            shaderSSAO.setInt("texNoise", 2);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, g_position_);
            glActiveTexture(GL_TEXTURE1);
            glBindTexture(GL_TEXTURE_2D, g_normal_);
            glActiveTexture(GL_TEXTURE2);
            glBindTexture(GL_TEXTURE_2D, noise_tex_);
            RenderQuad();
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }


    // Blur
    {
        auto shaderSSAOBlur = shaders_[kBlur];

        glBindFramebuffer(GL_FRAMEBUFFER, ssao_blur_fbo_);
            glClear(GL_COLOR_BUFFER_BIT);
            shaderSSAOBlur.use();
            shaderSSAOBlur.setInt("ssaoInput", 0);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, ssao_tex_);
            RenderQuad();
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }


    // Composite
    {
        auto shaderComposite = shaders_[kComposite];

        glBindFramebuffer(GL_FRAMEBUFFER, ssao_composite_fbo_);
            glClear(GL_COLOR_BUFFER_BIT);
            shaderComposite.use();
            shaderComposite.setInt("ssao", 0);
            shaderComposite.setInt("src", 1);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, ssao_tex_blur_);
            camera_framebuffer_list_[0]->ActivateAsTexture(shaderComposite.id(), "tex", 1);
            RenderQuad();
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
    
}

void Render::InitFXAA() {
    glGenFramebuffers(1, &fxaa_fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, fxaa_fbo_);

    glGenTextures(1, &fxaa_tex_);
    glBindTexture(GL_TEXTURE_2D, fxaa_tex_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width_, height_, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fxaa_tex_, 0);
}

void Render::FXAA() {

    // One Camera Only
    assert(camera_framebuffer_list_.size() < 2);

    glDisable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glClearColor(0,0,0,0);
 
    glBindFramebuffer(GL_FRAMEBUFFER, fxaa_fbo_);
    glViewport(0, 0, width_, height_);
    glClear(GL_COLOR_BUFFER_BIT);

    shaders_[kFXAA].use();
    shaders_[kFXAA].setInt("tex", 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, ssao_composite_tex_);
    //camera_framebuffer_list_[0]->ActivateAsTexture(shaders_[kColor].id(), "tex", 0);
    RenderQuad();

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Render::InitShaders() {
    std::string pwd = get_pwd(__FILE__);
    shaders_.resize((int) kShaderTypeSize);
    shaders_[kLambert] = Shader(pwd+"/shaders/lambert.vs",
                                pwd+"/shaders/lambert.fs");
    
    shaders_[kDepth] = Shader(pwd+"/shaders/quad.vs",
                              pwd+"/shaders/depth.fs");
    
    shaders_[kColor] = Shader(pwd+"/shaders/quad.vs",
                              pwd+"/shaders/flat.fs");

    shaders_[kLine] = Shader(pwd+"/shaders/line.vs",
                             pwd+"/shaders/line.fs");

    shaders_[kCrossHair] = Shader(pwd+"/shaders/crosshair.vs",
                                  pwd+"/shaders/line.fs");

    shaders_[kMultiply] = Shader(pwd+"/shaders/quad.vs",
                                 pwd+"/shaders/premult.fs");

    // Cone Tracing (gl 4.3 required)
    if(true || settings_.use_vct)
    {
        shaders_[kVoxelize] = Shader(pwd+"/shaders/voxelize.vs",
                                     pwd+"/shaders/voxelize.fs",
                                     pwd+"/shaders/voxelize.gs",
                                     -1);

        shaders_[kVoxelVisualization] = Shader(pwd+"/shaders/draw_voxels.vs",
                                               pwd+"/shaders/draw_voxels.fs",
                                               pwd+"/shaders/draw_voxels.gs",
                                               -1);

        shaders_[kRadianceInjection] = Shader(pwd+"/shaders/inject.comp");

        shaders_[kRadiancePropagation] = Shader(pwd+"/shaders/prop.comp");

        shaders_[kMipMapBase] = Shader(pwd+"/shaders/mip_base.comp");

        shaders_[kMipMapVolume] = Shader(pwd+"/shaders/mip_vol.comp");

        shaders_[kClearBuffer] = Shader(pwd+"/shaders/clear.comp");
    }

    shaders_[kVoxelConeTracing] = Shader(pwd+"/shaders/quad.vs",
                                         pwd+"/shaders/cone_trace.fs");

    // Deferred
    if(true || settings_.use_deferred)
    {
        shaders_[kGBuffer] = Shader(pwd+"/shaders/lambert.vs",
                                    pwd+"/shaders/g.fs");
    }

    // Physically Based Rendering
    if(true || settings_.use_pbr)
    {
        shaders_[kConvertToCubemap] = Shader(pwd+"/shaders/cubemap.vs",
                                             pwd+"/shaders/to_cubemap.fs");

        shaders_[kIrrandianceConv] = Shader(pwd+"/shaders/cubemap.vs",
                                            pwd+"/shaders/irradiance.fs");

        shaders_[kPrefilter] = Shader(pwd+"/shaders/cubemap.vs",
                                      pwd+"/shaders/prefilter.fs");

        shaders_[kBRDF] = Shader(pwd+"/shaders/quad.vs",
                                 pwd+"/shaders/brdf.fs");
    }

    // Cascade Shadow Map
    if(true || settings_.use_shadow)
    {
        shaders_[kPSSM] = Shader(pwd+"/shaders/csm.vs",
                                 pwd+"/shaders/csm.fs");
    }

    shaders_[kCaptureCubemap] = Shader(pwd+"/shaders/capture.vs",
                                       pwd+"/shaders/capture.fs");

    shaders_[kCubemapVisualization] = Shader(pwd+"/shaders/quad.vs",
                                             pwd+"/shaders/cube_visualize.fs");

    shaders_[kRay] = Shader(pwd+"/shaders/ray.vs",
                            pwd+"/shaders/ray.fs",
                            pwd+"/shaders/ray.gs",
                            -1);

    shaders_[kSkybox] = Shader(pwd+"/shaders/skybox.vs",
                               pwd+"/shaders/skybox.fs");

    shaders_[kFXAA] = Shader(pwd+"/shaders/quad.vs",
                               pwd+"/shaders/fxaa.fs");

    shaders_[kSSAO] = Shader(pwd+"/shaders/quad.vs",
                               pwd+"/shaders/ssao.fs");

    shaders_[kBlur] = Shader(pwd+"/shaders/quad.vs",
                               pwd+"/shaders/blur.fs");

    shaders_[kComposite] = Shader(pwd+"/shaders/quad.vs",
                               pwd+"/shaders/comp.fs");
}

void Render::GetDeltaTime() {
    TimeFrame current_frame = std::chrono::high_resolution_clock::now();
    delta_time_ = std::chrono::duration_cast<std::chrono::duration<double>>(
            current_frame - last_frame_).count();
    last_frame_ = current_frame;
}

void Render::GetFrameRate() {
    current_framerate_ = 1.0f / delta_time_;
    avg_framerate_ += current_framerate_;
    all_rendered_frames_ += 1;
    max_framerate_ = glm::max(max_framerate_, current_framerate_);

    std::string framerate_str = "FPS: " 
        + std::to_string((int)current_framerate_);
        // + " Avg: " + std::to_string((int)(avg_framerate_/all_rendered_frames_))
        // + " Max: " + std::to_string((int)max_framerate_);

    printf("%s\n", framerate_str.c_str());
    //ctx_->SetTitle(framerate_str.c_str());
}

} } // xrobot::render_engine
