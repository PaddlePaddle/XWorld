#ifndef RENDER_ENGINE_COMMON_H_
#define RENDER_ENGINE_COMMON_H_

#include <chrono>
#include <ctime>
#include <ratio>
#include <stdio.h>
#include <string>
#include <vector>

#include "gl_context.h"
#include "shader.h"
#include "csm.h"
#include "texture2d.h"
#include "texture3d.h"
#include "render_target.h"

namespace xrobot {
namespace render_engine {

struct Profile {
    bool shading;
    bool shadow;
    bool ao;
    bool ssr;
    bool vct;
    bool fxaa;
    bool visualize;
    bool multirays;
};

constexpr int kLidarCaptureRes = 64;

const struct Profile kVeryLowQuality = {
        false, false, false, false, false, false, false, false};

const struct Profile kLowQuality = {
        true, false, false, false, false, false, false, false};

const struct Profile kNormalQualityNS = {
        true, false, true, true, false, true, false, false};

const struct Profile kNormalQuality = {
        true, true, true, true, false, true, false, false};

const struct Profile kHighQuality = {
        true, true, true, false, true, true, false, false};

const struct Profile kVeryLowQualityVisualize = {
        false, false, false, false, false, false, true, false};

const struct Profile kLowQualityVisualize  = {
        true, false, false, false, false, false, true, false};

const struct Profile kNormalQualityNSVisualize = {
        true, false, true, true, false, true, false, false};

const struct Profile kNormalQualityVisualize  = {
        true, true, true, false, false, true, true, false};

const struct Profile kHighQualityVisualize  = {
        true, true, true, false, true, true, true, false};

const Profile profiles[10] = {
        kVeryLowQuality, kLowQuality, kNormalQualityNS, kNormalQuality, kHighQuality,
        kVeryLowQualityVisualize, kLowQualityVisualize, kNormalQualityNSVisualize,
        kNormalQualityVisualize, kHighQualityVisualize};

struct Lighting {
    float exposure = 0.5f;
    glm::vec3 bg = glm::vec3(0.5f);
};

struct DirectionalLight {
    glm::vec3 direction = glm::vec3(1, 2, 1);
    glm::vec3 diffuse = glm::vec3(0.8f, 0.8f, 0.8f);
    glm::vec3 specular = glm::vec3(0.5f, 0.5f, 0.5f);
    glm::vec3 ambient = glm::vec3(0.1f, 0.1f, 0.1f);
};

struct PSSM {
    CSMUniforms csm_uniforms;
    CSM csm;
    int shadow_map_size = 2048;
    int cascade_count = 4;
    float pssm_lamda = 0.7f;
    float near_offset = 80.0f;
    float shadow_bias_clamp = 0.0005f;
    float shadow_bias_scale = 0.0005f;
    bool first_run = true;
};

template <typename T>
struct Image {
    Image() : data(0) {}
    std::vector<T> data;
};

static std::string get_pwd(const std::string& file) {
    size_t p = file.find_last_of("/");
    return file.substr(0, p);
}

}
}

#endif // RENDER_ENGINE_COMMON_H_
