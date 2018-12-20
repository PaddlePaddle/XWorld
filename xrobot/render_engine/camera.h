#ifndef RENDER_ENGINE_CAMERA_H
#define RENDER_ENGINE_CAMERA_H

#include <vector>

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"

#define INCLUDE_GL_CONTEXT_HEADERS
#include "gl_header.h"

namespace xrobot {
namespace render_engine {

constexpr float kMovementSpeed     = 10.0f;
constexpr float kMouseSensitivity  = 0.5f;
constexpr float kFOV               = 40.0f;
constexpr float kNear              = 0.02f;
constexpr float kFar               = 100.0f;

class Camera {
public:
    enum Movement {
        kForward,
        kBackward,
        kLeft,
        kRight,
        kUp,
        kDown,
        kNumMovement
    };

    float pre_pitch_;
    float yaw_;
    float pitch_;
    glm::vec3 offset_;
    glm::vec3 position_;
    glm::vec3 front_;
    glm::vec3 up_;
    glm::vec3 right_;
    glm::vec3 world_up_;
    
    Camera(const glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f),
           const glm::vec3 offset = glm::vec3(0.0f, 0.0f, 0.0f),
           const glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f),
           const float yaw = 0.0f,
           const float pitch = 0.0f);

    float GetFOVRadius() const { return glm::radians(fov_); }
    float GetFOV() const { return fov_; }
    float GetNear() const { return near_; }
    float GetFar() const { return far_; }
    float GetAspect() const { return aspect_; }
    void SetFOV(const float fov) { fov_ = fov; }
    void SetNear(const float near) { near_ = near; }
    void SetFar(const float far) { far_ = far; }
    void SetAspect(const float aspect) { aspect_ = aspect; } 

    glm::mat4 GetViewMatrix() const {
        return glm::lookAt(position_, position_ + front_, up_);
    }
    
    glm::mat4 GetProjectionMatrix() const {
        return glm::perspective(glm::radians(fov_), aspect_, near_, far_);
    }
    
    void ProcessKeyboard(const Movement direction, const float delta_time);
    void ProcessMouseMovement(const float xoffset,
                              const float yoffset,
                              const bool lock = true);
    void Update();

private:
    float move_speed_;
    float sensitivity_;
    float fov_;
    float near_;
    float far_;
    float aspect_;
};

} } // xrobot::render_engine

#endif
