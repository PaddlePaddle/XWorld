#include "render_world.h"

namespace xrobot {
namespace render_engine {

void RenderWorld::render_step() {
    for (auto& kv : camera_to_body_) {
        auto camera = kv.first;
        auto body  = kv.second;
        if (!camera || !body) continue;
        attach_camera(camera, body);
    }
}

Camera* RenderWorld::camera(int i) {
    assert(i >= 0 && i < cameras_.size());
    return cameras_[i];
}

void RenderWorld::remove_all_cameras() {
    for (auto c : cameras_) {
        delete c;
    }
    cameras_.clear();
    camera_to_body_.clear();
}

void RenderWorld::attach_camera(Camera* camera, RenderBody* body) {
    if(!camera || !body) return;
    body->attach_camera(camera->offset_,
                        camera->pre_pitch_,
                        camera->position_,
                        camera->front_,
                        camera->right_,
                        camera->up_);
    camera_to_body_[camera] = body;
}

Camera* RenderWorld::add_camera(const glm::vec3& position,
                                const glm::vec3& offset, 
                                const float aspect_ratio,
                                const float fov,
                                const float near,
                                const float far) {
    Camera* camera = new Camera(position);
    camera->SetNear(near);
    camera->SetFar(far);
    camera->SetFOV(fov);
    camera->SetAspect(aspect_ratio);
    camera->offset_ = offset;
    cameras_.push_back(camera);
    return camera;
}

void RenderWorld::detach_camera(Camera* camera) {
    if (!camera) return;
    camera_to_body_.erase(camera);
}

void RenderWorld::remove_camera(Camera* camera) {
    if (!camera) return;
    auto it = find(cameras_.begin(), cameras_.end(), camera);
    if (it != cameras_.end()) {
        delete *it;
        cameras_.erase(it);
    }
    camera_to_body_.erase(camera);
}

void RenderWorld::rotate_camera(Camera* camera, const float pitch) {
    if (!camera) return;
    camera->pre_pitch_ = pitch;
}

} } // xrobot::render_engine
