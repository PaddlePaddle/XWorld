#include "render_world.h"

namespace xrobot {
namespace render_engine {

void RenderWorld::render_step() {
    for (auto& kv : camera_to_body_) {
        std::cout << "attach camera" << std::endl;
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
    body->attach_camera(camera->Offset,
                        camera->Pre_Pitch,
                        camera->Position,
                        camera->Front,
                        camera->Right,
                        camera->Up);
    camera_to_body_[camera] = body;
}

Camera* RenderWorld::add_camera(const glm::vec3& position,
                                const glm::vec3& offset, 
                                const float aspect_ratio,
                                const float fov,
                                const float near,
                                const float far) {
    Camera* camera = new Camera(position);
    camera->Near = near;
    camera->Far = far;
    camera->Zoom = fov;
    camera->Offset = offset;
    camera->Aspect = aspect_ratio;
    cameras_.push_back(camera);
    return camera;
}

void RenderWorld::detach_camera(Camera* camera) {
    if (!camera) return;
    camera_to_body_.erase(camera);
}

void RenderWorld::remove_camera(Camera* camera) {
    if (!camera) return;
    auto it = std::find(cameras_.begin(), cameras_.end(), camera);
    if (it != cameras_.end()) {
        delete *it;
        cameras_.erase(it);
    }
    camera_to_body_.erase(camera);
}

void RenderWorld::rotate_camera(Camera* camera, const float pitch) {
    if (!camera) return;
    camera->Pre_Pitch = pitch;
}

} } // xrobot::render_engine
