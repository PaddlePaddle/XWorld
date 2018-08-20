#ifndef RENDER_ENGINE_RENDER_WORLD_H
#define RENDER_ENGINE_RENDER_WORLD_H

#include <cassert>
#include <vector>

#include "glm/glm.hpp"

#include "camera.h"
#include "model.h"

namespace xrobot {
namespace render_engine {

class RenderPart {
public:
    RenderPart() : model_list_(0), transform_list_(0) {}

    virtual ~RenderPart() {
        for (size_t i = 0; i < transform_list_.size(); ++i) {
            delete transform_list_[i];
        }
        transform_list_.clear();
    }

    virtual int id() const = 0;

    size_t size() const { return model_list_.size(); }

    ModelData* model_data(const size_t i) const {
        assert(i < model_list_.size() && model_list_[i]);
        return model_list_[i];
    }

    OriginTransformation* transform(const size_t i) const {
        assert(i < transform_list_.size());
        return transform_list_[i];
    }

    virtual void GetAABB(glm::vec3& aabb_min, glm::vec3& aabb_max) = 0;

    virtual glm::mat4 position() const = 0;

    virtual glm::mat4 local_inertial_frame() const = 0;

public:
    std::vector<ModelData*> model_list_;
    std::vector<OriginTransformation*> transform_list_;
};

class RenderBody {
public:
    RenderBody() : recycle_(false) {}

    virtual ~RenderBody() {}

    bool recycle() const { return recycle_; }

    void recycle(const bool value) { recycle_ = value; }

    virtual size_t size() const = 0;

    virtual const RenderPart* render_root_ptr() const = 0;
    virtual RenderPart* render_root_ptr() = 0;
    virtual const RenderPart* render_part_ptr(const size_t i) const = 0;
    virtual RenderPart* render_part_ptr(const size_t i) = 0;

protected:
    bool recycle_;
};

class RenderWorld {
public:
    RenderWorld() : camera_list_(0) {}
    virtual size_t size() const = 0;

    virtual const RenderBody* render_body_ptr(const size_t i) const = 0;
    virtual RenderBody* render_body_ptr(const size_t i) = 0;
// TODO: will change them to protected
public:
    std::vector<Camera*> camera_list_;
};

} } // xrobot::render_engine

#endif
