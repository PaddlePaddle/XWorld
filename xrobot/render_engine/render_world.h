#ifndef RENDER_ENGINE_RENDER_WORLD_H
#define RENDER_ENGINE_RENDER_WORLD_H

#include <cassert>
#include <memory>
#include <vector>

#include "glm/glm.hpp"

#include "camera.h"
#include "model.h"

namespace xrobot {
namespace render_engine {

class RenderPart {
public:
    RenderPart() : model_list_(0), transform_list_(0) {}

    virtual ~RenderPart() {}

    size_t size() const { return model_list_.size(); }

    virtual int id() const = 0;

    virtual void set_id(int id) = 0;

    ModelDataSPtr model_data(const size_t i) const {
        assert(i < model_list_.size() && model_list_[i]);
        return model_list_[i];
    }

    std::shared_ptr<OriginTransformation> transform(const size_t i) const {
        assert(i < transform_list_.size());
        return transform_list_[i];
    }

    virtual void GetAABB(glm::vec3& aabb_min, glm::vec3& aabb_max) = 0;

    virtual glm::mat4 translation_matrix() const = 0;

    virtual glm::mat4 local_inertial_frame() const = 0;

public:
    std::vector<ModelDataSPtr> model_list_;
    std::vector<std::shared_ptr<OriginTransformation>> transform_list_;
};

typedef std::shared_ptr<RenderPart> RenderPartSPtr;

class RenderBody {
public:
    RenderBody() : recycle_(false), baking_(false), hide_(false) {}

    virtual ~RenderBody() {}

    bool is_hiding() const { return hide_; }

    virtual void hide(const bool value) = 0; 

    bool ignore_baking() const { return baking_; }

    void ignore_baking(const bool value) { baking_ = value; }

    bool is_recycled() const { return recycle_; }

    virtual void recycle() = 0;

    virtual void reuse() = 0;

    virtual size_t size() const = 0;

    virtual const RenderPart* render_root_ptr() const = 0;
    virtual RenderPart* render_root_ptr() = 0;
    virtual const RenderPart* render_part_ptr(const size_t i) const = 0;
    virtual RenderPart* render_part_ptr(const size_t i) = 0;
    virtual void attach_camera(const glm::vec3& offset,
                               const float pitch,
                               glm::vec3& loc,
                               glm::vec3& front,
                               glm::vec3& right,
                               glm::vec3& up) = 0;

protected:
    bool hide_;
    bool baking_;
    bool recycle_;
};

class RenderWorld {
public:

    RenderWorld() : cameras_(0),
                    world_min_x_(-2),
                    world_min_z_(-2),
                    world_max_x_( 2),
                    world_max_z_( 2),
                    debug_subtiles_(0),
                    debug_subtile_status_(0),
                    highlight_center_(-1),
                    icon_cache_(),
                    icon_inventory_(0),
                    inventory_size_(-1) {}

    virtual size_t size() const = 0;

    void load_icon(const std::string& path) {
        if(icon_cache_.find(path) == icon_cache_.end()) {
            icon_cache_[path] = -1;
        }
    }

    void set_world_size(const float min_x,
                        const float min_z,
                        const float max_x,
                        const float max_z) {
        world_min_x_ = min_x;
        world_min_z_ = min_z;
        world_max_x_ = max_x;
        world_max_z_ = max_z;
    }

    void get_world_size(float& min_x,
                        float& min_z,
                        float& max_x,
                        float& max_z) const {
        min_x = world_min_x_;
        min_z = world_min_z_;
        max_x = world_max_x_;
        max_z = world_max_z_;
    }

    void set_highlight_center(const int highlight) {
        highlight_center_ = highlight;
    }

    int get_highlight_center() const { return highlight_center_; }

    virtual const RenderBody* render_body_ptr(const size_t i) const = 0;
 
    virtual RenderBody* render_body_ptr(const size_t i) = 0;

    void render_step();

    void remove_all_cameras();

    Camera* camera(int i);

    size_t cameras_size() { return cameras_.size(); }

    void attach_camera(Camera* camera, RenderBody* body);

    Camera* add_camera(const glm::vec3& position,
                       const glm::vec3& offset = glm::vec3(0),
                       const float aspect_ratio = 4.0f / 3.0f,
                       const float fov = 60.0f,
                       const float near = 0.02f,
                       const float far = 70.0f);
            
    void detach_camera(Camera* camera);

    void remove_camera(Camera* camera);

    void rotate_camera(Camera* camera, const float pitch);

protected:
    std::vector<Camera*> cameras_;
    std::map<Camera*, RenderBody*> camera_to_body_;

    float world_min_x_;
    float world_min_z_;
    float world_max_x_;
    float world_max_z_;

public:
    int highlight_center_;
    int inventory_size_;
    std::vector<std::string> icon_inventory_;
    std::map<std::string, int> icon_cache_;

    std::vector<std::pair<glm::vec2, glm::vec2>> debug_subtiles_;
    std::vector<int> debug_subtile_status_;
};

} } // xrobot::render_engine

#endif
