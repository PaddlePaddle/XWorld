#include "world.h"
#include "utils.h"

namespace xrobot {

using namespace render_engine;
using namespace bullet_engine;

RobotBase::RobotBase(std::weak_ptr<World> bullet_world)
        : bullet_world_(bullet_world),
          root_part_(nullptr),
          parts_(),
          joints_() {}

void RobotBase::LoadURDFFile(
        const std::string& filename,
        const glm::vec3& pos,
        const glm::vec4& quat, //quat
        const xScalar scale,
        const std::string& label,
        const bool fixed_base,
        const bool self_collision,
        const bool use_multibody,
        const bool concave) {

    auto bullet_world = wptr_to_sptr(bullet_world_);
        
    BulletBody::load_urdf(
            bullet_world->client_,
            filename,
            pos,
            quat,
            scale,
            fixed_base,
            self_collision,
            use_multibody,
            concave);
    body_data_.label = label;

    load_robot_joints(filename);

    load_robot_shapes(scale);

    // TODO: return shared_ptr of RobotBase to caller (i.e. World) and let
    // caller handle this
    bullet_world->robot_list_.push_back(shared_from_this());
    bullet_world->id_to_robot_[body_data_.body_uid] = shared_from_this();

    reuse();
}

void RobotBase::load_robot_joints(const std::string &filename) {
    auto bullet_world = wptr_to_sptr(bullet_world_);

    root_part_ = std::make_shared<Object>();
    load_root_part(bullet_world->client_, root_part_.get());
    root_part_->bullet_world_ = bullet_world;
    root_part_->set_id(body_data_.body_uid);
    root_part_->EnableSleeping();

    int num_joints = get_num_joints(bullet_world->client_, body_data_.body_uid);
    joints_.resize(num_joints);
    parts_.resize(num_joints);

    for (int c = 0; c < num_joints; ++c) {

        parts_[c] = std::make_shared<Object>();
        joints_[c]= std::make_shared<Joint>();

        if (load_part(
                bullet_world->client_, c, joints_[c].get(), parts_[c].get())) {
            joints_[c]->bullet_robot_ = shared_from_this();
            joints_[c]->bullet_world_ = bullet_world;
        } else {
            joints_[c].reset();
        }

        parts_[c]->bullet_world_ = bullet_world;
        parts_[c]->set_id(body_data_.body_uid);
    }
}

void RobotBase::load_robot_shapes(const xScalar scale) {
    auto bullet_world = wptr_to_sptr(bullet_world_);

    int num_shapes = get_visual_shape_info(bullet_world->client_);
    assert(num_shapes >= 0);

    glm::vec4 color;
    glm::vec3 s;
    glm::vec3 pos;
    glm::vec4 quat;
    std::string filename;
    int geometry_type;
    for (int i = 0; i < num_shapes; ++i) {
        int link_id = get_visual_shape(
                i, color, s, pos, quat, filename, geometry_type);

        std::shared_ptr<Object> part;
        if(link_id == -1) {
            part = root_part_;
        } else {
            part = parts_[link_id];
        }

        glm::mat4 transform = TransformToMat4(TransformFromReals(pos, quat));

        auto T = std::make_shared<OriginTransformation>();
        T->origin = transform;
        T->scale = scale;
        T->color = color;

        bool create_new = true;
        ModelDataSPtr model_data = bullet_world->FindInCache(
                filename, part->model_list_, create_new);
        // why do we need to do this?
        if (geometry_type == kMesh && create_new) {
            model_data->directory_ = filename;
        }

        model_data->Reset(
                geometry_type,
                create_new,
                glm::vec3(s[0], s[1], s[2]),
                /*out*/T);

        part->transform_list_.push_back(T);
        part->bullet_link_id_ = link_id;
    }
}

void RobotBase::LoadOBJFile(
        const std::string& filename,
        const glm::vec3& pos,
        const glm::vec4& quat,
        const glm::vec3& scale,
        const std::string& label,
        const xScalar mass,
        const bool flip,
        const bool concave) {
    auto bullet_world = wptr_to_sptr(bullet_world_);

    root_part_ = std::make_shared<Object>();
    load_obj(
             bullet_world->client_,
             root_part_.get(),
             filename,
             pos,
             quat,
             scale,
             mass,
             concave);
    root_part_->bullet_world_ = bullet_world;
    root_part_->set_id(body_data_.body_uid);
    body_data_.label = label;

    bool reset = true;
    auto model_data = bullet_world->FindInCache(
            filename, root_part_->model_list_, /*out*/reset);
    // FindInCache may change the value of reset
    if (reset) {
        model_data->primitive_type_ = kMesh;
        model_data->directory_ = filename;
        model_data->Reset();
    }

    auto transform = TransformToMat4(TransformFromReals(pos, quat));
    auto origin_transform = std::make_shared<OriginTransformation>();
    origin_transform->origin = transform;
    origin_transform->scale = 1.0f;
    origin_transform->local_scale = glm::vec3(scale[0], scale[1], scale[2]);
    origin_transform->color = glm::vec4(1);
    origin_transform->flip = flip ? -1.0f : 1.0f;
    root_part_->transform_list_.push_back(origin_transform);
       
    // TODO: return shared_ptr of RobotBase to caller (i.e. World) and let
    // caller handle this
    bullet_world->robot_list_.push_back(shared_from_this());
    bullet_world->id_to_robot_[body_data_.body_uid] = shared_from_this();

    reuse();
}

void RobotBase::RemoveRobotFromBullet() {
    if (body_data_.body_uid < 0) return;
    auto bullet_world = wptr_to_sptr(bullet_world_);
    remove_from_bullet(bullet_world->client_, body_data_.body_uid);
}

void RobotBase::do_recycle(const std::string& key) {
    auto bullet_world = bullet_world_.lock();
    assert(bullet_world);
    recycle_ = true;
    bullet_world->recycle_robot_map_[key].push_back(shared_from_this());
    body_data_.attach_to_id = -2;
    // Remove Label
    bullet_world->RemoveObjectWithLabel(body_data_.body_uid);
    //bullet_world->id_to_robot_[key] = nullptr;

}

void RobotBase::recycle() {
    // hide
    hide(true);
    // recycle
    do_recycle(body_data_.urdf_name);
}

void RobotBase::Sleep() {
    if (body_data_.body_uid < 0) return;

    if (root_part_) {
        root_part_->Sleep();
    }

    for (auto& part : parts_) {
       if (part) {
            part->Sleep();
       }
    }
}

void RobotBase::Wake() {
    if(body_data_.body_uid < 0) return;

    if (root_part_) {
        root_part_->Wake();
    }

    for (auto& part : parts_) {
       if (part) {
            part->Wake();
       }
    }
}

void RobotBase::DisableSleeping() {
    if(body_data_.body_uid < 0) return;

    if (root_part_) {
        root_part_->DisableSleeping();
        root_part_->Wake();
    }

    for (auto part : parts_) {
       if (part) {
            part->DisableSleeping();
            part->Wake();
       }
    }
}
              
// Updated Version        
void RobotBase::Move(const xScalar translate, const xScalar rotate) {
    assert(root_part_);
    auto bullet_world = wptr_to_sptr(bullet_world_);
    xScalar pos[3];
    xScalar quat[4];
    xScalar prev_quat[4];
    xScalar prev_orn[4];
    xScalar prev_angle = angle_;
    move(translate, rotate, root_part_.get(), pos, quat, prev_quat, prev_orn);

    set_pose(bullet_world->client_, body_data_.body_uid, pos, quat);
    bullet_world->BulletStep();

    bool step = false;
    std::vector<ContactPoint> contact_points;
    bullet_world->GetContactPoints(
            shared_from_this(), root_part_, contact_points);
    for (auto& cp : contact_points) {
        glm::vec3 current_pos(pos[0], 0, pos[2]); // projected onto x-z plane
        glm::vec3 cn   = glm::normalize(cp.contact_normal);
        glm::vec3 cp_a = cp.contact_position_a;
        glm::vec3 cp_b = cp.contact_position_b;
        cp_b.y = 0; // projected onto x-z plane

        glm::vec3 dir = glm::normalize(cp_b - current_pos);
        if (cp.contact_distance < -0.01 && abs(cn[1]) < 0.2) { // hack
            int sign = 1;
            xScalar delta = glm::clamp(cp.contact_distance, -0.05f, 0.0f);
            auto& object_a = bullet_world->id_to_robot_[cp.bullet_id_a]; 
            auto& object_b = bullet_world->id_to_robot_[cp.bullet_id_b];
            
            // btVector3 contact_normal(to_object.x, to_object.y, to_object.z);
            pos[0] += sign * dir[0] * delta;
            pos[1] += sign * dir[1] * delta;
            pos[2] += sign * dir[2] * delta;
            step = true;
        }
    }

    if(step) {

        if(fabs(rotate) > 0.0f) {
            angle_ = prev_angle;
            orientation_[0] = prev_orn[0];
            orientation_[1] = prev_orn[1];
            orientation_[2] = prev_orn[2];
            orientation_[3] = prev_orn[3];
        }

        set_pose(bullet_world->client_, body_data_.body_uid, pos, prev_quat);
        bullet_world->BulletStep();
    }
}

void RobotBase::UnFreeze() { Move(0, 0); }

void RobotBase::Freeze() { Move(0, 0); }

void RobotBase::MoveForward(const xScalar speed) { Move(0.005*speed, 0); }

void RobotBase::MoveBackward(const xScalar speed) { Move(-0.005*speed, 0); }

void RobotBase::TurnLeft(const xScalar speed) { Move(0, 0.005*speed); }

void RobotBase::TurnRight(const xScalar speed) { Move(0, -0.005*speed); }
 
void RobotBase::SetJointVelocity(
        const int joint_id,
        const xScalar speed,
        const xScalar k_d,
        const xScalar max_force) {
    assert(joint_id >= 0 && joint_id < joints_.size());
    auto bullet_world = wptr_to_sptr(bullet_world_);
    joints_[joint_id]->set_motor_control_velocity(
            bullet_world->client_, body_data_.body_uid, speed, k_d, max_force);
}

void RobotBase::SetJointPosition(
        const int joint_id,
        const xScalar target,
        const xScalar k_p,
        const xScalar k_d,
        const xScalar max_force) {
    assert(joint_id >= 0 && joint_id < joints_.size());
    auto bullet_world = wptr_to_sptr(bullet_world_);
    joints_[joint_id]->set_motor_control_position(
            bullet_world->client_,
            body_data_.body_uid,
            target,
            k_d,
            k_p,
            max_force);
}

void RobotBase::ResetJointState(
        const int joint_id, const xScalar pos, const xScalar vel) {
    assert(joint_id >= -1 && joint_id < joints_.size());
    auto bullet_world = wptr_to_sptr(bullet_world_);
    joints_[joint_id]->reset_state(
            bullet_world->client_, body_data_.body_uid, pos, vel);
}

std::string RobotBase::PickUp(
        std::shared_ptr<Inventory>& inventory, 
        const glm::vec3& from,
        const glm::vec3& to) {

    assert(inventory);
    auto bullet_world = wptr_to_sptr(bullet_world_);

    std::vector<RayTestInfo> test;
    std::vector<Ray> ray;
    ray.push_back({from, to});
    bullet_world->BatchRayTest(ray, test);

    if(test[0].bullet_id >= 0) {
        auto object = bullet_world->id_to_robot_[test[0].bullet_id];
        if (object->body_data_.pickable) {
            if(inventory->PutObject(object)) {
                bullet_world->icon_inventory_.push_back(object->path());
            }
            return object->body_data_.label;
        }
    }

    return "Nothing";
}
 
bool RobotBase::occupy_test(
        std::shared_ptr<World>& world,
        std::shared_ptr<RobotBase>& item,
        const glm::vec3& c,
        xScalar& low) {

    glm::vec3 aabb_min, aabb_max;
    item->root_part_->GetAABB(aabb_min, aabb_max);
    float width = (aabb_max.x - aabb_min.x) / 2;
    float height = (aabb_max.z - aabb_min.z) / 2;
    low = (xScalar) (aabb_min.y - 20.0f);

    std::vector<RayTestInfo> ray_test;
    std::vector<Ray> ray;
    glm::vec3 o = glm::vec3(c.x, c.y + 0.05f, c.z);
    glm::vec3 d = glm::vec3(width + 0.02f, 0.0f, 0.0f);
    ray.push_back({o, o + d});
    ray.push_back({o, o - d});
    d = glm::vec3(0.0f, 0.0f, height + 0.02f);
    ray.push_back({o, o + d});
    ray.push_back({o, o - d});

    world->BatchRayTest(ray, ray_test);
    for (int i = 0; i < ray_test.size(); ++i) {
        if (ray_test[i].bullet_id >= 0) { return true; }
    }

    // TODO: Add a List for Special Objs 
    for (size_t i = 0; i < world->size(); i++) {
        auto body = world->robot_list_[i];
        auto part = body->root_part_;
        if (part &&
            !body->is_recycled() &&
            part->id() != item->body_data_.body_uid &&
            part->id() != body->body_data_.body_uid &&
            body->body_data_.label!= "Wall" &&
            body->body_data_.label!= "Floor"&&
            body->body_data_.label!= "Ceiling") {

            glm::vec3 aabb_min1, aabb_max1;
            part->GetAABB(aabb_min1, aabb_max1);
            if(aabb_min1.y >= (aabb_max.y - aabb_min.y)) continue;

            if(aabb_min1.x < aabb_max.x &&
               aabb_max1.x > aabb_min.x &&
               aabb_min1.z < aabb_max.z &&
               aabb_max1.z > aabb_min.z) {
                return true;
            }
        }
    }

    return false;
}

std::string RobotBase::PutDown(
        std::shared_ptr<Inventory>& inventory, 
        const glm::vec3& from,
        const glm::vec3& to) {
    
    auto bullet_world = wptr_to_sptr(bullet_world_);

    std::vector<RayTestInfo> ray_test;
    std::vector<Ray> ray;
    ray.push_back({from, to});
    // TODO: isn't batch test slower in this case?
    bullet_world->BatchRayTest(ray, ray_test);
    // If a ray hits
    if(!ray_test.empty() && ray_test[0].bullet_id >= 0) {
        auto object = bullet_world->id_to_robot_[ray_test[0].bullet_id];
        glm::vec3 normal = ray_test[0].norm;
        glm::vec3 pos = ray_test[0].pos;
        // Horizontal Flat Fragment
        if(object && normal[1] > 0.8f) {
            // On the Surface ???
            auto item = inventory->GetObjectLast().lock();
            // Get Object File Path
            if (item) {
                // put the item 20 units above the hit point

                xScalar p[3];
                xScalar q[4];

                p[0] = pos.x;
                p[1] = 20.0f;
                p[2] = pos.z;

                glm::vec3 pos_temp;
                glm::vec4 quat_temp;
                item->root_part_->pose(pos_temp, quat_temp);

                q[0] = quat_temp[0];
                q[1] = quat_temp[1];
                q[2] = quat_temp[2];
                q[3] = quat_temp[3];

                set_pose(bullet_world->client_,
                         item->body_data_.body_uid,
                         p,
                         q);                

                // call bullet's step to enalbe the transformation
                bullet_world->BulletStep();
                // freeze the item for now
                item->Sleep();

                xScalar low = 0.0f;
                bool occupied = occupy_test(bullet_world, item, pos, low);

                if (!occupied) {

                    p[0] = pos.x;
                    p[1] = pos.y - low + 0.01f;
                    p[2] = pos.z;

                    set_pose(bullet_world->client_,
                             item->body_data_.body_uid,
                             p,
                             q);
                    bullet_world->BulletStep();
                    bullet_world->icon_inventory_.pop_back();
                    return item->body_data_.label;
                } else {
                    inventory->PutObject(item);
                }
            }
        }
    }

    return "Nothing";
}

std::string RobotBase::Rotate(
        const glm::vec3& angle,
        const glm::vec3& from,
        const glm::vec3& to) {
    auto bullet_world = wptr_to_sptr(bullet_world_);
    std::vector<RayTestInfo> ray_test;
    std::vector<Ray> ray;
    ray.push_back({from, to});
    bullet_world->BatchRayTest(ray, ray_test);
    if (ray_test[0].bullet_id >= 0) {
        auto item = bullet_world->id_to_robot_[ray_test[0].bullet_id];
        if (item
            && item->body_data_.label != "Wall" 
            && item->body_data_.label != "Floor"
            && item->body_data_.label != "Ceiling") {
            rotate(bullet_world->client_, item->body_data_.body_uid, angle);
            bullet_world->BulletStep();

            return item->body_data_.label;
        }
    }

    return "Nothing";
}

void RobotBase::Detach() {
    if(body_data_.attach_to_id < -1) {
        printf("Nothing to detach!\n");
        return;
    }

    body_data_.attach_transform = btTransform();

    body_data_.attach_to_id = -2;
}

void RobotBase::AttachTo(std::weak_ptr<RobotBase> object) {

    auto object_sptr = object.lock();
    if (!object_sptr || 
        object_sptr->body_data_.label == "Wall" || 
        object_sptr->body_data_.label == "Floor" || 
        object_sptr->body_data_.label == "Ceiling") {
        return;
    }

    for (int i = -1; i < parts_.size(); ++i) {
        if (object_sptr->root_part_ == (i == -1 ? root_part_ : parts_[i])) {
            printf("Cannot attach itself!\n");
            return;
        }
    }

    if (body_data_.attach_to_id != -2) {
        printf("This link has been attached already! Detach first!\n");
        return;
    }

    const int id = -1;

    body_data_.attach_to_id = id;

    object_sptr->Sleep();

    // TODO
    // camera
    auto bullet_world = bullet_world_.lock();
    float pitch = glm::radians(bullet_world->camera(0)->pre_pitch_);
    glm::vec3 offset = bullet_world->camera(0)->offset_;

    auto part = (id < 0 ? root_part_ : parts_[id]);
    part->attach_object_ = object;

    attach(root_part_.get(), object_sptr->root_part_.get(), pitch, offset);
}

const RenderPart* RobotBase::render_root_ptr() const {
    return root_part_.get();
}

RenderPart* RobotBase::render_root_ptr() {
    return root_part_.get();
}

const RenderPart* RobotBase::render_part_ptr(const size_t i) const {
    assert(i >= 0 && i < parts_.size());
    return parts_[i].get();
}

RenderPart* RobotBase::render_part_ptr(const size_t i) {
    assert(i >= 0 && i < parts_.size());
    return parts_[i].get();
}

void RobotBase::attach_camera(const glm::vec3& offset,
                          const float pitch,
                          glm::vec3& loc,
                          glm::vec3& front,
                          glm::vec3& right,
                          glm::vec3& up) {

    if (!root_part_) {
        return;
    }
    BulletBody::attach_camera(
            root_part_.get(), offset, pitch, loc, front, right, up);
}

void RobotBase::hide(const bool hide) {
    if (hide == is_hiding()) { return; }
    hide_ = hide;
    if (hide_) {
        auto bullet_world = wptr_to_sptr(bullet_world_);

        xScalar pos[3] = {0, -10, 0};
        set_pose(bullet_world->client_, body_data_.body_uid, pos, (xScalar*)NULL);
        // Set Velocity to 0
        double velocity[3] = {0, 0, 0};
        set_velocity(bullet_world->client_, body_data_.body_uid, velocity);
        // Change Root to Static
        root_part_->SetStatic();
        root_part_->Sleep();
         // Change Rest to Static
        for (auto part : parts_) {
            part->SetStatic();
            part->Sleep();
        }
    } else {
        root_part_->RecoverFromStatic();
        root_part_->Wake();

        for (auto joint : joints_) {
            if (joint) {
                joint->ResetJointState(0.0f, 0.005f);
            }
        }

        for (auto part : parts_) {
            part->RecoverFromStatic();
        }
    }
}

void Robot::CalculateInverseKinematics(
        const int end_index, 
        const glm::vec3& target_pos,
        const glm::vec4& target_quat,
        xScalar* joint_damping,
        xScalar* output_joint_pos,
        int &num_poses) {
    auto bullet_world = wptr_to_sptr(bullet_world_);
    inverse_kinematics(
            bullet_world->client_,
            body_data_.body_uid,
            end_index,
            target_pos,
            target_quat,
            joint_damping,
            output_joint_pos,
            num_poses);
}

RobotWithConvertion::RobotWithConvertion(std::weak_ptr<World> bullet_world) :
        RobotBase(bullet_world),
        status_(0),
        label_("unlabeled"),
        scale_(1.0f),
        cycle_(false),
        lock_(false),
        path_(""),
        unlock_tag_(""),
        object_path_list_(0),
        object_name_list_(0) {}

void RobotWithConvertion::LoadConvertedObject(
        const std::string& filename,
        const glm::vec3& pos,
        const glm::vec4& quat, // quat
        const xScalar scale,
        const std::string& label,
        const bool concave,
        const bool fixed) {

    Json::Value json_root;
    if (!json_parse_text(filename, json_root)) {
        fprintf(stderr, "Unable to parse %s\n", filename.c_str());
        return;
    }

    assert(json_get_string(&json_root, "type") == "convert");
    bool cycle = json_get_bool(&json_root, "cycle");
    auto unlock = json_get_string(&json_root, "unlock", "");

    SetCycle(cycle);
    SetStatus(0);
    scale_ = scale;
    path_ = filename;
    label_ = label;

    if(strcmp(unlock.c_str(), "")==0) {
        lock_ = false;
    } else {
        lock_ = true;
        unlock_tag_ = std::string(unlock);
    }

    // Parse Actions
    Json::Value *json_level1, *json_level2;
    if (!json_get_object(json_level1, &json_root, "actions", Json::arrayValue)) {
        return;
    }

    for (Json::ArrayIndex index = 0; index < json_level1->size(); index++) {
        if (!json_get_array(json_level2, json_level1, index)) {
            return;
        }
        if (json_level2->type() != Json::objectValue) {
            continue;
        }

        auto action_name = json_get_string(json_level2, "name", "NoActionName");
        auto object_path = json_get_string(json_level2, "object", "NoObjectPath");
        if ((int) index == 0) {
            LoadURDFFile(
                    std::string(object_path),
                    pos,
                    quat, // quat
                    scale, 
                    std::string(label_),
                    fixed,
                    false,
                    true,
                    concave);
        }
        object_path_list_.push_back(std::string(object_path));
        object_name_list_.push_back(std::string(action_name));
    }
}

bool RobotWithConvertion::InteractWith(const std::string& tag)
{
    assert(tag.size());

    if(tag == unlock_tag_) { 
        lock_ = false; 
        printf("Actions Unlocked!\n");
        return true;
    }

    return false;
}

bool RobotWithConvertion::TakeAction(const int act_id) {
    assert(act_id > -1 && status_ > -1);
    assert(object_path_list_.size() > 0);
    assert(object_name_list_.size() > 0);

    auto bullet_world = wptr_to_sptr(bullet_world_);

    if(lock_) {
        printf("The Object Actions Locked!\n");
        return false;
    }

    if (!cycle_ && act_id <= status_) {
        printf("The Object is Not Convertable!\n");
        return false;
    }

    if (act_id == status_) {
        //printf("Convertion Ignored!\n");
        return false;
    }

    // TODO: do we need to call this here
    bullet_world->BulletStep();

    glm::vec3 pos;
    glm::vec4 quat;
    root_part_->pose(pos, quat);
    
    Remove();

    LoadURDFFile(
            object_path_list_[act_id],
            pos,
            quat,
            scale_,
            label_,
            body_data_.fixed);
    bullet_world->robot_list_.pop_back();
    status_ = act_id;    

    UpdatePickable(body_data_.pickable);
    bullet_world->AddObjectWithLabel(label_, body_data_.body_uid);
    return true;
}


void RobotWithConvertion::recycle() {
    hide(true);
    do_recycle(path_);
}

void RobotWithConvertion::Remove() {
    auto bullet_world = wptr_to_sptr(bullet_world_);
    if (root_part_) {
        root_part_.reset();
    }
    for (size_t i = 0; i < parts_.size(); ++i) {
        parts_[i].reset();
    }
    parts_.clear();

    for (size_t i = 0; i < joints_.size(); ++i) {
        joints_[i].reset();
    }
    joints_.clear();

    RemoveRobotFromBullet();

    bullet_world->RemoveObjectWithLabel(body_data_.body_uid);

    bullet_world->id_to_robot_[body_data_.body_uid] = nullptr;
}

RobotWithAnimation::RobotWithAnimation(std::weak_ptr<World> bullet_world) 
    : RobotBase(bullet_world),
      status_(0),
      joint_(1),
      lock_(false),
      unlock_tag_(""),
      positions_() {}

RobotWithAnimation::~RobotWithAnimation() {}

void RobotWithAnimation::recycle() {
    hide(true);
    do_recycle(path_);
}

bool RobotWithAnimation::InteractWith(const std::string& tag) {
    assert(tag.size());

    if(tag == unlock_tag_) { 
        lock_ = false; 
        printf("Actions Unlocked!\n");
        return true;
    }

    return false;
}

bool RobotWithAnimation::TakeAction(const int act_id) {
    assert(act_id > -1 && status_ > -1);
    assert(positions_.size() > 0);

    if(positions_.find(act_id) != positions_.end() && !lock_) {
        SetJointPosition(joint_, positions_[act_id], 1.0f, 0.1f, 200000.0f);
        return true;
    } else {
        printf("Cannot Find Action ID or Object Actions Locked!\n");
        return false;
    }
}

void RobotWithAnimation::LoadAnimatedObject(
        const std::string& filename,
        const glm::vec3& pos,
        const glm::vec4& quat,
        const xScalar scale,
        const std::string& label,
        const bool concave,
        const bool fixed) {
    
    Json::Value json_root;
    if (!json_parse_text(filename, json_root)) {
        fprintf(stderr, "Unable to parse %s\n", filename.c_str());
        return;
    }

    assert(json_get_string(&json_root, "type") == "animate");
    int joint_id = json_get_int(&json_root, "joint_id", 0);
    unlock_tag_ = json_get_string(&json_root, "unlock");
    lock_ = unlock_tag_ == "" ? false : true;
    std::string object_path =
            json_get_string(&json_root, "object", "NoObjectPath");
    LoadURDFFile(
            object_path, pos, quat, scale, label, fixed, false, true, concave);

    ignore_baking(true);
    DisableSleeping();
    SetStatus(0);
    SetJoint(joint_id);
    path_ = filename;
    object_path_ = object_path;

    // Parse Actions
    Json::Value *json_level1, *json_level2;
    if (!json_get_object(json_level1, &json_root, "actions", Json::arrayValue)) {
        return;
    }
    for (Json::ArrayIndex index = 0; index < json_level1->size(); index++) {
        if (!json_get_array(json_level2, json_level1, index)) {
            return;
        }
        if (json_level2->type() != Json::objectValue) continue;
        std::string action_name =
                json_get_string(json_level2, "name", "NoActionName");
        xScalar position = json_get_float(json_level2, "position");
        object_name_list_.push_back(action_name);
        positions_[index] = position;
    }
}

>>>>>>> fix bad pixels at left bottom corner, fix icon loading issue, fix convertible object cannot be picked up issue
World::World() : BulletWorld(),
                 id_to_robot_(),
                 recycle_robot_map_(),
                 model_cache_(),
                 object_locations_(),
                 reset_count_(0),
                 pickable_list_(),
                 tag_list_() {}

World::~World() {
    CleanEverything();
    ClearCache();
}

void World::UpdatePickableList(const std::string& tag, const bool pick) {
    pickable_list_[tag] = pick;
}

void World::AssignTag(const std::string& path, const std::string& tag) {
    tag_list_[path] = tag;
}

void World::LoadMetadata(const char* filename) {
    errno = 0;
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        fprintf(stderr, "[%d] Unable to open metadata file %s\n", errno, filename);
        return;
    }

    int line_number = 1;
    char key_buffer[4096];
    std::vector<char *> keys;
    if (fgets(key_buffer, 4096, fp)) {
        char *token = strtok(key_buffer, ",\n");
        while (token) {
            keys.push_back(token);
            token = strtok(NULL, ",\n");
        }
    }

    // Check index
    int model_id_k = -1;
    for (int i = 0; i < keys.size(); i++) {
        if (!strcmp(keys[i], "index")) {
            model_id_k = i;
            break;
        }
    }

    if (model_id_k < 0) {
        fprintf(stderr, "Did not find \"index\" in header on line %d of %s\n",
                line_number, filename);
        return;
    }

    // Check path
    int path_k = -1;
    for (int i = 0; i < keys.size(); i++) {
        if (!strcmp(keys[i], "path")) {
            path_k = i;
            break;
        }
    }

    if (path_k < 0) {
        fprintf(stderr, "Did not find \"path\" in header on line %d of %s\n",
                line_number, filename);
        return;
    }

    // Check tag
    int tag_k = -1;
    for (int i = 0; i < keys.size(); i++) {
        if (!strcmp(keys[i], "tag")) {
            tag_k = i;
            break;
        }
    }

    if (tag_k < 0) {
        fprintf(stderr, "Did not find \"tag\" in header on line %d of %s\n",
                line_number, filename);
        return;
    }

    // Check pickable
    int pickable_k = -1;
    for (int i = 0; i < keys.size(); i++) {
        if (!strcmp(keys[i], "pickable")) {
            pickable_k = i;
            break;
        }
    }

    if (pickable_k < 0) {
        fprintf(stderr, "Did not find \"pickable\" in header on line %d of %s\n",
                line_number, filename);
        return;
    }

    char value_buffer[4096];
        while (fgets(value_buffer, 4096, fp)) {
        line_number++;

        std::vector<char *> values;
        char *token = strtok(value_buffer, ",\n");
        while (token) {
            values.push_back(token);
            token = strtok(NULL, ",\n");
        }

        if (values.size() == 0) continue;
        if (values.size() != keys.size()) {
            fprintf(stderr, "Invalid number of entries at line %d in %s\n",
                    line_number, filename);
            return;
        }

        const char *model_id = values[model_id_k];
        if (!model_id) continue;
        int model_id_length = strlen(model_id);
        if (model_id_length == 0) continue;

        const char *path = values[path_k];
        if (!path) continue;
        int path_length = strlen(path);
        if (path_length == 0) continue;

        const char *tag = values[tag_k];
        if (!tag) continue;
        int tag_length = strlen(tag);
        if (tag_length == 0) continue;

        const char *pickable = values[pickable_k];
        if (!pickable) continue;
        int pickable_length = strlen(pickable);
        if (pickable_length == 0) continue;

        std::string rel_path(path);
        // std::string dir_str(filename);
        // size_t p = dir_str.find_last_of("/");
        // std::string abs_path(dir_str.substr(0, p) + std::string(path));

        bool pickable_bool = !strcmp(pickable, "0");

        pickable_list_[std::string(tag)] = pickable_bool;
        tag_list_[rel_path] = std::string(tag);        
    }

    fclose(fp);
}
std::weak_ptr<RobotBase> World::LoadRobot(
        const std::string& filename,
        const glm::vec3& pos,
        const glm::vec3& rot_axis,
        const xScalar rot_angle,
        const glm::vec3& scale,
        const std::string& label,
        const bool fixed_base,
        const xScalar mass,
        const bool flip,
        const bool concave) {

    double d = glm::length(rot_axis);
    double s = sin(rot_angle*0.5) / d;
    double c = cos(rot_angle*0.5);
    glm::vec4 quat(rot_axis[0]*s, rot_axis[1]*s, rot_axis[2]*s, c);

    return LoadRobot(filename, pos, quat, scale,
            label, fixed_base, mass, flip, concave);
}


std::weak_ptr<RobotBase> World::LoadRobot(
        const std::string& filename,
        const glm::vec3& pos,
        const glm::vec4& quat, // quat
        const glm::vec3& scale,
        const std::string& label,
        const bool fixed_base,
        const xScalar mass,
        const bool flip,
        const bool concave) {
    assert(filename.size());

    // If label is valid overide
    // Otherwise, load meta
    std::string tag(label);
    bool pickable = false;

    if(!label.length() || !label.compare("unlabeled")) {
        if(tag_list_.find(filename) != tag_list_.end()) {
            tag = tag_list_[filename];
        }
    }

    if(pickable_list_.find(tag) != pickable_list_.end()) {
        pickable = pickable_list_[tag];
    }

    int find = (int) filename.find(".obj");
    if(find > -1) {
        // Load OBJ
        std::string filename_with_scale =
                filename + ":" + 
                std::to_string(scale[0]) + ":" + 
                std::to_string(scale[1]) + ":" + 
                std::to_string(scale[2]);
        auto robot = LoadModelFromCache(filename_with_scale, pos, quat);
        if (!robot) {
            robot = std::make_shared<Robot>(shared_from_this());
            robot->LoadOBJFile(filename, pos, quat, scale,
                    tag, mass, flip, concave);
        }

        if (mass > 0) {
            robot->root_part_->SetMass(mass);
            robot->Wake();
        }

        load_icon(filename);

        robot->UpdatePickable(pickable);
        robot->reuse();
        AddObjectWithLabel(tag, robot->body_data_.body_uid);

        return robot;
    
    } 

    find = (int) filename.find(".urdf");
    if(find > -1) {
        // Load URDF
        auto robot = LoadModelFromCache(filename, pos, quat);
        if (robot) {
            robot->root_part_->RecoverFromStatic();
            robot->root_part_->Wake();
            for (auto joint : robot->joints_) {
                if(joint) {
                    joint->ResetJointState(0.0f, 0.005f);
                }
            }
            for (auto part : robot->parts_) {
                part->RecoverFromStatic();
            }
        } else {
            robot = std::make_shared<Robot>(shared_from_this());
            robot->LoadURDFFile(
                    filename, pos, quat, scale[0], tag, fixed_base, concave);
            robot->reuse();
            robot->Wake();
        }

        load_icon(filename);

        robot->UpdatePickable(pickable);
        robot->reuse();
        AddObjectWithLabel(tag, robot->body_data_.body_uid);

        return robot;
    }

    find = (int) filename.find(".json");
    if (find > -1) {
        auto robot = LoadModelFromCache(filename, pos, quat);

        if (!robot) {
            Json::Value json_root;
            if (!json_parse_text(filename, json_root)) {
                fprintf(stderr, "Unable to parse %s\n", filename.c_str());
                return std::weak_ptr<RobotBase>();
            }
            // Check Type
            std::string type = json_get_string(&json_root, "type", "NoType"); 
            if (type == "convert") {
                robot = std::make_shared<RobotWithConvertion>(
                        shared_from_this());
                auto robot_conv =
                        std::dynamic_pointer_cast<RobotWithConvertion>(robot);
                robot_conv->LoadConvertedObject(
                        filename, pos, quat, scale[0], tag, concave, fixed_base);

                load_icon(robot_conv->path());

                robot_conv->reuse();
                robot_conv->Wake();
                robot_conv->UpdatePickable(pickable); // update pickable
                AddObjectWithLabel(tag, robot_conv->body_data_.body_uid);
                return robot_conv;
            } else if (type == "animate") {
                robot = std::make_shared<RobotWithAnimation>(shared_from_this());
                auto robot_anim =
                        std::dynamic_pointer_cast<RobotWithAnimation>(robot);
                robot_anim->LoadAnimatedObject(
                        filename, pos, quat, scale[0], tag, concave, fixed_base);

                load_icon(robot_anim->path());

                robot_anim->reuse();
                robot_anim->Wake();
                robot_anim->UpdatePickable(pickable); // update pickable
                AddObjectWithLabel(tag, robot_anim->body_data_.body_uid);
                return robot_anim;
            } else {
                fprintf(stderr, "Incorrect Action File [%s]\n", type.c_str());
                return std::weak_ptr<RobotBase>();
            }
        } else {
            robot->root_part_->RecoverFromStatic();
            robot->root_part_->Wake();

            for (auto joint : robot->joints_) {
                if(joint) {
                    joint->ResetJointState(0.0f, 0.005f);
                }
            }
            for (auto part : robot->parts_) {
                part->RecoverFromStatic();
                part->Wake();
            }

            auto robot_anim =
                    std::dynamic_pointer_cast<RobotWithAnimation>(robot);
            auto robot_conv = 
                    std::dynamic_pointer_cast<RobotWithConvertion>(robot);

            if (robot_anim) {
                if(robot_anim->unlock_tag_.size()) {
                    robot_anim->SetLock(false);
                    robot_anim->TakeAction(1);
                    robot_anim->SetLock(true);
                } else {
                    robot_anim->TakeAction(1);
                }

                load_icon(robot_anim->path());

                BulletStep();
            } else if (robot_conv) {
                bool cycle = robot_conv->GetCycle();
                bool lock  = robot_conv->GetUnlockedTage().size();

                if(!cycle)
                    robot_conv->SetCycle(true);

                if(lock)
                    robot_conv->SetLock(false);

                robot_conv->TakeAction(0);

                if(!cycle)
                    robot_conv->SetCycle(false);

                if(lock)
                    robot_conv->SetLock(true);

                load_icon(robot_conv->path());

                BulletStep();
            }

            robot->reuse();
            robot->UpdatePickable(pickable);
            AddObjectWithLabel(tag, robot->body_data_.body_uid);

            return robot;
        }
    }

    // printf("file: %s [%d]\n", filename.c_str(), find);
    // printf("Format Not Support!\n");
    return std::weak_ptr<RobotBase>();
}


void World::ResetSimulation() {
    reset();
}

void World::RemoveRobot(const RobotBaseWPtr& rm_robot) {
    
    auto robot = wptr_to_sptr(rm_robot);
    int uid = robot->body_data_.body_uid;
    if (id_to_robot_.find(uid) != id_to_robot_.end()) {
        robot->root_part_->Sleep();
        for (auto& part : robot->parts_) {
            if (part) {
                part->Sleep();
            }
        }
    }
    robot->body_data_.attach_to_id = -2;
    RemoveObjectWithLabel(robot->body_data_.body_uid);
    id_to_robot_.erase(robot->body_data_.body_uid);
}

void World::CleanEverything2() {

    for (auto& kv : id_to_robot_) {
        kv.second->reset_move();
        if(!kv.second->is_recycled())
            kv.second->recycle();
    }

    for(auto &recycle_robot : recycle_robot_map_) {
        if(recycle_robot.second.size() >= kCacheSize) {
            while(recycle_robot.second.size() > kCacheSize - 4) {
                auto robot = recycle_robot.second.back();
                robot->RemoveRobotFromBullet();
                RemoveRobot(robot);
                recycle_robot.second.pop_back();
            }
        }
    }

    reset_count_++;

    remove_all_cameras();

    tag_list_.clear();
    pickable_list_.clear();
    icon_inventory_.clear();
}

void World::PrintCacheInfo() {
    for(auto& recycle_robot : recycle_robot_map_) {
        printf("urdf: %s   ", recycle_robot.first.c_str());
        printf("size: %d\n", (int) recycle_robot.second.size());
    }
}

void World::CleanEverything() {
    reset_count_++;

    remove_all_cameras();

    tag_list_.clear();
    pickable_list_.clear();
    icon_inventory_.clear();

    recycle_robot_map_.clear();
    id_to_robot_.clear();
    object_locations_.clear();
    ResetSimulation();
}

void World::UpdateAttachObjects(const RobotBaseSPtr& robot) {
    if(robot->body_data_.attach_to_id < -1) 
        return;

    if(cameras_size() < 1 || !camera(0)) 
        return;

    if (robot->body_data_.attach_to_id == -1) {
        if(auto attach_object_sptr = robot->root_part_->attach_object_.lock()) {

            bool contact = false;
            std::vector<ContactPoint> contact_points;
            GetContactPoints(
                    attach_object_sptr, 
                    attach_object_sptr->root_part_,
                    contact_points);

            for (int i = 0; i < contact_points.size(); ++i) {
                ContactPoint cp = contact_points[i];
                if(cp.contact_distance < -0.008f) {
                    robot->Detach();
                    contact = true;
                    break;
                }
            }
            // TODO
            // camera 
            if(!contact) {
                float pitch = glm::radians(camera(0)->pre_pitch_);
                glm::vec3 offset = camera(0)->offset_;

                btTransform mat_rc_r;
                btQuaternion cam_q(pitch, 0, 0);
                mat_rc_r.setIdentity();
                mat_rc_r.setRotation(cam_q);

                btTransform mat_rc_t;
                mat_rc_t.setIdentity();
                mat_rc_t.setOrigin(btVector3(0,offset.y,0));

                btTransform new_transform = 
                        robot->root_part_->object_position_;
                btTransform attach_transform = 
                        robot->body_data_.attach_transform;
                btTransform object_orn = 
                        robot->body_data_.attach_orientation;
                btTransform obj_tr = 
                        mat_rc_t * new_transform * mat_rc_r * attach_transform;

                obj_tr.setRotation(object_orn.getRotation());
                SetTransformation(attach_object_sptr, obj_tr);
            }
        } 
    }
}

void World::FixLockedObjects(const RobotBaseSPtr& robot) {
    if(auto anim_robot = std::dynamic_pointer_cast<RobotWithAnimation>(robot)) {
        if(anim_robot->GetLock()) {
            int joint = anim_robot->GetJoint();
            float position = anim_robot->GetPosition(1);
            anim_robot->SetJointPosition(joint, position, 1.0f, 0.1f, 1000000.0f);
        }
    }
}

void World::QueryPose(RobotBaseSPtr& robot) {
    const xScalar* root_inertial_frame;
    const xScalar* q;
    const xScalar* q_dot;

    robot->query_pose(client_, &root_inertial_frame, &q, &q_dot);

    robot->root_part_->object_position_ = TransformFromReals(q, q+3);
    robot->root_part_->object_speed_[0] = q_dot[0];
    robot->root_part_->object_speed_[1] = q_dot[1];
    robot->root_part_->object_speed_[2] = q_dot[2];
    robot->root_part_->object_angular_speed_[0] = q_dot[3];
    robot->root_part_->object_angular_speed_[1] = q_dot[4];
    robot->root_part_->object_angular_speed_[2] = q_dot[5];
    robot->root_part_->object_local_inertial_frame_ = 
        TransformFromReals(root_inertial_frame, root_inertial_frame+3);
    robot->root_part_->object_link_position_ = TransformFromReals(q, q+3);

    for (auto& part : robot->parts_)
    {
        struct b3LinkState link_state;
        if(!part) continue;
        if(part->bullet_link_id_ == -1) continue;

        robot->query_link(client_, part->bullet_link_id_, link_state);

        part->object_position_ = TransformFromReals(link_state.m_worldPosition,
                                                    link_state.m_worldOrientation);
        part->object_local_inertial_frame_ =
                TransformFromReals(link_state.m_localInertialPosition,
                                     link_state.m_localInertialOrientation);
        part->object_link_position_ =
                TransformFromReals(link_state.m_worldLinkFramePosition,
                                     link_state.m_worldLinkFrameOrientation);
        part->object_speed_[0] = link_state.m_worldLinearVelocity[0];
        part->object_speed_[1] = link_state.m_worldLinearVelocity[1];
        part->object_speed_[2] = link_state.m_worldLinearVelocity[2];
        part->object_angular_speed_[0] = link_state.m_worldAngularVelocity[0];
        part->object_angular_speed_[1] = link_state.m_worldAngularVelocity[1];
        part->object_angular_speed_[2] = link_state.m_worldAngularVelocity[2];
    }

    for (auto& joint : robot->joints_)
    {
        if(!joint) continue;
        joint->joint_current_position_ = q[joint->bullet_q_index_];
        joint->joint_current_speed_ = q_dot[joint->bullet_u_index_];
    }
}

void World::BulletStep(const int skip_frames)
{
    if(cameras_size() < 1 || !camera(0)) 
        return;

    // Check Object at Camera Center is Movable
    if(get_highlight_center() > -1) 
        QueryMovable();

    for (auto& kv : id_to_robot_) {
        auto robot = kv.second;
        if (!robot || robot->is_recycled() || robot->is_hiding()) 
            continue;

        // Check Object at Camera Center is Interactable
        if(get_highlight_center() == 0)
            QueryInteractable(robot);

        UpdateAttachObjects(robot);
        FixLockedObjects(robot);

        for (auto& joint : robot->joints_) {
            if(joint) {
                robot->update_joints(client_);
                break;
            }
        }
    }

    step();

    for (auto& kv : id_to_robot_) {
        if (!kv.second) continue;
        QueryPose(kv.second);
    }

    render_step();
}

void World::QueryMovable() {
    // TODO
    // Camera
    glm::vec3 from_3d = camera(0)->position_;
    glm::vec3 frnt_3d = camera(0)->front_;

    std::vector<RayTestInfo> raytest_result;
    std::vector<Ray> ray;
    ray.push_back({from_3d, from_3d + kDistanceThreshold * frnt_3d});
    BatchRayTest(ray, raytest_result);

    if(raytest_result[0].bullet_id >= 0) {
        auto object = id_to_robot_[raytest_result[0].bullet_id];

        if(object->body_data_.pickable) {
            set_highlight_center(2);
            return;
        }
    }

    set_highlight_center(0);
}

void World::QueryInteractable(const std::shared_ptr<RobotBase>& robot) {
    // TODO
    // Camera
    glm::vec3 from_3d = camera(0)->position_;
    glm::vec3 frnt_3d = camera(0)->front_;

    if (!robot->is_recycled() && !robot->is_hiding()) {

        if(std::dynamic_pointer_cast<RobotWithAnimation>(robot) || 
           std::dynamic_pointer_cast<RobotWithConvertion>(robot)) {

            glm::vec3 aabb_min, aabb_max;
            robot->root_part_->GetAABB(aabb_min, aabb_max);

            bool intersect = RayAABBIntersect({from_3d, from_3d + frnt_3d},
                                              aabb_min,
                                              aabb_max);

            glm::vec3 object_pos = (aabb_max - aabb_min) * 0.5f + aabb_min;
            float dist = glm::distance(object_pos, from_3d);

            if(intersect && dist < kDistanceThreshold) {
                set_highlight_center(1);
                return;
            }
        }
    }

    set_highlight_center(0);
}

void World::BulletInit(const float gravity, const float timestep) {
    init(gravity, timestep);    
}

void World::SetTransformation(const RobotBaseWPtr& robot, const btTransform& tr) {

    if (auto robot_sptr = robot.lock()) {
        xScalar p[3];
        p[0] = tr.getOrigin()[0];
        p[1] = tr.getOrigin()[1];
        p[2] = tr.getOrigin()[2];

        xScalar q[4];
        q[0] = tr.getRotation()[0];
        q[1] = tr.getRotation()[1];
        q[2] = tr.getRotation()[2];
        q[3] = tr.getRotation()[3];

        set_pose(client_, robot_sptr->id(), p, q);
    }
}

void World::GetRootClosestPoints(
        const RobotBaseWPtr& robot_in,
        const std::weak_ptr<Object>& part_in,
        std::vector<ContactPoint>& contact_points) {
    if (auto robot = robot_in.lock()) {
        robot->get_closest_points(client_, contact_points);
    }        
}

void World::GetContactPoints(
        const std::weak_ptr<RobotBase>& robot_in,
        const std::weak_ptr<Object>& part_in,
        std::vector<ContactPoint>& contact_points,
        const int link) {
    if(auto robot = robot_in.lock()) {
        robot->get_contact_points(client_, contact_points, link);
    } 
}

void World::BatchRayTest(
        const std::vector<Ray>& rays,
        std::vector<RayTestInfo>& result,
        const int num_threads) {
    cast_rays(rays, result, num_threads);
}

void World::RayTest(
        const glm::vec3& from, const glm::vec3& to, RayTestInfo& result) {
    cast_ray({from, to}, result);
}

std::shared_ptr<RobotBase> World::LoadModelFromCache(
        const std::string& filename,
        const glm::vec3& position,
        const glm::vec4& rotation) {

    std::shared_ptr<RobotBase> robot;

    if (recycle_robot_map_.find(filename) != recycle_robot_map_.end() && 
            !recycle_robot_map_[filename].empty()) {

        robot = recycle_robot_map_[filename].back();
        recycle_robot_map_[filename].pop_back();
        robot->reuse();

        set_pose(client_, robot->id(), position, rotation);
        set_velocity(client_, robot->id(), kFloat3Zero);
    }
    return robot;
}

void World::QueryObjectByLabel(
        const std::string& label, std::vector<ObjectAttributes>& result) {
    assert(!label.empty());

    if(object_locations_.find(label) != object_locations_.end()) {

        auto find = object_locations_[label];

        result.clear();
        for (int i = 0; i < find.size(); ++i) {
            int bullet_id = find[i];
            auto object = id_to_robot_[bullet_id];

            glm::vec3 root_aabb_min, root_aabb_max;
            object->root_part_->GetAABB(root_aabb_min, root_aabb_max);

            ObjectAttributes attribs;
            attribs.aabb_min = root_aabb_min;
            attribs.aabb_max = root_aabb_max;
            attribs.bullet_id = bullet_id;

            result.push_back(attribs);
        }
    }
}

void World::AddObjectWithLabel(const std::string& label, const int id) {
    assert(!label.empty());

    if(object_locations_.find(label) == object_locations_.end()) {
        std::vector<int> bullet_id_list;
        bullet_id_list.push_back(id);
        object_locations_[label] = bullet_id_list;
    } else {
        object_locations_[label].push_back(id);
    }
}

void World::RemoveObjectWithLabel(const int id) {
    for (auto it = object_locations_.begin(); it != object_locations_.end(); ++it) {
        if (it->second.size() > 0) {
            auto find = std::find(it->second.begin(), it->second.end(), id);
            if (find != it->second.end()) {
                it->second.erase(find);
            }
        }
    }
}

render_engine::ModelDataSPtr World::FindInCache(
        const std::string& key, 
        std::vector<render_engine::ModelDataSPtr>& model_list,
        bool& reset) {
    auto it = model_cache_.find(key);
    if (it != model_cache_.end()) {
        auto model_data = it->second;
        model_list.push_back(model_data);
        reset = false;
        if (model_data) {
            return model_data;
        }
    } else {
        auto model_data  = std::make_shared<ModelData>();
        model_list.push_back(model_data);
        model_cache_[key] = model_data;
        return model_data;
    }
    return nullptr;
}

void World::ClearCache() {
    model_cache_.clear();
}

void World::robot_iteration_begin() {
    robot_it_ = id_to_robot_.begin();
}

RenderBody* World::next_robot() {
    if (robot_it_ == id_to_robot_.end()) {
        return NULL;
    } else {
        RenderBody* ret = static_cast<RenderBody*>(robot_it_->second.get());
        robot_it_++;
        return ret;
    }
}

bool World::has_next_robot() const {
    return robot_it_ != id_to_robot_.end();
}

}
