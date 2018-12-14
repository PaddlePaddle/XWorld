#include "world.h"
#include "utils.h"

namespace xrobot {

using namespace glm;
using namespace render_engine;
using namespace bullet_engine;

Joint::Joint() : BulletJoint(),
                 bullet_world_(),
                 bullet_robot_() {}

Joint::~Joint() {}

void Joint::EnableJointSensor(const bool enable) {
    auto robot = wptr_to_sptr(bullet_robot_);
    auto world = wptr_to_sptr(bullet_world_);
    enable_sensor(world->client_, robot->id(), enable);
}

void Joint::GetJointMotorState(glm::vec3& force, glm::vec3& torque) {
    auto robot = wptr_to_sptr(bullet_robot_);
    auto world = wptr_to_sptr(bullet_world_);
    get_motor_state(world->client_, robot->id(), force, torque);
}

void Joint::ResetJointState(const xScalar pos, const xScalar vel) {
    auto robot = wptr_to_sptr(bullet_robot_);
    auto world = wptr_to_sptr(bullet_world_);
    reset_state(world->client_, robot->id(), pos, vel);
}

void Joint::SetJointMotorControlTorque(const xScalar torque) {
    auto robot = wptr_to_sptr(bullet_robot_);
    auto world = wptr_to_sptr(bullet_world_);
    set_motor_control_torque(world->client_, robot->id(), torque);
}

void Joint::SetJointMotorControlVelocity(const xScalar speed,
                                         const xScalar k_d,
                                         const xScalar max_force) {
    
    auto robot = wptr_to_sptr(bullet_robot_);
    auto world = wptr_to_sptr(bullet_world_);
    set_motor_control_velocity(
            world->client_, robot->id(), speed, k_d, max_force);
}

void Joint::SetJointMotorControlPosition(
        const xScalar target,
        const xScalar k_p,
        const xScalar k_d,
        const xScalar max_force) {
    auto robot = wptr_to_sptr(bullet_robot_);
    auto world = wptr_to_sptr(bullet_world_);
    set_motor_control_position(
            world->client_, robot->id(), target, k_p, k_d, max_force);
}

Object::Object() : RenderPart(),
                   BulletObject(),
                   bullet_world_(),
                   body_uid_(-1) {}

void Object::Sleep() {
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::sleep(world->client_, id());
}

void Object::EnableSleeping() {
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::enable_sleeping(world->client_, id());
}

void Object::DisableSleeping() {
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::disable_sleeping(world->client_, id());
}

void Object::Wake() {
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::wake(world->client_, id());
}

void Object::GetMass(xScalar& mass) {
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::get_mass(world->client_, id(), mass);
}

void Object::SetStatic() {
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::get_mass(world->client_, id(), object_mass_original_);
    BulletObject::change_mass(world->client_, id(), 0.0f);
}

void Object::RecoverFromStatic() {
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::change_mass(
            world->client_, id(), object_mass_original_);
}

void Object::GetAABB(glm::vec3& aabb_min, glm::vec3& aabb_max) {
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::get_AABB(world->client_, id(), aabb_min, aabb_max);
}

void Object::ChangeLinearDamping(const xScalar damping) {
    assert(damping >= 0);
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::change_linear_damping(world->client_, id(), damping);
}

void Object::ChangeAngularDamping(const xScalar damping) {
    assert(damping >= 0);
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::change_angular_damping(world->client_, id(), damping);
}

void Object::ChangeLateralFriction(const xScalar friction) {
    assert(friction >= 0);
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::change_lateral_friction(world->client_, id(), friction);
}

void Object::ChangeSpinningFriction(const xScalar friction) {
    assert(friction >= 0);
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::change_spinning_friction(world->client_, id(), friction);
}

void Object::ChangeRollingFriction(xScalar friction) {
    assert(friction >= 0);
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::change_rolling_friction(world->client_, id(), friction);
}

void Object::ApplyForce(
        const xScalar x, const xScalar y, const xScalar z, const int flags) {
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::apply_force(world->client_, id(), x, y, z, flags);
}

void Object::ApplyTorque(
        const xScalar x, const xScalar y, const xScalar z, const int flags) {
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::apply_torque(world->client_, id(), x, y, z, flags);
}

glm::mat4 Object::translation_matrix() const {
    return BulletObject::translation_matrix();
}

glm::mat4 Object::local_inertial_frame() const {
    return BulletObject::local_inertial_frame();
}

RobotBase::RobotBase(std::weak_ptr<World> bullet_world)
        : bullet_world_(bullet_world),
          root_part_(nullptr),
          parts_(),
          joints_() {}

void RobotBase::LoadURDFFile(
        const std::string& filename,
        const glm::vec3& pos,
        const glm::vec4& quat,
        const xScalar scale,
        const std::string& label,
        const bool fixed_base,
        const bool self_collision,
        const bool use_multibody,
        const bool concave) {

    auto bullet_world = wptr_to_sptr(bullet_world_);
        
    assert(BulletBody::load_urdf(
            bullet_world->client_,
            filename,
            pos,
            quat,
            scale,
            fixed_base,
            self_collision,
            use_multibody,
            concave));
    body_data_.label = label;

    load_robot_joints(filename);

    load_robot_shapes(scale);

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
                s,
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
    assert(load_obj(
             bullet_world->client_,
             root_part_.get(),
             filename,
             pos,
             quat,
             scale,
             mass,
             concave));
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
    body_data_.attach_to_id_ = -2;
    // Remove Label
    bullet_world->RemoveObjectWithLabel(body_data_.bullet_handle_);
    //bullet_world->id_to_robot_[key] = nullptr;

}

void RobotBase::recycle() {
    // hide
    hide(true);
    // recycle
    do_recycle(body_data_.urdf_name_);
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
                      
void RobotBase::Move(const xScalar move, const xScalar rotate) {
    assert(root_part_);
    auto bullet_world = wptr_to_sptr(bullet_world_);
    xScalar pos[3];
    xScalar quat[4];
    xScalar prev_quat[4];
    move(move, rot, root_part.get(), pos, quat, prev_quat);

    std::vector<ContactPoint> contact_points;
    bullet_world->GetRootContactPoints(
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
            xScalar delta = glm::clamp(cp.contact_distance, -0.05, 0.0);
            auto& object_a = bullet_world->id_to_robot_[cp.bullet_id_a]; 
            auto& object_b = bullet_world->id_to_robot_[cp.bullet_id_b];
            btVector3 contact_normal(to_object.x, to_object.y, to_object.z);
            pos[0] += sign * dir[0] * delta;
            pos[1] += sign * dir[1] * delta;
            pos[2] += sign * dir[2] * delta;
            set_pose(bullet_world->client, body_data_.body_uid, pos, prev_quat);
            bullet_world->BulletStep();
            break;
        }
    }
}

void RobotBase::UnFreeze() { Move(0, 0); }

void RobotBase::Freeze() { Move(0, 0); }

void RobotBase::MoveForward(const float speed) { Move(0.005*speed, 0); }

void RobotBase::MoveBackward(const float speed) { Move(-0.005*speed, 0); }

void RobotBase::TurnLeft(const float speed) { Move(0, 0.005*speed); }

void RobotBase::TurnRight(const float speed) { Move(0, -0.005*speed); }
 
void RobotBase::SetJointVelocity(
        const int joint_id,
        const float speed,
        const float k_d,
        const float max_force) {
    assert(joint_id >= 0 && joint_id < joints_.size());
    joints_[joint_id]->SetJointMotorControlVelocity(speed, k_d, max_force);
}

void RobotBase::SetJointPosition(
        const int joint_id,
        const float target,
        const float k_p,
        const float k_d,
        const float max_force) {
    assert(joint_id >= 0 && joint_id < joints_.size());
    joints_[joint_id]->SetJointMotorControlPosition(
            target, k_d, k_p, max_force);
}

void RobotBase::ResetJointState(
        const int joint_id, const float pos, const float vel) {
    assert(joint_id >= -1 && joint_id < joints_.size());
    joints_[joint_id]->ResetJointState(pos, vel);
}

void RobotBase::PickUp(
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
        auto object = bullet_world->id_to_robot_[temp_res[0].bullet_id];
        if (object->body_data_.pickable) {
            inventory->PutObject(object);
        }
    }
}
 
bool RobotBase::occupy_test(
        std::shared_ptr<World>& world,
        std::shared_ptr<RobotBase>& item,
        const glm::vec3& c) {

    glm::vec3 aabb_min, aabb_max;
    item->root_part_->GetAABB(aabb_min, aabb_max);
    float width = (aabb_max.x - aabb_min.x) / 2;
    float height = (aabb_max.z - aabb_min.z) / 2;

    std::vector<RayTestInfo> ray_test;
    std::vector<Ray> ray;
    glm::vec3 o = glm::vec3(c.x, c.y + 0.05f, c.z);
    glm::vec3 d = glm::vec3(width + 0.01f, 0.0f, 0.0f);
    ray.push_back({o, o + d});
    ray.push_back({o, o - d});
    d = glm::vec3(width + 0.01f, -0.01f, height + 0.01f);
    ray.push_back({o, o + d});
    ray.push_back({o, o - d});
    d = glm::vec3(-width - 0.01f, -0.01f, height + 0.01f)
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
            part->id() != object->body_data_.body_uid &&
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

void RobotBase::PutDown(
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
                set_pose(bullet_world->client_,
                         item->body_data_.body_uid,
                         &(glm::vec3(pos.x, pos.y + 20.0f, pos.z).x));
                // call bullet's step to enalbe the transformation
                bullet_world->BulletStep();
                // freeze the item for now
                item->Sleep();

                bool occupied = occupy_test(bullet_world, item, pos);
                if (!occupied) {
                    set_pose(bullet_world->client_,
                             item->body_data_.body_uid,
                             &(glm::vec3(pos.x, pos.y + 0.01f, pos.z).x));
                    bullet_world->BulletStep();
                } else {
                    inventory->PutObject(item);
                }
            }
        }
    }
}

void RobotBase::Rotate(
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
            rotate(bullet_world->client, item->body_data_.body_uid, angle);
            bullet_world->BulletStep();
        }
    }
}

void RobotBase::AttachTo(const int id, std::weak_ptr<RobotBase> object) {
    assert(id >= -1 && id <= parts_.size());
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

    body_data_.attach_to_id = id;

    object_sptr->Sleep();

    auto part = (id == -1 ? root_part_ : parts_[id]);
    part->attach_object_ = object;
    attach(part.get(), object_sptr->root_part_.get());
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
        set_pose(bullet_world->client_, body_data_.body_uid, pos, nullptr);
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
        path_(""),
        object_path_list_(0),
        object_name_list_(0) {}

RobotWithConvertion::~RobotWithConvertion() {}

void RobotWithConvertion::LoadConvertedObject(
        const std::string& filename,
        const glm::vec3& pos,
        const glm::vec4& quat,
        const xScalar scale,
        const std::string& label,
        const bool concave) {

    Json::Value json_root;
    if (!json_parse_text(filename, json_root)) {
        fprintf(stderr, "Unable to parse %s\n", filename.c_str());
        return;
    }
    label_ = label;
    assert(json_get_string(json_root, "type") == "convert");
    bool cycle = json_get_bool(json_root, "cycle");

    SetCycle(cycle);
    SetStatus(0);
    scale_ = scale;
    path_ = filename;

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
                    quat,
                    scale, 
                    std::string(label_),
                    false,
                    false,
                    true,
                    concave);
        }
        object_path_list_.push_back(std::string(object_path));
        object_name_list_.push_back(std::string(action_name));
    }
}

void RobotWithConvertion::TakeAction(const int act_id) {
    assert(act_id > -1 && status_ > -1);
    assert(object_path_list_.size() > 0);
    assert(object_name_list_.size() > 0);

    auto bullet_world = wptr_to_sptr(bullet_world_);

    if(!cycle_ && act_id <= status_) {
        printf("The Object is Not Convertable!\n");
        return false;
    }
    if(act_id == status_) {
        //printf("Convertion Ignored!\n");
        return false;
    }

    // TODO: do we need to call this here
    bullet_world->BulletStep();

    xScalar pos[3];
    xScalar quat[4];
    get_pose(root_part_.get(), pos, quat);
    
    Remove();

    body_data_ = BulletBodyData();
    LoadURDFFile(
            object_path_list_[act_id],
            pos, quat, scale_,
            label_ + "_" + object_name_list_[act_id], true);
    bullet_world->robot_list_.pop_back();
    status_ = act_id;    
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
        const bool concave) {
    
    Json::Value json_root;
    if (!json_parse_text(filename, json_root)) {
        fprintf(stderr, "Unable to parse %s\n", filename.c_str());
        return;
    }

    assert(json_get_string(json_root, "type") == "animate");
    int joint_id = json_get_int(json_root, "joint_id", 0);
    unlock_tag_ = json_get_string(json_root, "unlock");
    lock_ = unlock_tag_ == "" ? false : true;
    std::object_path = json_get_string(json_root, "object", "NoObjectPath");
    LoadURDFFile(object_path, pos, quat, scale, label, true, false, true, concave);

    move(true);
    DisableSleeping();
    SetStatus(0);
    SetJoint(joint_id);
    path_ = filename;

    // Parse Actions
    Json::Value *json_level1, *json_level2;
    if (!json_get_object(json_level1, &json_root, "actions", Json::arrayValue)) {
        return;
    }
    for (Json::ArrayIndex index = 0; index < json_levels->size(); index++) {
        if (!json_get_array(json_level2, json_level1, index)) {
            return;
        }
        if (json_level2->type() != Json::objectValue) continue;
        std::string action_name =
                json_get_string(json_level2, "name", "NoActionName");
        xScalar position = json_get_real(json_level2, "position");
        object_name_list_.push_back(action_name);
        positions_[index] = position;
    }
}

World::World() : robot_list_(0),
                 id_to_robot_(),
                 client_(0), 
                 recycle_robot_map_(),
                 model_cache_(),
                 object_locations_(),
                 bullet_gravity_(0),
                 bullet_timestep_(0),
                 bullet_timestep_sent_(0),
                 bullet_skip_frames_sent_(0),
                 bullet_ts_(0),
                 reset_count_(0),
                 pickable_list_(),
                 tag_list_() {}

World::~World() {
    CleanEverything();
    ClearCache();
    b3DisconnectSharedMemory(client_);
}

void World::UpdatePickableList(const std::string& tag, const bool pick)
{
    //printf("[Update Pickable List] Tag: %s\n", tag.c_str());
    pickable_list_[tag] = pick;
}

void World::AssignTag(const std::string& path, const std::string& tag)
{
    //printf("[Assign Tag] Tag: %s\n", tag.c_str());
    tag_list_[path] = tag;
}

void World::LoadMetadata(const char * filename)
{
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

        //printf("[Object Category] %s %s %s %d\n", model_id, tag, rel_path.c_str(), pickable_bool);
    }

    fclose(fp);
}

std::weak_ptr<RobotBase> World::LoadRobot(
    const std::string& filename,
    const btVector3 position,
    const btQuaternion rotation,
    const btVector3 scale,
    const std::string& label,
    const bool fixed_base,
    const float mass,
    const bool flip,
    const bool concave) 
{
    assert(filename.size());

    // If label is valid overide
    // Otherwise, load meta
    std::string tag(label);
    bool pick = false;

    if(!label.length() || !label.compare("unlabeled")) {
        if(tag_list_.find(filename) != tag_list_.end()) {
            // Invalid label but find in csv
            tag = tag_list_[filename];
            //printf("[Load Robot] Find Tag: %s\n", tag.c_str());
        }
    } else {
        //printf("[Load Robot] Load Tag: %s\n", tag.c_str());
    }

    if(pickable_list_.find(tag) != pickable_list_.end()) {
        // Invalid label but find in csv
        pick = pickable_list_[tag];
       // printf("[Load Robot] Find Pickable Object: %s\n", tag.c_str());
    }

    int find = (int) filename.find(".obj");
    if(find > -1) {
        // Load OBJ

        std::string filename_with_scale = filename + ":" + 
        std::to_string(scale[0]) + ":" + 
        std::to_string(scale[1]) + ":" + 
        std::to_string(scale[2]);

        auto robot = LoadModelFromCache(
                filename_with_scale, position, rotation);

        if(!robot) {
            robot = std::make_shared<Robot>(shared_from_this());
            robot->LoadOBJFile(filename, position, rotation, scale,
                tag, mass, flip, concave);
            robot->Wake();
        }

        robot->UpdatePickable(pick);

        robot->reuse();

        AddObjectWithLabel(tag, robot->body_data_.bullet_handle_);

        return robot;
    } 

    find = (int) filename.find(".urdf");
    if(find > -1) {
        // Load URDF
        std::shared_ptr<RobotBase> robot = LoadModelFromCache(
            filename, position, rotation);

        if(robot) {
            robot->body_data_.root_part_->RecoverFromStatic();
            robot->body_data_.root_part_->Wake();

            for (auto joint : robot->body_data_.joints_list_) {
                if(joint) {
                    joint->ResetJointState(0.0f, 0.005f);
                }
            }

            for (auto part : robot->body_data_.other_parts_) {
                part->RecoverFromStatic();
            }
        } else {
            robot = std::make_shared<Robot>(shared_from_this());
            robot->LoadURDFFile(filename, position, rotation, scale[0],
                tag, fixed_base, concave);
            robot->reuse();
            robot->Wake();
        }

        robot->UpdatePickable(pick);

        robot->reuse();

        AddObjectWithLabel(tag, robot->body_data_.bullet_handle_);

        return robot;
    }

    find = (int) filename.find(".json");
    if(find > -1) {
        std::shared_ptr<RobotBase> robot = LoadModelFromCache(
            filename, position, rotation);

        if(!robot) {
            FILE* fp = fopen(filename.c_str(), "rb");
            if (!fp) {
                fprintf(stderr, "Unable to open action file %s\n", filename.c_str());
                return std::weak_ptr<RobotBase>();
            }

            std::string text;
            fseek(fp, 0, SEEK_END);
            long const size = ftell(fp);
            fseek(fp, 0, SEEK_SET);
            char* buffer = new char[size + 1];
            unsigned long const usize = static_cast<unsigned long const>(size);
            if (fread(buffer, 1, usize, fp) != usize) { 
                fprintf(stderr, "Unable to read %s\n", filename.c_str()); 
                return std::weak_ptr<RobotBase>();
            }
            else { buffer[size] = 0; text = buffer; }
            delete[] buffer;
            fclose(fp);

            // Digest file
            Json::Value json_root;
            Json::Reader json_reader;
            Json::Value *json_items, *json_item, *json_value;
            if (!json_reader.parse(text, json_root, false)) {
                fprintf(stderr, "Unable to parse %s\n", filename.c_str());
                return std::weak_ptr<RobotBase>();
            }

            // Get Label
            char action_label[1024];
            strncpy(action_label, "NoLabel", 1024);
            if (GetJsonObjectMember(json_value, &json_root, "label", Json::stringValue)) {
                strncpy(action_label, json_value->asString().c_str(), 1024);
            }

            // Check Type
            char type[1024];
            strncpy(type, "NoType", 1024);
            if (GetJsonObjectMember(json_value, &json_root, "type", Json::stringValue)) {
                strncpy(type, json_value->asString().c_str(), 1024);
                if (strcmp(type, "convert")== 0) {
                    robot = std::make_shared<RobotWithConvertion>(shared_from_this());
                    robot->LoadConvertedObject(filename, position, rotation, scale[0], tag, concave);
                    robot->reuse();
                    robot->Wake();
                } else if (strcmp(type, "animate")== 0) {
                    robot = std::make_shared<RobotWithAnimation>(shared_from_this());
                    robot->LoadAnimatedObject(filename, position, rotation, scale[0], tag, concave);
                    robot->reuse();
                    robot->Wake();
                } else {
                    fprintf(stderr, "Incorrect Action File [%s]\n", type);
                    return std::weak_ptr<RobotBase>();
                }
            }
        } else {
            robot->body_data_.root_part_->RecoverFromStatic();
            robot->body_data_.root_part_->Wake();

            for (auto joint : robot->body_data_.joints_list_) {
                if(joint) {
                    joint->ResetJointState(0.0f, 0.005f);
                }
            }

            for (auto part : robot->body_data_.other_parts_) {
                part->RecoverFromStatic();
                part->Wake();
            }

           if(auto robot_anim = std::dynamic_pointer_cast<RobotWithAnimation>(robot))
           {
                if(robot_anim->unlock_tag_.size()) {
                    robot_anim->SetLock(false);
                    robot_anim->TakeAction(1);
                    robot_anim->SetLock(true);
                } else {
                    robot_anim->TakeAction(1);
                }

           } else {
                robot_anim->TakeAction(0);
           }

            BulletStep();
        }

        robot->reuse();

        robot->UpdatePickable(pick);

        AddObjectWithLabel(tag, robot->body_data_.bullet_handle_);

        return robot;
    }

    printf("Format Not Support!\n");
    return std::weak_ptr<RobotBase>();
}


void World::ResetSimulation() 
{
    b3SubmitClientCommandAndWaitStatus(
        client_, b3InitResetSimulationCommand(client_));
}

void World::RemoveRobot(std::weak_ptr<RobotBase> rm_robot) 
{
    
    if(auto robot = rm_robot.lock()) {

        auto index = std::find(robot_list_.begin(), robot_list_.end(), robot);
        if(index != robot_list_.end())
        {
            int i = index - robot_list_.begin();
            auto robot = robot_list_[i];

            robot->body_data_.root_part_->Sleep();
            for (int i = 0; i < robot->body_data_.other_parts_.size(); ++i)
            {
                if(robot->body_data_.other_parts_[i])
                    robot->body_data_.other_parts_[i]->Sleep();
            }

            //delete robot_list_[i];
            robot_list_[i] = nullptr;
            robot_list_.erase(index);        
        }

        // ???
        //robot->RemoveRobotFromBullet();

        robot->body_data_.attach_to_id_ = -2;

        RemoveObjectWithLabel(robot->body_data_.bullet_handle_);

        id_to_robot_[robot->body_data_.bullet_handle_] = nullptr;
    }
}

void World::CleanEverything2()
{
    // PrintCacheInfo();

    for (auto robot : robot_list_)
    {
        if(!robot->is_recycled())
            robot->recycle();
    }

    for(auto &recycle_robot : recycle_robot_map_)
    {
        if(recycle_robot.second.size() >= kCacheSize)
        {
            while(recycle_robot.second.size() > kCacheSize - 4)
            {
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
}

void World::PrintCacheInfo()
{
    printf("\n==============Cache=============\n");
    for(auto& recycle_robot : recycle_robot_map_)
    {
        printf("urdf: %s   ", recycle_robot.first.c_str());
        printf("size: %d\n", (int) recycle_robot.second.size());
    }
    printf("================================\n");
}

void World::CleanEverything()
{

    for (unsigned int i = 0; i < robot_list_.size(); ++i)
    {
        if(robot_list_[i]){
            robot_list_[i] = nullptr;
        }
    }

    reset_count_++;

    remove_all_cameras();

    tag_list_.clear();
    pickable_list_.clear();

    recycle_robot_map_.clear();
    robot_list_.clear();
    id_to_robot_.clear();
    object_locations_.clear();
    ResetSimulation();
}

void World::BulletStep(const int skip_frames)
{
    float need_timestep = bullet_timestep_ * skip_frames;
    if (bullet_timestep_sent_ != need_timestep ||
        bullet_skip_frames_sent_ != skip_frames || true) {
        CommandHandle cmd_handle = b3InitPhysicsParamCommand(client_);
        b3PhysicsParamSetGravity(cmd_handle, 0, bullet_gravity_, 0);
        b3PhysicsParamSetDefaultContactERP(cmd_handle, 0.2);
        b3PhysicsParamSetTimeStep(cmd_handle, 0.005); // bullet_timestep_
        bullet_timestep_sent_ = need_timestep;
        bullet_skip_frames_sent_ = skip_frames;
        b3SubmitClientCommandAndWaitStatus(client_, cmd_handle);
    }

    for (auto robot : robot_list_)
    {
        if (!robot) continue;

        // Attach
        if(robot->body_data_.attach_to_id_ > -2)
        {
            if(robot->body_data_.attach_to_id_ == -1)
            {
                if(auto attach_object_sptr = robot->body_data_.root_part_->attach_object_.lock())
                {
                    btTransform new_transform = robot->body_data_.root_part_->object_position_;

                    SetTransformation(attach_object_sptr,
                        new_transform * robot->body_data_.root_part_->attach_transform_);
                } 
            }
            else if(auto attach_object_sptr = 
                robot->body_data_.other_parts_[robot->body_data_.attach_to_id_]->attach_object_.lock())
            {
                btTransform new_transform = 
                    robot->body_data_.other_parts_[robot->body_data_.attach_to_id_]->object_position_;

                SetTransformation(attach_object_sptr,
                    new_transform * robot->body_data_.other_parts_[robot->body_data_.attach_to_id_]->attach_transform_);
            } 
        }

        // Lock
        if(auto anim_robot = std::dynamic_pointer_cast<RobotWithAnimation>(robot)) {
            if(anim_robot->GetLock()) {
                int joint = anim_robot->GetJoint();
                float position = anim_robot->GetPosition(1);
                anim_robot->SetJointPosition(joint, position, 1.0f, 0.1f, 100000000.0f);
            }
        }

        CommandHandle cmd_handle = 0;
        
        for (auto joint : robot->body_data_.joints_list_)
        {
            if(!joint) continue;
            if(!cmd_handle) 
                cmd_handle = b3JointControlCommandInit2(client_, 
                        robot->body_data_.bullet_handle_, kVelocity);
        }
        if (cmd_handle)
            b3SubmitClientCommandAndWaitStatus(client_, cmd_handle);
    }

    CommandHandle cmd_handle = b3InitStepSimulationCommand(client_);
    b3SubmitClientCommandAndWaitStatus(client_, cmd_handle);

    bullet_ts_ += bullet_timestep_ * skip_frames;
    QueryPositions();

    render_step();
}

void World::QueryPositions()
{
    for (auto robot : robot_list_)
    {
        if (!robot) continue;
        QueryPosition(robot);
    }
}

void World::QueryPosition(std::shared_ptr<RobotBase> robot)
{
    if(!robot->body_data_.root_part_) return;

    CommandHandle cmd_handle =
            b3RequestActualStateCommandInit(client_, robot->body_data_.bullet_handle_);
    b3RequestActualStateCommandComputeLinkVelocity(cmd_handle, kComputeVelocity);
    StatusHandle status_handle = b3SubmitClientCommandAndWaitStatus(client_, cmd_handle);

    const double* root_inertial_frame;
    const double* q;
    const double* q_dot;
    
    b3GetStatusActualState(
        status_handle,
        0,
        0,
        0,
        &root_inertial_frame,
        &q,
        &q_dot,
        0
    );

    assert(robot->body_data_.root_part_->bullet_link_id_==-1);
    robot->body_data_.root_part_->object_position_ = TransformFromReals(q, q+3);
    robot->body_data_.root_part_->object_speed_[0] = q_dot[0];
    robot->body_data_.root_part_->object_speed_[1] = q_dot[1];
    robot->body_data_.root_part_->object_speed_[2] = q_dot[2];
    robot->body_data_.root_part_->object_angular_speed_[0] = q_dot[3];
    robot->body_data_.root_part_->object_angular_speed_[1] = q_dot[4];
    robot->body_data_.root_part_->object_angular_speed_[2] = q_dot[5];
    robot->body_data_.root_part_->object_local_inertial_frame_ = TransformFromReals(root_inertial_frame, root_inertial_frame+3);
    robot->body_data_.root_part_->object_link_position_ = TransformFromReals(q, q+3);

    int status_type = b3GetStatusType(status_handle);
    if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED){
        printf("Query Position Failed!\n");
        return;
    }

    for (auto part : robot->body_data_.other_parts_)
    {
        struct b3LinkState link_state;
        if(!part) continue;
        if(part->bullet_link_id_ == -1) continue;

        b3GetLinkState(client_, status_handle, part->bullet_link_id_, &link_state);
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

    for (auto joint : robot->body_data_.joints_list_)
    {
        if(!joint) continue;
        joint->joint_current_position_ = q[joint->bullet_q_index_];
        joint->joint_current_speed_ = q_dot[joint->bullet_u_index_];
    }
}

void World::BulletInit(const float gravity, const float timestep)
{
    client_ = b3ConnectPhysicsDirect();
    bullet_gravity_ = gravity;
    bullet_timestep_ = timestep;

    CommandHandle cmd_handle = b3InitPhysicsParamCommand(client_);
    b3PhysicsParamSetEnableFileCaching(cmd_handle, 1);
    b3SubmitClientCommandAndWaitStatus(client_, cmd_handle);
}

void World::GetRootClosestPoints(
    std::weak_ptr<RobotBase> robot_in,
    std::weak_ptr<Object> part_in,
    std::vector<ContactPoint>& contact_points)
{
    if(auto robot = robot_in.lock()) 
    {
        auto part  = part_in.lock();

        int bodyUniqueIdA = robot->body_data_.root_part_->id();
        int linkIndexA = part->bullet_link_id_;

        struct b3ContactInformation contact_point_data;

        CommandHandle cmd_handle = b3InitClosestDistanceQuery(client_);

        b3SetClosestDistanceFilterBodyA(cmd_handle, bodyUniqueIdA);
        b3SetClosestDistanceThreshold(cmd_handle, 0.f);

        StatusHandle status_handle = b3SubmitClientCommandAndWaitStatus(client_, cmd_handle);
        int status_type = b3GetStatusType(status_handle);
        if (status_type == CMD_CONTACT_POINT_INFORMATION_COMPLETED)
        {
            b3GetContactPointInformation(client_, &contact_point_data);

            contact_points.clear();

            for (int i = 0; i < contact_point_data.m_numContactPoints; ++i)
            {
                ContactPoint point;

                float normal_x, normal_y, normal_z;
                normal_x = contact_point_data.m_contactPointData[i].m_contactNormalOnBInWS[0];
                normal_y = contact_point_data.m_contactPointData[i].m_contactNormalOnBInWS[1];
                normal_z = contact_point_data.m_contactPointData[i].m_contactNormalOnBInWS[2];

                float pos_a_x, pos_a_y, pos_a_z;
                pos_a_x = contact_point_data.m_contactPointData[i].m_positionOnAInWS[0];
                pos_a_y = contact_point_data.m_contactPointData[i].m_positionOnAInWS[1];
                pos_a_z = contact_point_data.m_contactPointData[i].m_positionOnAInWS[2];

                float pos_b_x, pos_b_y, pos_b_z;
                pos_b_x = contact_point_data.m_contactPointData[i].m_positionOnBInWS[0];
                pos_b_y = contact_point_data.m_contactPointData[i].m_positionOnBInWS[1];
                pos_b_z = contact_point_data.m_contactPointData[i].m_positionOnBInWS[2];

                point.contact_normal = glm::vec3(normal_x, normal_y, normal_z);
                point.contact_force = contact_point_data.m_contactPointData[i].m_normalForce;
                point.contact_distance = contact_point_data.m_contactPointData[i].m_contactDistance;
                point.bullet_id_a = contact_point_data.m_contactPointData[i].m_bodyUniqueIdA;
                point.bullet_id_b = contact_point_data.m_contactPointData[i].m_bodyUniqueIdB;
                point.contact_position_a = glm::vec3(pos_a_x, pos_a_y, pos_a_z);
                point.contact_position_b = glm::vec3(pos_b_x, pos_b_y, pos_b_z);
                contact_points.push_back(point);
            }
            //printf("contact: %d\n", contact_point_data.m_numContactPoints);
        }
    }
}

void World::GetRootContactPoints(
    std::weak_ptr<RobotBase> robot_in,
    std::weak_ptr<Object> part_in,
    std::vector<ContactPoint>& contact_points)
{
    if(auto robot = robot_in.lock()) 
    {
        auto part  = part_in.lock();

        int bodyUniqueIdA = robot->body_data_.root_part_->id();
        int linkIndexA = part->bullet_link_id_;

        struct b3ContactInformation contact_point_data;

        CommandHandle cmd_handle = b3InitRequestContactPointInformation(client_);

        b3SetContactFilterBodyA(cmd_handle, bodyUniqueIdA);
        // b3SetContactFilterLinkA(cmd_handle, -1);

        StatusHandle status_handle = b3SubmitClientCommandAndWaitStatus(client_, cmd_handle);
        int status_type = b3GetStatusType(status_handle);
        if (status_type == CMD_CONTACT_POINT_INFORMATION_COMPLETED)
        {
            b3GetContactPointInformation(client_, &contact_point_data);

            contact_points.clear();

            for (int i = 0; i < contact_point_data.m_numContactPoints; ++i)
            {
                ContactPoint point;

                float normal_x, normal_y, normal_z;
                normal_x = contact_point_data.m_contactPointData[i].m_contactNormalOnBInWS[0];
                normal_y = contact_point_data.m_contactPointData[i].m_contactNormalOnBInWS[1];
                normal_z = contact_point_data.m_contactPointData[i].m_contactNormalOnBInWS[2];

                float pos_a_x, pos_a_y, pos_a_z;
                pos_a_x = contact_point_data.m_contactPointData[i].m_positionOnAInWS[0];
                pos_a_y = contact_point_data.m_contactPointData[i].m_positionOnAInWS[1];
                pos_a_z = contact_point_data.m_contactPointData[i].m_positionOnAInWS[2];

                float pos_b_x, pos_b_y, pos_b_z;
                pos_b_x = contact_point_data.m_contactPointData[i].m_positionOnBInWS[0];
                pos_b_y = contact_point_data.m_contactPointData[i].m_positionOnBInWS[1];
                pos_b_z = contact_point_data.m_contactPointData[i].m_positionOnBInWS[2];

                point.contact_normal = glm::vec3(normal_x, normal_y, normal_z);
                point.contact_force = contact_point_data.m_contactPointData[i].m_normalForce;
                point.contact_distance = contact_point_data.m_contactPointData[i].m_contactDistance;
                point.bullet_id_a = contact_point_data.m_contactPointData[i].m_bodyUniqueIdA;
                point.bullet_id_b = contact_point_data.m_contactPointData[i].m_bodyUniqueIdB;
                point.contact_position_a = glm::vec3(pos_a_x, pos_a_y, pos_a_z);
                point.contact_position_b = glm::vec3(pos_b_x, pos_b_y, pos_b_z);
                contact_points.push_back(point);
            }
            //printf("contact: %d\n", contact_point_data.m_numContactPoints);
        }
    }
}

void World::CharacterMove(std::weak_ptr<RobotBase> robot_move, 
    const glm::vec3 walk_move, const glm::vec3 walk_rotate, const float speed)
{

    if(auto robot = robot_move.lock()) 
    {
        btTransform robot_transform = robot->body_data_.root_part_->object_position_;
        btVector3 robot_position = robot_transform.getOrigin();
        btQuaternion robot_rotation = robot_transform.getRotation();

        glm::vec3 walk_dir_temp = glm::normalize(walk_move);
        btVector3 walk_dir = btVector3(walk_dir_temp.x, walk_dir_temp.y, walk_dir_temp.z);
        walk_dir = btTransform(robot_rotation) * walk_dir;

        btVector3 walk_rot_temp = btVector3(walk_rotate.x, walk_rotate.y, walk_rotate.z);
        btQuaternion walk_rot = btQuaternion(walk_rot_temp, speed);

        // std::vector<ContactPoint> contact_points;
        // GetRootContactPoints(robot, contact_points);
        // int contact_ground_flag = 0;

        glm::vec3 fromPosition = glm::vec3(robot_position[0], robot_position[1], robot_position[2]) + glm::vec3(0, 0.15f, 0);
        glm::vec3 toPosition = fromPosition - glm::vec3(0, 0.2f, 0);
        int res = RayTest(fromPosition, toPosition);

        if(glm::dot(walk_move, glm::vec3(1)) > 0.001f) {
            robot_position += walk_dir * speed;
        }
        if(glm::dot(walk_rotate, glm::vec3(1)) > 0.001f) {
            robot_rotation *= walk_rot;
        }

        if(res < 0) {
            robot_position += btVector3(0,-9.81,0) * 0.001f;
        }
        //printf("Contact: %d\n", res);

        robot_transform.setOrigin(robot_position);
        robot_transform.setRotation(robot_rotation);
        SetTransformation(robot, robot_transform);
    }
} 

void World::ChangeFixedRootToTargetConstraint(
    const int constraint_id,
    const btVector3& child_relative_position,
    const btQuaternion& child_relative_orientation,
    const float max_force)
{
    CommandHandle cmd_handle = b3InitChangeUserConstraintCommand(client_, constraint_id);

    double joint_child_pivot[3];
    joint_child_pivot[0] = child_relative_position[0];
    joint_child_pivot[1] = child_relative_position[1];
    joint_child_pivot[2] = child_relative_position[2];

    double joint_child_frame_orientation[4];
    joint_child_frame_orientation[0] = child_relative_orientation[0];
    joint_child_frame_orientation[1] = child_relative_orientation[1];
    joint_child_frame_orientation[2] = child_relative_orientation[2];
    joint_child_frame_orientation[3] = child_relative_orientation[3];

    b3InitChangeUserConstraintSetPivotInB(cmd_handle, joint_child_pivot);
    b3InitChangeUserConstraintSetFrameInB(cmd_handle, joint_child_frame_orientation);
    b3InitChangeUserConstraintSetMaxForce(cmd_handle, max_force);
    b3SubmitClientCommandAndWaitStatus(client_, cmd_handle);
}

int World::CreateFixedRootToTargetConstraint(
    std::weak_ptr<RobotBase> parent_robot,
    const btVector3& parent_relative_position,
    const btVector3& child_relative_position,
    const btQuaternion& parent_relative_orientation,
    const btQuaternion& child_relative_orientation)
{

    if(auto parent = parent_robot.lock())
    {
        struct b3JointInfo joint_info;
        joint_info.m_jointType = kFixed;
        joint_info.m_parentFrame[0] = parent_relative_position[0];
        joint_info.m_parentFrame[1] = parent_relative_position[1];
        joint_info.m_parentFrame[2] = parent_relative_position[2];
        joint_info.m_parentFrame[3] = parent_relative_orientation[0];
        joint_info.m_parentFrame[4] = parent_relative_orientation[1];
        joint_info.m_parentFrame[5] = parent_relative_orientation[2];
        joint_info.m_parentFrame[6] = parent_relative_orientation[3];
        joint_info.m_childFrame[0] = child_relative_position[0];
        joint_info.m_childFrame[1] = child_relative_position[1];
        joint_info.m_childFrame[2] = child_relative_position[2];
        joint_info.m_childFrame[3] = child_relative_orientation[0];
        joint_info.m_childFrame[4] = child_relative_orientation[1];
        joint_info.m_childFrame[5] = child_relative_orientation[2];
        joint_info.m_childFrame[6] = child_relative_orientation[3];
        joint_info.m_jointAxis[0] = 0;
        joint_info.m_jointAxis[1] = 0;
        joint_info.m_jointAxis[2] = 0;


        StatusHandle status_handle = b3SubmitClientCommandAndWaitStatus(
            client_, 
            b3InitCreateUserConstraintCommand(
                client_,
                parent->body_data_.bullet_handle_,
                -1, 
                -1, 
                -1, 
                &joint_info
            )
        );
        int status_type = b3GetStatusType(status_handle);
        if (status_type == CMD_USER_CONSTRAINT_COMPLETED)
        {
            int user_constraint_id = b3GetStatusUserConstraintUniqueId(status_handle);
            return user_constraint_id;
        }
    }
    return -1;
}

void World::SetVelocity(
    std::weak_ptr<RobotBase> robot_vel,
    const btVector3& velocity)
{

    if(auto robot = robot_vel.lock())
    {
        CommandHandle cmd_handle = b3CreatePoseCommandInit(client_,
                robot->body_data_.bullet_handle_);

        double velocity_temp[3];
        velocity_temp[0] = velocity[0];
        velocity_temp[1] = velocity[1];
        velocity_temp[2] = velocity[2];
        b3CreatePoseCommandSetBaseLinearVelocity(cmd_handle, velocity_temp);
        b3SubmitClientCommandAndWaitStatus(client_, cmd_handle);
    }
}

void World::SetTransformation(
    std::weak_ptr<RobotBase> robot_tr,
    const btTransform& tranform)
{
    if(auto robot = robot_tr.lock())
    {

        CommandHandle cmd_handle = b3CreatePoseCommandInit(client_,
                robot->body_data_.bullet_handle_);

        b3CreatePoseCommandSetBasePosition(
            cmd_handle,
            tranform.getOrigin()[0],
            tranform.getOrigin()[1],
            tranform.getOrigin()[2]
        );
        
        b3CreatePoseCommandSetBaseOrientation(
            cmd_handle,
            tranform.getRotation()[0],
            tranform.getRotation()[1],
            tranform.getRotation()[2],
            tranform.getRotation()[3]
        );

        b3SubmitClientCommandAndWaitStatus(client_, cmd_handle);
    }
}

void World::BatchRayTest(const std::vector<Ray> rays, std::vector<RayTestInfo>& result,
    const int num_threads)
{
    CommandHandle cmd_handle = b3CreateRaycastBatchCommandInit(client_);
    b3RaycastBatchSetNumThreads(cmd_handle, num_threads);

    double ray_from_position_temp[rays.size() * 3];
    double ray_to_position_temp[rays.size() * 3];
    for (int i = 0; i < rays.size(); ++i)
    {
        ray_from_position_temp[3 * i + 0] = rays[i].from.x;
        ray_from_position_temp[3 * i + 1] = rays[i].from.y;
        ray_from_position_temp[3 * i + 2] = rays[i].from.z;

        ray_to_position_temp[3 * i + 0] = rays[i].to.x;
        ray_to_position_temp[3 * i + 1] = rays[i].to.y;
        ray_to_position_temp[3 * i + 2] = rays[i].to.z;
    }

    b3RaycastBatchAddRays(client_, cmd_handle, ray_from_position_temp, ray_to_position_temp, rays.size());

    StatusHandle status_handle = b3SubmitClientCommandAndWaitStatus(client_, cmd_handle);
    if (b3GetStatusType(status_handle) == CMD_REQUEST_RAY_CAST_INTERSECTIONS_COMPLETED)
    {
        struct b3RaycastInformation raycast_info;
        b3GetRaycastInformation(client_, &raycast_info);

        for (int i = 0; i < raycast_info.m_numRayHits; ++i)
        {
            RayTestInfo res;
            res.bullet_id = raycast_info.m_rayHits[i].m_hitObjectUniqueId;

            glm::vec3 pos_temp;
            pos_temp[0] = raycast_info.m_rayHits[i].m_hitPositionWorld[0];
            pos_temp[1] = raycast_info.m_rayHits[i].m_hitPositionWorld[1];
            pos_temp[2] = raycast_info.m_rayHits[i].m_hitPositionWorld[2];
            res.pos = pos_temp;

            glm::vec3 norm_temp;
            norm_temp[0] = raycast_info.m_rayHits[i].m_hitNormalWorld[0];
            norm_temp[1] = raycast_info.m_rayHits[i].m_hitNormalWorld[1];
            norm_temp[2] = raycast_info.m_rayHits[i].m_hitNormalWorld[2];
            res.norm = norm_temp;
            
            result.push_back(res);
        } 
    }
    return;
}

int World::RayTest(const vec3 ray_from_position, const vec3 ray_to_position)
{
    CommandHandle cmd_handle = b3CreateRaycastBatchCommandInit(client_);
    b3RaycastBatchSetNumThreads(cmd_handle, 2);

    double ray_from_position_temp[3];
    ray_from_position_temp[0] = ray_from_position.x;
    ray_from_position_temp[1] = ray_from_position.y;
    ray_from_position_temp[2] = ray_from_position.z;

    double ray_to_position_temp[3];
    ray_to_position_temp[0] = ray_to_position.x;
    ray_to_position_temp[1] = ray_to_position.y;
    ray_to_position_temp[2] = ray_to_position.z;

    b3RaycastBatchAddRays(client_, cmd_handle, ray_from_position_temp, ray_to_position_temp, 1);

    StatusHandle status_handle = b3SubmitClientCommandAndWaitStatus(client_, cmd_handle);
    if (b3GetStatusType(status_handle) == CMD_REQUEST_RAY_CAST_INTERSECTIONS_COMPLETED)
    {
        struct b3RaycastInformation raycast_info;
        b3GetRaycastInformation(client_, &raycast_info);
        return raycast_info.m_rayHits[0].m_hitObjectUniqueId;
    }
    return -1;
}

std::shared_ptr<RobotBase> World::LoadModelFromCache(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation) {

    std::shared_ptr<RobotBase> robot;

    if (recycle_robot_map_.find(filename) != recycle_robot_map_.end() && 
            !recycle_robot_map_[filename].empty()) {

        robot = recycle_robot_map_[filename].back();
        recycle_robot_map_[filename].pop_back();
        robot->reuse();

        btTransform transform;
        transform.setIdentity();
        transform.setOrigin(position);
        transform.setRotation(rotation);

        SetTransformation(robot, transform);
        SetVelocity(robot, btVector3(0,0,0));
    }
    return robot;
}

void World::QueryObjectDirectionByLabel(const std::string& label, const glm::vec3 front,
        const glm::vec3 eye, std::vector<ObjectDirections>& result) {
    assert(!label.empty());

    if(object_locations_.find(label) != object_locations_.end()) {

        auto find = object_locations_[label];

        result.clear();

        for (int i = 0; i < find.size(); ++i)
        {
            int bullet_id = find[i];
            auto object = id_to_robot_[bullet_id];

            glm::vec3 root_aabb_min, root_aabb_max;
            object->body_data_.root_part_->GetAABB(root_aabb_min, root_aabb_max);

            ObjectDirections directions;

            glm::vec3 bbox_positions[9];
            bbox_positions[0] = (root_aabb_min + root_aabb_max) * 0.5f;
            bbox_positions[1] = glm::vec3(root_aabb_min.x, root_aabb_min.y, root_aabb_min.z);
            bbox_positions[2] = glm::vec3(root_aabb_min.x, root_aabb_min.y, root_aabb_max.z);
            bbox_positions[3] = glm::vec3(root_aabb_min.x, root_aabb_max.y, root_aabb_min.z);
            bbox_positions[4] = glm::vec3(root_aabb_min.x, root_aabb_max.y, root_aabb_max.z);
            bbox_positions[5] = glm::vec3(root_aabb_max.x, root_aabb_max.y, root_aabb_min.z);
            bbox_positions[6] = glm::vec3(root_aabb_max.x, root_aabb_max.y, root_aabb_max.z);
            bbox_positions[7] = glm::vec3(root_aabb_max.x, root_aabb_min.y, root_aabb_max.z);
            bbox_positions[8] = glm::vec3(root_aabb_max.x, root_aabb_max.y, root_aabb_max.z);

            directions.dirs[0] = glm::angle(glm::normalize(bbox_positions[0] - eye), front);
            directions.dirs[1] = glm::angle(glm::normalize(bbox_positions[1] - eye), front);
            directions.dirs[2] = glm::angle(glm::normalize(bbox_positions[2] - eye), front);
            directions.dirs[3] = glm::angle(glm::normalize(bbox_positions[3] - eye), front);
            directions.dirs[4] = glm::angle(glm::normalize(bbox_positions[4] - eye), front);
            directions.dirs[5] = glm::angle(glm::normalize(bbox_positions[5] - eye), front);
            directions.dirs[6] = glm::angle(glm::normalize(bbox_positions[6] - eye), front);
            directions.dirs[7] = glm::angle(glm::normalize(bbox_positions[7] - eye), front);
            directions.dirs[8] = glm::angle(glm::normalize(bbox_positions[8] - eye), front);
            directions.bullet_id = bullet_id;

            result.push_back(directions);
        }
    }
}

void World::QueryObjectByLabel(const std::string& label,
        std::vector<ObjectAttributes>& result) {
    assert(!label.empty());

    if(object_locations_.find(label) != object_locations_.end()) {

        auto find = object_locations_[label];

        result.clear();

        for (int i = 0; i < find.size(); ++i)
        {
            int bullet_id = find[i];
            auto object = id_to_robot_[bullet_id];

            glm::vec3 root_aabb_min, root_aabb_max;
            object->body_data_.root_part_->GetAABB(root_aabb_min, root_aabb_max);

            ObjectAttributes attribs;
            attribs.aabb_min = root_aabb_min;
            attribs.aabb_max = root_aabb_max;
            attribs.bullet_id = bullet_id;

            result.push_back(attribs);
        }
    }
}

void World::AddObjectWithLabel(const std::string& label,
        const int id) {
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
    for (auto it = object_locations_.begin(); it != object_locations_.end(); ++it)
    {
        if(it->second.size() > 0) {
            auto find = std::find(it->second.begin(), it->second.end(), id);
            if(find != it->second.end()) {
                it->second.erase(find);
            }
        }
    }
}

render_engine::ModelDataSPtr World::FindInCache(
        const std::string &key, 
        std::vector<render_engine::ModelDataSPtr> &model_list,
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

const RenderBody* World::render_body_ptr(const size_t i) const {
    assert(i < robot_list_.size());
    return static_cast<const RenderBody*>(robot_list_[i].get());
}

RenderBody* World::render_body_ptr(const size_t i) {
    assert(i < robot_list_.size());
    return static_cast<RenderBody*>(robot_list_[i].get());
}

}
