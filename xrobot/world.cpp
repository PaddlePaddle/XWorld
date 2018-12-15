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

        printf("---------------------%f\n", s[0]);

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
                      
void RobotBase::Move(const xScalar translate, const xScalar rotate) {
    assert(root_part_);
    auto bullet_world = wptr_to_sptr(bullet_world_);
    xScalar pos[3];
    xScalar quat[4];
    xScalar prev_quat[4];
    move(translate, rotate, root_part_.get(), pos, quat, prev_quat);

    set_pose(bullet_world->client_, body_data_.body_uid, pos, quat);
    bullet_world->BulletStep();

    bool reward = false;
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
            xScalar delta = glm::clamp(cp.contact_distance, -0.05f, 0.0f);
            auto& object_a = bullet_world->id_to_robot_[cp.bullet_id_a]; 
            auto& object_b = bullet_world->id_to_robot_[cp.bullet_id_b];
            
            // btVector3 contact_normal(to_object.x, to_object.y, to_object.z);
            pos[0] += sign * dir[0] * delta;
            pos[1] += sign * dir[1] * delta;
            pos[2] += sign * dir[2] * delta;
            reward = true;
        }
    }

    if(reward) {
        set_pose(bullet_world->client_, body_data_.body_uid, pos, prev_quat);
        bullet_world->BulletStep();
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
        auto object = bullet_world->id_to_robot_[test[0].bullet_id];
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
    d = glm::vec3(-width - 0.01f, -0.01f, height + 0.01f);
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

                xScalar p[3];
                p[0] = pos.x;
                p[1] = pos.y + 20.0f;
                p[2] = pos.z;

                set_pose(bullet_world->client_,
                         item->body_data_.body_uid,
                         p,
                         (xScalar*)NULL);
                // call bullet's step to enalbe the transformation
                bullet_world->BulletStep();
                // freeze the item for now
                item->Sleep();

                bool occupied = occupy_test(bullet_world, item, pos);
                if (!occupied) {

                    p[0] = pos.x;
                    p[1] = pos.y + 0.01f;
                    p[2] = pos.z;


                    set_pose(bullet_world->client_,
                             item->body_data_.body_uid,
                             p,
                             (xScalar*)NULL);
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
            rotate(bullet_world->client_, item->body_data_.body_uid, angle);
            bullet_world->BulletStep();
        }
    }
}

void RobotBase::Detach() {
    if(body_data_.attach_to_id < -1) {
        printf("Nothing to detach!\n");
        return;
    }

    if(body_data_.attach_to_id == -1) {
        root_part_->detach();
    } else {
        //body_data_.other_parts_[attach_to_id_]->attach_object_ = object;
        parts_[body_data_.attach_to_id]->detach();
    }
    body_data_.attach_to_id = -2;
}

void RobotBase::AttachTo(std::weak_ptr<RobotBase> object, const int id) {
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
        set_pose(bullet_world->client_, body_data_.body_uid, pos, (xScalar*)NULL);
        // Set Velocity to 0
        double velocity[3] = {0, 0, 0};
        set_vel(bullet_world->client_, body_data_.body_uid, velocity);
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
        path_(""),
        object_path_list_(0),
        object_name_list_(0) {}

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

    LoadURDFFile(object_path, position, rotation, scale, label, true,false,true,concave);
    ignore_baking(true);
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

World::World() : BulletWorld(),
                 robot_list_(0),
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

void World::LoadMetadata(const char * filename) {
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

        AddObjectWithLabel(tag, robot->body_data_.body_uid);

        return robot;
    } 

    find = (int) filename.find(".urdf");
    if(find > -1) {
        // Load URDF
        std::shared_ptr<RobotBase> robot = LoadModelFromCache(
            filename, position, rotation);

        if(robot) {
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
            robot->LoadURDFFile(filename, position, rotation, scale[0],
                tag, fixed_base, concave);
            robot->reuse();
            robot->Wake();
        }

        robot->UpdatePickable(pick);

        robot->reuse();

        AddObjectWithLabel(tag, robot->body_data_.body_uid);

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

                    auto robot_conv = std::dynamic_pointer_cast<RobotWithConvertion>(robot);
                    robot_conv->LoadConvertedObject(filename, position, rotation, scale[0], tag, concave);
                    robot_conv->reuse();
                    robot_conv->Wake();
                } else if (strcmp(type, "animate")== 0) {
                    robot = std::make_shared<RobotWithAnimation>(shared_from_this());

                    auto robot_anim = std::dynamic_pointer_cast<RobotWithAnimation>(robot);
                    robot_anim->LoadAnimatedObject(filename, position, rotation, scale[0], tag, concave);
                    robot_anim->reuse();
                    robot_anim->Wake();
                } else {
                    fprintf(stderr, "Incorrect Action File [%s]\n", type);
                    return std::weak_ptr<RobotBase>();
                }
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

        AddObjectWithLabel(tag, robot->body_data_.body_uid);

        return robot;
    }

    printf("Format Not Support!\n");
    return std::weak_ptr<RobotBase>();
}


void World::ResetSimulation() {
    reset();
}

void World::RemoveRobot(std::weak_ptr<RobotBase> rm_robot) 
{
    
    if(auto robot = rm_robot.lock()) {

        auto index = std::find(robot_list_.begin(), robot_list_.end(), robot);
        if(index != robot_list_.end())
        {
            int i = index - robot_list_.begin();
            auto robot = robot_list_[i];

            robot->root_part_->Sleep();
            for (int i = 0; i < robot->parts_.size(); ++i)
            {
                if(robot->parts_[i])
                    robot->parts_[i]->Sleep();
            }

            //delete robot_list_[i];
            robot_list_[i] = nullptr;
            robot_list_.erase(index);        
        }

        // ???
        //robot->RemoveRobotFromBullet();

        robot->body_data_.attach_to_id = -2;

        RemoveObjectWithLabel(robot->body_data_.body_uid);

        id_to_robot_[robot->body_data_.body_uid] = nullptr;
    }
}

void World::CleanEverything2()
{

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
    for(auto& recycle_robot : recycle_robot_map_)
    {
        printf("urdf: %s   ", recycle_robot.first.c_str());
        printf("size: %d\n", (int) recycle_robot.second.size());
    }
}

void World::CleanEverything()
{

    for (unsigned int i = 0; i < robot_list_.size(); ++i) {
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

void World::UpdateAttachObjects(RobotBaseSPtr robot) {
    if(robot->body_data_.attach_to_id < -1) 
        return;

    if(robot->body_data_.attach_to_id == -1)
    {
        if(auto attach_object_sptr = robot->root_part_->attach_object_.lock())
        {
            btTransform new_transform = 
                    robot->root_part_->object_position_;

            SetTransformation(attach_object_sptr,
                    new_transform * robot->root_part_->attach_transform_);
        } 
    }
    else if(auto attach_object_sptr = 
        robot->parts_[robot->body_data_.attach_to_id]->attach_object_.lock())
    {
        btTransform new_transform = 
                robot->parts_[robot->body_data_.attach_to_id]->object_position_;

        SetTransformation(attach_object_sptr,
                new_transform * robot->parts_[robot->body_data_.attach_to_id]->attach_transform_);
    } 
}

void World::FixLockedObjects(RobotBaseSPtr robot) {
    if(auto anim_robot = std::dynamic_pointer_cast<RobotWithAnimation>(robot)) {
        if(anim_robot->GetLock()) {
            int joint = anim_robot->GetJoint();
            float position = anim_robot->GetPosition(1);
            anim_robot->SetJointPosition(joint, position, 1.0f, 0.1f, 10000.0f);
        }
    }
}

void World::QueryPose(RobotBaseSPtr robot) {
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
    for (auto robot : robot_list_) {

        if (!robot) continue;
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

    for (auto robot : robot_list_) {
        if (!robot) continue;
        QueryPose(robot);
    }

    render_step();
}

void World::BulletInit(const float gravity, const float timestep)
{
    init(gravity, timestep);    
}

void World::SetTransformation(RobotBaseWPtr robot, const btTransform& tr) {

    if(auto robot_sptr = robot.lock()) {
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
    std::weak_ptr<RobotBase> robot_in,
    std::weak_ptr<Object> part_in,
    std::vector<ContactPoint>& contact_points)
{
    if(auto robot = robot_in.lock()) {
        robot->get_closest_points(client_, contact_points);
    }        
}

void World::GetRootContactPoints(
    std::weak_ptr<RobotBase> robot_in,
    std::weak_ptr<Object> part_in,
    std::vector<ContactPoint>& contact_points)
{
    if(auto robot = robot_in.lock()) {
        robot->get_contact_points(client_, contact_points);
    } 
}

void World::BatchRayTest(const std::vector<Ray> rays, std::vector<RayTestInfo>& result,
    const int num_threads)
{
    cast_rays(rays, result, num_threads);
}

void World::RayTest(const vec3 from, const vec3 to, RayTestInfo& result)
{
    cast_ray({from, to}, result);
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

        SetTransformation(robot, btTransform(rotation, position));
        set_vel(client_, robot->id(), kFloat3Zero);
    }
    return robot;
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
            object->root_part_->GetAABB(root_aabb_min, root_aabb_max);

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
