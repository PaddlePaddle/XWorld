#include "world.h"
#include "utils.h"

namespace xrobot {

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

void Object::SetMass(const xScalar mass) {
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::change_mass(world->client_, id(), mass);
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

RobotBase::RobotBase(const WorldWPtr& bullet_world) :
        bullet_world_(bullet_world),
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

    if (test[0].bullet_id >= 0) {
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
        WorldSPtr& world,
        RobotBaseSPtr& item,
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
    for (auto& kv : world->id_to_robot_) {
        auto body = kv.second;
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

void RobotBase::AttachTo(const RobotBaseWPtr& object) {

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

    if (object_sptr->body_data_.fixed) {
        printf("Cannot attach fixed object!\n");
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

RobotWithConvertion::RobotWithConvertion(const WorldWPtr& bullet_world) :
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
    
    if (root_part_) {
        root_part_.reset();
    }
    parts_.clear();
    joints_.clear();
    RemoveRobotFromBullet();
    bullet_world->RemoveObjectWithLabel(body_data_.body_uid);

    body_data_.label = label_;
    BulletBody::load_urdf(
            bullet_world->client_,
            object_path_list_[act_id],
            pos,
            quat,
            scale_,
            body_data_.fixed, /*fixed_base*/
            false, /*self_collision*/
            true, /*use_multibody*/
            false /*concave*/);
    load_robot_joints(object_path_list_[act_id]);
    load_robot_shapes(scale_);

    bullet_world->id_to_robot_[body_data_.body_uid] = shared_from_this();
    status_ = act_id;    

    bullet_world->AddObjectWithLabel(label_, body_data_.body_uid);
    return true;
}


void RobotWithConvertion::recycle() {
    hide(true);
    do_recycle(path_);
}

RobotWithAnimation::RobotWithAnimation(const WorldWPtr& bullet_world) :
        RobotBase(bullet_world),
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

}
