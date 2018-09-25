#include "world.h"

namespace xrobot {

using namespace glm;
using namespace render_engine;

Joint::Joint() : bullet_robot_(nullptr),
                 bullet_world_(nullptr),
                 joint_name_(""),
                 joint_type_(0),
                 bullet_joint_id_(-1),
                 bullet_q_index_(-1),
                 bullet_u_index_(-1),
                 joint_current_position_(0),
                 joint_current_speed_(0),
                 joint_limit_1_(-1),
                 joint_limit_2_(-2),
                 joint_max_force_(1.0f),
                 joint_max_velocity_(1.0f),
                 joint_has_limits_(false) {}

Joint::~Joint() {}

void Joint::SetJointMotorControlTorque(const float torque) {
    if(!bullet_robot_ || !bullet_world_) return;

    CommandHandle cmd_handle = b3JointControlCommandInit2(
        bullet_world_->client_,
        bullet_robot_->bullet_handle_,
        kTorque
    );

    b3JointControlSetDesiredForceTorque(cmd_handle, bullet_u_index_, torque);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle);
}

void Joint::SetJointMotorControlVelocity(const float speed,
                                         const float k_d,
                                         const float max_force) {
    if(!bullet_robot_ || !bullet_world_) return;

    CommandHandle cmd_handle = b3JointControlCommandInit2(
        bullet_world_->client_,
        bullet_robot_->bullet_handle_,
        kVelocity
    );

    b3JointControlSetDesiredVelocity(cmd_handle, bullet_u_index_, speed);
    b3JointControlSetKd(cmd_handle, bullet_u_index_, k_d);
    b3JointControlSetMaximumForce(cmd_handle, bullet_u_index_, max_force);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle);
}

void Joint::SetJointMotorControlPosition(
        const float target,
        const float k_p,
        const float k_d,
        const float max_force)
{
    if(!bullet_robot_ || !bullet_world_) return;

    CommandHandle cmd_handle = b3JointControlCommandInit2(
        bullet_world_->client_,
        bullet_robot_->bullet_handle_,
        kPositionVelocity
    );

    b3JointControlSetDesiredPosition(cmd_handle, bullet_q_index_, target);
    b3JointControlSetKd(cmd_handle, bullet_u_index_, k_d);
    b3JointControlSetKp(cmd_handle, bullet_u_index_, k_p);
    b3JointControlSetMaximumForce(cmd_handle, bullet_u_index_, max_force);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle);
}

void Joint::ResetJointState(const float pos, const float vel) {
    if(!bullet_robot_ || !bullet_world_) return;

    CommandHandle cmd_handle = b3CreatePoseCommandInit(
        bullet_world_->client_, bullet_robot_->bullet_handle_);
    b3CreatePoseCommandSetJointPosition(
            bullet_world_->client_, cmd_handle, bullet_joint_id_, pos);
    b3CreatePoseCommandSetJointVelocity(
            bullet_world_->client_, cmd_handle, bullet_joint_id_, vel);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle);
}

Object::Object() : RenderPart(),
                   bullet_world_(nullptr),
                   object_name_(),
                   bullet_handle_(-1),
                   bullet_link_id_(-1),
                   object_mass_original_(0) {
    object_position_.setIdentity();
    object_link_position_.setIdentity();
    object_local_inertial_frame_.setIdentity();
    object_speed_ = btVector3(0,0,0);
    object_angular_speed_ = btVector3(0,0,0);
}

Object::~Object() {
    for (size_t i = 0; i < transform_list_.size(); ++i) {
        delete transform_list_[i];
    }
    transform_list_.clear();
}

void Object::GetMass(float& mass) {
    if(!bullet_world_ || bullet_handle_ < 0) return;

    CommandHandle cmd_handle = b3GetDynamicsInfoCommandInit(
            bullet_world_->client_, bullet_handle_, bullet_link_id_);
    StatusHandle status_handle = b3SubmitClientCommandAndWaitStatus(
            bullet_world_->client_, cmd_handle);

    int status_type = b3GetStatusType(status_handle);
    if (status_type != CMD_GET_DYNAMICS_INFO_COMPLETED) {
        printf("Get Mass Failed! Or Could Be Static Object\n");
        return;
    }

    struct b3DynamicsInfo dynamics_info;
    b3GetDynamicsInfo(status_handle, &dynamics_info);

    mass = static_cast<float>(dynamics_info.m_mass);
}

void Object::GetAABB(glm::vec3& aabb_min, glm::vec3& aabb_max)
{
    if(!bullet_world_ || bullet_handle_ < 0) return;

    CommandHandle cmd_handle = b3RequestCollisionInfoCommandInit(
            bullet_world_->client_, bullet_handle_);
    StatusHandle status_handle = b3SubmitClientCommandAndWaitStatus(
            bullet_world_->client_, cmd_handle);

    int status_type = b3GetStatusType(status_handle);
    if (status_type != CMD_REQUEST_COLLISION_INFO_COMPLETED) {
        printf("Get AABB Failed!\n");
        return;
    }

    double aabb_min_temp[3];
    double aabb_max_temp[3];
    b3GetStatusAABB(
            status_handle, bullet_link_id_, aabb_min_temp, aabb_max_temp);

    aabb_min = vec3(aabb_min_temp[0], aabb_min_temp[1], aabb_min_temp[2]);
    aabb_max = vec3(aabb_max_temp[0], aabb_max_temp[1], aabb_max_temp[2]);
}

void Object::ApplyForce(
        const float x, const float y, const float z, const int flags) {
    if(!bullet_world_ || bullet_handle_ < 0) return;

    printf("Apply Force Has Not Been Implemented Yet!\n");
}

void Object::ApplyTorque(
        const float x, const float y, const float z, const int flags) {
    if(!bullet_world_ || bullet_handle_ < 0) return;

    CommandHandle cmd_handle = 
        b3ApplyExternalForceCommandInit(bullet_world_->client_);

    double torque_temp[3];
    torque_temp[0] = x;
    torque_temp[1] = y;
    torque_temp[2] = z;

    b3ApplyExternalTorque(
            cmd_handle, bullet_handle_, bullet_link_id_, torque_temp, flags);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle);
}

void Object::EnableSleeping() {
    if(!bullet_world_ || bullet_handle_ < 0) return;

    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(bullet_world_->client_);

    b3ChangeDynamicsInfoSetActivationState(
            cmd_handle, bullet_handle_, eActivationStateEnableSleeping);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle); 
}

void Object::DisableSleeping() {
    if(!bullet_world_ || bullet_handle_ < 0) return;

    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(bullet_world_->client_);

    b3ChangeDynamicsInfoSetActivationState(
            cmd_handle, bullet_handle_, eActivationStateDisableSleeping);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle); 
}

void Object::Sleep() {
    if(!bullet_world_ || bullet_handle_ < 0) return;

    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(bullet_world_->client_);

    b3ChangeDynamicsInfoSetActivationState(
            cmd_handle, bullet_handle_, eActivationStateSleep);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle);
}

void Object::Wake() {
    if(!bullet_world_ || bullet_handle_ < 0) return;

    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(bullet_world_->client_);

    b3ChangeDynamicsInfoSetActivationState(
            cmd_handle, bullet_handle_, eActivationStateWakeUp);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle);
}

void Object::ChangeMass(const float mass) {
    if(!bullet_world_ || bullet_handle_ < 0) return;
    if(mass < 0) return;

    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(bullet_world_->client_);

    b3ChangeDynamicsInfoSetMass(
            cmd_handle, bullet_handle_, bullet_link_id_, mass);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle);
}

void Object::ChangeLinearDamping(const float damping) {
    if(!bullet_world_ || bullet_handle_ < 0) return;
    if(damping < 0) return;

    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(bullet_world_->client_);

    b3ChangeDynamicsInfoSetLinearDamping(cmd_handle, bullet_handle_, damping);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle);
}

void Object::ChangeAngularDamping(const float damping) {
    if(!bullet_world_ || bullet_handle_ < 0) return;
    if(damping < 0) return;

    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(bullet_world_->client_);

    b3ChangeDynamicsInfoSetAngularDamping(cmd_handle, bullet_handle_, damping);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle);
}


void Object::ChangeLateralFriction(const float friction) {
    if(!bullet_world_ || bullet_handle_ < 0) return;
    if(friction < 0) return;

    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(bullet_world_->client_);

    b3ChangeDynamicsInfoSetLateralFriction(
            cmd_handle, bullet_handle_, bullet_link_id_, friction);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle);
}

void Object::ChangeSpinningFriction(const float friction) {
    if(!bullet_world_ || bullet_handle_ < 0) return;
    if(friction < 0) return;

    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(bullet_world_->client_);

    b3ChangeDynamicsInfoSetSpinningFriction(
            cmd_handle, bullet_handle_, bullet_link_id_, friction);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle);
}

void Object::ChangeRollingFriction(float friction) {
    if(!bullet_world_ || bullet_handle_ < 0) return;
    if(friction < 0) return;

    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(bullet_world_->client_);

    b3ChangeDynamicsInfoSetRollingFriction(
            cmd_handle, bullet_handle_, bullet_link_id_, friction);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle);
}

void Object::SetMassOriginal(const float mass) {
    if(mass < 0) return;

    object_mass_original_ = mass;
}

float Object::GetMassOriginal()
{
    return object_mass_original_;
}

glm::mat4 Object::translation_matrix() const {
    return TransformToMat4(object_position_);
}

glm::mat4 Object::local_inertial_frame() const {
    return TransformToMat4(object_local_inertial_frame_);
}

RobotWithConvertion::RobotWithConvertion() {


}

RobotWithConvertion::~RobotWithConvertion() {
  
}

void RobotWithConvertion::TakeAction(const int act_id) {
  
}

RobotWithAnimation::RobotWithAnimation() {

}


RobotWithAnimation::~RobotWithAnimation() {

}

Robot::Robot() : RenderBody(),
                 bullet_world_(nullptr),
                 label_("unlabeled"),
                 urdf_name_("robot_unnamed"),
                 path_("unknow_path"),
                 bullet_handle_(-1), 
                 root_part_(nullptr),
                 other_parts_(0),
                 joints_list_(0) {}

Robot::~Robot() {
    if (root_part_) {
        delete root_part_;
        root_part_ = nullptr;
    }
    for (size_t i = 0; i < other_parts_.size(); ++i) {
        delete other_parts_[i];
    }
    other_parts_.clear();

    for (size_t i = 0; i < joints_list_.size(); ++i) {
        delete joints_list_[i];
    }
}

void Robot::Teleport2(const glm::vec3 walk_move,
                      const glm::vec3 walk_rotate,
                      const float speed, const bool remit) {

    static std::queue<btTransform> robot_transform_queue;

    if(remit) {
        while(!robot_transform_queue.empty()) {
            robot_transform_queue.pop();
        }
    }

    static btTransform robot_transform = root_part_->object_position_;
    btVector3 robot_position = robot_transform.getOrigin();
    btQuaternion robot_rotation = robot_transform.getRotation(); 

    robot_transform_queue.push(robot_transform);

    if(robot_transform_queue.size() > 50) {
        robot_transform_queue.pop();
    }

    glm::vec3 walk_dir_temp = glm::normalize(walk_move);
    btVector3 walk_dir = btVector3(walk_dir_temp.x, walk_dir_temp.y, walk_dir_temp.z);
    walk_dir = btTransform(robot_rotation) * walk_dir;

    btVector3 walk_rot_temp = btVector3(walk_rotate.x, walk_rotate.y, walk_rotate.z);
    btQuaternion walk_rot = btQuaternion(walk_rot_temp, speed);

    btVector3 robot_position_t = robot_position;
    btQuaternion robot_rotation_t = robot_rotation;

    if(glm::dot(walk_move, glm::vec3(1)) > 0.001f) {
        robot_position_t += walk_dir * speed;
    }
    if(glm::dot(walk_rotate, glm::vec3(1)) > 0.001f) {
        robot_rotation_t *= walk_rot;
    }

    btTransform robot_transform_t = robot_transform;
    robot_transform_t.setOrigin(robot_position_t);
    robot_transform_t.setRotation(robot_rotation_t);
    bullet_world_->SetTransformation(this, robot_transform);
    bullet_world_->BulletStep();

    std::vector<ContactPoint> contact_points;
    bullet_world_->GetRootContactPoints(this, this->root_part_, contact_points);

    for (int i = 0; i < contact_points.size(); ++i)
    {
        ContactPoint cp = contact_points[i];
        if(cp.contact_distance < -0.01) {

            robot_transform.setOrigin(robot_transform_queue.front().getOrigin());
            robot_transform.setRotation(robot_transform_queue.front().getRotation());
            bullet_world_->SetTransformation(this, robot_transform);
            bullet_world_->BulletStep();
            while(!robot_transform_queue.empty()) {
                robot_transform_queue.pop();
            }
            return;
        }
    }

    robot_transform = robot_transform_t;
}

void Robot::Teleport(const glm::vec3 walk_move,
                     const glm::vec3 walk_rotate,
                     const float speed, const bool remit) {

    static btTransform robot_transform = root_part_->object_position_;
    static btVector3 robot_position = robot_transform.getOrigin();
    static btQuaternion robot_rotation = robot_transform.getRotation();

    if(remit) {
        robot_transform = root_part_->object_position_;
        robot_position = robot_transform.getOrigin();
        robot_rotation = robot_transform.getRotation(); 
    }

    glm::vec3 walk_dir_temp = glm::normalize(walk_move);
    btVector3 walk_dir = btVector3(walk_dir_temp.x, walk_dir_temp.y, walk_dir_temp.z);
    walk_dir = btTransform(robot_rotation) * walk_dir;

    btVector3 walk_rot_temp = btVector3(walk_rotate.x, walk_rotate.y, walk_rotate.z);
    btQuaternion walk_rot = btQuaternion(walk_rot_temp, speed);

    btVector3 robot_position_t = robot_position;
    btQuaternion robot_rotation_t = robot_rotation;

    bool freeze_flag = true;

    if(glm::dot(walk_move, glm::vec3(1)) > 0.001f) {
        robot_position_t += walk_dir * speed;
        freeze_flag = false;
    }
    if(glm::dot(walk_rotate, glm::vec3(1)) > 0.001f) {
        robot_rotation_t *= walk_rot;
        freeze_flag = false;
    }

    // if(freeze_flag)
    //     return;

    robot_transform.setOrigin(robot_position_t);
    robot_transform.setRotation(robot_rotation_t);
    bullet_world_->SetTransformation(this, robot_transform);
    bullet_world_->BulletStep();

    std::vector<ContactPoint> contact_points;
    bullet_world_->GetRootContactPoints(this, this->root_part_, contact_points);
    int contact_ground_flag = 0;

    for (int i = 0; i < contact_points.size(); ++i)
    {
        ContactPoint cp = contact_points[i];
        if(cp.contact_distance < -0.01) {

            float p = glm::clamp(cp.contact_distance, -0.02f, 0.0f) * 1.0f;
            glm::vec3 move_out_temp = glm::normalize(cp.contact_normal) * p;
            btVector3 move_out = btVector3(move_out_temp.x, 0, move_out_temp.z);

            //printf("move: %f %f %f\n", move_out[0], move_out[1], move_out[2]);

            //robot_position += move_out;

            robot_transform.setOrigin(robot_position);
            robot_transform.setRotation(robot_rotation);
            bullet_world_->SetTransformation(this, robot_transform);
            bullet_world_->BulletStep();

            return;
        }
    }

    robot_position = robot_position_t;
    robot_rotation = robot_rotation_t;
    // printf("res: %d\n", res);
}

void Robot::Freeze(const bool remit) {
    if(root_part_) {
        Teleport(kVec3Zero, kVec3Zero, 0.0f, remit);
    }
}

void Robot::MoveForward(const float speed) {
    if(root_part_) {
        Teleport(kVec3Front, kVec3Zero, speed);
    }
}

void Robot::MoveBackward(const float speed) {
    if(root_part_) {
        Teleport(kVec3Front, kVec3Zero, -speed);
    }
}

void Robot::TurnLeft(const float speed) {
    if(root_part_) {
        Teleport(kVec3Zero, kVec3Right, speed);
    }
}

void Robot::TurnRight(const float speed) {
    if(root_part_) {
        Teleport(kVec3Zero, kVec3Right, -speed);
    }
}

void Robot::PickUp(Inventory * inventory, 
        const glm::vec3 from, const glm::vec3 to) {

    assert(inventory);
    assert(glm::dot(from, glm::vec3(0,0,0)) != 0.0f);
    assert(glm::dot(to, glm::vec3(0,0,0)) != 0.0f);

    std::vector<RayTestInfo> temp_res;
    std::vector<Ray> temp_ray;
    temp_ray.push_back({from, to});

    bullet_world_->BatchRayTest(temp_ray, temp_res);

    if(temp_res[0].bullet_id > 0) {

        Robot* object = bullet_world_->bullet_handle_to_robot_map_[
                temp_res[0].bullet_id];

        if(inventory->IsPickableObject(object->label_)) {
            inventory->PutObject(object->label_, object->path_);
            bullet_world_->RemoveRobot2(object);
            //scene_->map_bullet_label_.erase(
            //        scene_->map_bullet_label_.find(temp_res[0].bullet_id));
        }
    }

}

void Robot::PutDown(Inventory * inventory, 
        const glm::vec3 from, const glm::vec3 to) {
    
    assert(inventory);
    assert(glm::dot(from, glm::vec3(0,0,0)) != 0.0f);
    assert(glm::dot(to, glm::vec3(0,0,0)) != 0.0f);

    std::vector<RayTestInfo> temp_res;
    std::vector<Ray> temp_ray;
    temp_ray.push_back({from, to});

    bullet_world_->BatchRayTest(temp_ray, temp_res);

    // Ray Hit
    if(temp_res[0].bullet_id > 0) {
        Robot * object = bullet_world_->bullet_handle_to_robot_map_[
                temp_res[0].bullet_id];
        glm::vec3 normal = temp_res[0].norm;
        glm::vec3 position = temp_res[0].pos;

        // Horizontal Flat Fragment
        if(object && glm::dot(normal, glm::vec3(0,1,0)) > 0.8f) {
            glm::vec3 aabb_min, aabb_max;
            object->root_part_->GetAABB(aabb_min, aabb_max);


            // On the Surface ???
            if(aabb_max.y - 0.05f < position.y || true) {
                std::string temp_obj_label; 
                std::string temp_obj_path = inventory->GetObjectRandomly(temp_obj_label);

                std::size_t postfix = temp_obj_path.find(".urdf");

                // Get Object File Path
                if(!temp_obj_path.empty()) {
                    Robot * temp_obj;
                    if((int) postfix < 0) {
                        temp_obj = bullet_world_->LoadOBJ(
                            temp_obj_path,
                            btVector3(position.x,20,position.z),
                            btQuaternion(btVector3(-1,0,0),1.57),
                            btVector3(1,1,1),
                            temp_obj_label,
                            0,
                            false,
                            true
                        );
                        printf("obj!!!!!!!!!\n");
                    } else {
                        temp_obj = bullet_world_->LoadURDF(
                            temp_obj_path,
                            btVector3(position.x,20,position.z),
                            btQuaternion(btVector3(-1,0,0),1.57),
                            1.0f,
                            temp_obj_label,
                            true
                        );
                    }
                    temp_obj->Sleep();

                    glm::vec3 aabb_min0, aabb_max0;
                    temp_obj->root_part_->GetAABB(aabb_min0, aabb_max0);

                    float width = (aabb_max0.x - aabb_min0.x) / 2;
                    float height = (aabb_max0.z - aabb_min0.z) / 2;


                    bool intersect = false;

                    printf("position: %f %f %f\n", position.x, position.y, position.z);

                    // In Range
                    std::vector<RayTestInfo> temp1_res;
                    std::vector<Ray> temp1_ray;
                    temp1_ray.push_back({glm::vec3(position.x,0.05f + position.y,position.z),
                            glm::vec3(position.x,0.05f + position.y,position.z) + glm::vec3(width + 0.01f, 0, 0)});
                    temp1_ray.push_back({glm::vec3(position.x,0.05f + position.y,position.z),
                            glm::vec3(position.x,0.05f + position.y,position.z) - glm::vec3(width + 0.01f, 0, 0)});
                    temp1_ray.push_back({glm::vec3(position.x,0.05f + position.y,position.z),
                            glm::vec3(position.x,0.05f + position.y,position.z) + glm::vec3(0, 0, height + 0.01f)});
                    temp1_ray.push_back({glm::vec3(position.x,0.05f + position.y,position.z),
                            glm::vec3(position.x,0.05f + position.y,position.z) - glm::vec3(0, 0, height + 0.01f)});
                    temp1_ray.push_back({glm::vec3(position.x,0.05f + position.y,position.z),
                            glm::vec3(position.x,0.05f + position.y,position.z) + glm::vec3(width + 0.01f, -0.01f, height + 0.01f)});
                    temp1_ray.push_back({glm::vec3(position.x,0.05f + position.y,position.z),
                            glm::vec3(position.x,0.05f + position.y,position.z) - glm::vec3(width + 0.01f, -0.01f, height + 0.01f)});
                    temp1_ray.push_back({glm::vec3(position.x,0.05f + position.y,position.z),
                            glm::vec3(position.x,0.05f + position.y,position.z) + glm::vec3(-width - 0.01f, -0.01f, height + 0.01f)});
                    temp1_ray.push_back({glm::vec3(position.x,0.05f + position.y,position.z),
                            glm::vec3(position.x,0.05f + position.y,position.z) - glm::vec3(-width - 0.01f, -0.01f, height + 0.01f)});

                    bullet_world_->BatchRayTest(temp1_ray, temp1_res);

                    for (int i = 0; i < temp1_res.size(); ++i)
                    {
                        printf("uid: %d\n", temp1_res[i].bullet_id);
                        if(temp1_res[i].bullet_id >= 0)
                            intersect = true;
                    }

                    
                    if(!intersect) {
                        // TODO
                        // Add a List for Special Objs

                        printf("ok......\n");

                        for (size_t i = 0; i < bullet_world_->size(); i++) {
                            Robot* body = bullet_world_->robot_list_[i];
                            Object * part = body->root_part_;
                            if (part && !body->recycle() 
                                && part->id()!=temp_obj->bullet_handle_
                                && part->id()!=object->bullet_handle_
                                && body->label_!= "Wall"
                                && body->label_!= "Floor"
                                && body->label_!= "Ceiling")
                            {
                                glm::vec3 aabb_min1, aabb_max1;
                                part->GetAABB(aabb_min1, aabb_max1);

                                if(aabb_min1.x < aabb_max0.x &&
                                   aabb_max1.x > aabb_min0.x &&
                                   aabb_min1.z < aabb_max0.z &&
                                   aabb_max1.z > aabb_min0.z) {
                                    intersect = true;
                                    break;
                                }
                            }
                        }

                        if(intersect) {
                            inventory->PutObject(temp_obj->label_, temp_obj->path_);
                            bullet_world_->RemoveRobot2(temp_obj);
                        } else {
                            float height = (aabb_max0.y - aabb_min0.y) * 0.5f;
                            glm::vec3 intersection = temp_res[0].pos;

                            btTransform tr;
                            tr.setIdentity();
                            tr.setOrigin(btVector3(intersection.x, intersection.y, intersection.z));

                            bullet_world_->SetTransformation(temp_obj, tr);
                            bullet_world_->BulletStep();
                        }

                    } else {
                        inventory->PutObject(temp_obj->label_, temp_obj->path_);
                        bullet_world_->RemoveRobot2(temp_obj);
                    }
                }
            }
        }
    }
}

void Robot::RotateObject(const float rotate_angle_y,
    const glm::vec3 from, const glm::vec3 to) {
    
    assert(glm::dot(from, glm::vec3(0,0,0)) != 0.0f);
    assert(glm::dot(to, glm::vec3(0,0,0)) != 0.0f);

    std::vector<RayTestInfo> temp_res;
    std::vector<Ray> temp_ray;
    temp_ray.push_back({from, to});

    bullet_world_->BatchRayTest(temp_ray, temp_res);

    if(temp_res[0].bullet_id > 0) {
        Robot * temp_obj = bullet_world_->bullet_handle_to_robot_map_[temp_res[0].bullet_id];

        if(temp_obj->label_!= "Wall" 
            && temp_obj->label_!= "Floor"
            && temp_obj->label_!= "Ceiling") {

            btTransform tr = temp_obj->root_part_->object_position_;
            tr.setRotation(btQuaternion(btVector3(0,1,0), rotate_angle_y));
            bullet_world_->SetTransformation(temp_obj, tr);
            bullet_world_->BulletStep();
        }
    }
}

void Robot::SetJointVelocity(const int joint_id, const float speed,
                             const float k_d, const float max_force) {
    assert(joint_id > -1 && joint_id < joints_list_.size());

    joints_list_[joint_id]->SetJointMotorControlVelocity(speed, k_d,
            max_force);
}

void Robot::SetJointPosition(const int joint_id, const float target,
                             const float k_p, const float k_d,
                             const float max_force) {
    assert(joint_id > -1 && joint_id < joints_list_.size());

    joints_list_[joint_id]->SetJointMotorControlPosition(target, k_d,
            k_p, max_force);
}

void Robot::ResetJointState(const int joint_id, const float pos,
                            const float vel) {
    assert(joint_id > -2 && joint_id < joints_list_.size());

    joints_list_[joint_id]->ResetJointState(pos, vel);
}

void Robot::RemoveRobotFromBullet() {
    if(bullet_handle_ < 0) return;

    b3SubmitClientCommandAndWaitStatus(
            bullet_world_->client_, 
            b3InitRemoveBodyCommand(bullet_world_->client_, bullet_handle_));
}

void Robot::Sleep()
{
    if(bullet_handle_ < 0) return;

    if (root_part_) {
        root_part_->Sleep();
    }

    for (Object* part : other_parts_) {
       if (part) {
            part->Sleep();
       }
    }
}

void Robot::Wake()
{
    if(bullet_handle_ < 0) return;

    if (root_part_) {
        root_part_->Wake();
    }

    for (Object* part : other_parts_) {
       if (part) {
            part->Wake();
       }
    }
}

void Robot::DisableSleeping()
{
    if(bullet_handle_ < 0) return;

    if (root_part_) {
        root_part_->DisableSleeping();
        root_part_->Wake();
    }

    for (Object* part : other_parts_) {
       if (part) {
            part->DisableSleeping();
            part->Wake();
       }
    }
}

void Robot::CalculateInverseKinematics(
        const int end_index, 
        const btVector3 target_position,
        const btQuaternion target_orientation,
        double* joint_damping,
        double* ik_output_joint_pos,
        int &num_poses) {
    const int solver = 0;
    const int num_joints = b3GetNumJoints(
            bullet_world_->client_, bullet_handle_);
    const int dof = b3ComputeDofCount(bullet_world_->client_, bullet_handle_);

    CommandHandle cmd_handle = b3CalculateInverseKinematicsCommandInit(
            bullet_world_->client_, bullet_handle_);
    b3CalculateInverseKinematicsSelectSolver(cmd_handle, solver);

    double position_temp[3];
    double orientation_temp[4] = {0, 0, 0, 1};

    position_temp[0] = target_position[0];
    position_temp[1] = target_position[1];
    position_temp[2] = target_position[2];

    orientation_temp[0] = target_orientation[0];
    orientation_temp[1] = target_orientation[1];
    orientation_temp[2] = target_orientation[2];
    orientation_temp[3] = target_orientation[3];

    b3CalculateInverseKinematicsAddTargetPositionWithOrientation(
            cmd_handle, end_index, position_temp, orientation_temp);
    b3CalculateInverseKinematicsSetJointDamping(cmd_handle, dof, joint_damping);

    StatusHandle status_handle = b3SubmitClientCommandAndWaitStatus(
            bullet_world_->client_, cmd_handle);

    int result_body_index;
    int result = b3GetStatusInverseKinematicsJointPositions(
            status_handle, &result_body_index, &num_poses, 0);

    if (result && num_poses) {
        result = b3GetStatusInverseKinematicsJointPositions(
                status_handle,
                &result_body_index,
                &num_poses,
                ik_output_joint_pos);
    }
}

const RenderPart* Robot::render_root_ptr() const {
    return static_cast<const RenderPart*>(root_part_);
}

RenderPart* Robot::render_root_ptr() {
    return static_cast<RenderPart*>(root_part_);
}

const RenderPart* Robot::render_part_ptr(const size_t i) const {
    assert(i < other_parts_.size());
    return static_cast<const RenderPart*>(other_parts_[i]);
}

RenderPart* Robot::render_part_ptr(const size_t i) {
    assert(i < other_parts_.size());
    return static_cast<RenderPart*>(other_parts_[i]);
}

void Robot::attach_camera(const glm::vec3& offset,
                          const float pitch,
                          glm::vec3& loc,
                          glm::vec3& front,
                          glm::vec3& right,
                          glm::vec3& up) {
    if (!root_part_) { return; }

    btTransform pose = root_part_->object_position_;
    auto base_orientation = pose.getBasis();
    btVector3 base_front = base_orientation * btVector3(1, 0, 0); 

    // TODO: this part of codes are just used to compute the offset of camera
    // relative to the body, i.e., camera_aim.
    glm::vec3 f = glm::normalize(
            glm::vec3(base_front[0], base_front[1], base_front[2]));
    glm::vec3 r = glm::normalize(glm::cross(f, glm::vec3(0,-1,0)));
    glm::vec3 u = glm::normalize(glm::cross(f, r));
    glm::vec3 camera_aim(f*offset.x + u*offset.y + r*offset.z);

    btMatrix3x3 pre_orientation;
    pre_orientation.setIdentity();
    pre_orientation.setEulerYPR(0, glm::radians(pitch), 0);
    base_front = base_orientation * pre_orientation * btVector3(1, 0, 0); 
    front = glm::normalize(vec3(base_front[0], base_front[1], base_front[2]));
    right = glm::normalize(glm::cross(front, glm::vec3(0,-1,0)));
    up = glm::normalize(glm::cross(front, right));

    auto base_position = pose.getOrigin();
    loc = glm::vec3(base_position[0], base_position[1], base_position[2]) 
          + camera_aim;
}

World::World() : robot_list_(0),
                 bullet_handle_to_robot_map_(),
                 client_(0), 
                 recycle_robot_map_(),
                 model_cache_(),
                 object_locations_(),
                 bullet_gravity_(0),
                 bullet_timestep_(0),
                 bullet_timestep_sent_(0),
                 bullet_skip_frames_sent_(0),
                 bullet_ts_(0),
                 reset_count_(0) {}

World::~World() {
    CleanEverything();
    ClearCache();
    b3DisconnectSharedMemory(client_);
}

void World::ResetSimulation() {
    b3SubmitClientCommandAndWaitStatus(
        client_, b3InitResetSimulationCommand(client_));
}

void World::RemoveRobot(Robot * robot) {
    if(!robot) return;

    auto index = std::find(robot_list_.begin(), robot_list_.end(), robot);
    if(index != robot_list_.end())
    {
        int i = index - robot_list_.begin();
        Robot * robot = robot_list_[i];

        robot->root_part_->Sleep();
        for (int i = 0; i < robot->other_parts_.size(); ++i)
        {
            if(robot->other_parts_[i])
                robot->other_parts_[i]->Sleep();
        }

        delete robot_list_[i];
        robot_list_[i] = nullptr;
        robot_list_.erase(index);        
    }

    RemoveObjectWithLabel(robot->bullet_handle_);

    bullet_handle_to_robot_map_[robot->bullet_handle_] = nullptr;
}

void World::RemoveRobot2(Robot * robot) {
    robot->recycle(true);

    // Move to Bottom
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(0, -5, 0));

    // Set Velocity to 0
    SetTransformation(robot, transform);
    SetVelocity(robot, btVector3(0,0,0));

    // Change Root to Static
    float mass;
    robot->root_part_->GetMass(mass);
    robot->root_part_->ChangeMass(0.0f);
    robot->root_part_->SetMassOriginal(mass);
    robot->root_part_->Sleep();

    // Change Rest to Static
    for (Object * part : robot->other_parts_)
    {
        part->GetMass(mass);
        part->ChangeMass(0.0f);
        part->SetMassOriginal(mass);
        part->Sleep();
    }

    // Push Into Recycles
    if(recycle_robot_map_.find(robot->urdf_name_) != recycle_robot_map_.end())
    {
        recycle_robot_map_[robot->urdf_name_].push_back(robot);
    }
    else
    {
        std::vector<Robot *> robot_list_temp;
        robot_list_temp.push_back(robot);
        recycle_robot_map_[robot->urdf_name_] = robot_list_temp;
    }

    RemoveObjectWithLabel(robot->bullet_handle_);
}

void World::CleanEverything2()
{
    for (Robot * robot : robot_list_)
    {
        if(!robot->recycle())
            RemoveRobot2(robot);
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
            delete robot_list_[i];
        }
    }

    reset_count_++;

    remove_all_cameras();

    recycle_robot_map_.clear();
    robot_list_.clear();
    bullet_handle_to_robot_map_.clear();
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

    for (Robot * robot : robot_list_)
    {
        if (!robot) continue;
        CommandHandle cmd_handle = 0;
        
        for (Joint * joint : robot->joints_list_)
        {
            if(!joint) continue;
            if(!cmd_handle) 
                cmd_handle = b3JointControlCommandInit2(client_, robot->bullet_handle_, kVelocity);
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
    for (const Robot* robot : robot_list_)
    {
        if (!robot) continue;
        QueryPosition(robot);
    }
}

void World::QueryPosition(const Robot* robot)
{
    if(!robot->root_part_) return;

    CommandHandle cmd_handle =
            b3RequestActualStateCommandInit(client_, robot->bullet_handle_);
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

    assert(robot->root_part_->bullet_link_id_==-1);
    robot->root_part_->object_position_ = TransformFromDoubles(q, q+3);
    robot->root_part_->object_speed_[0] = q_dot[0];
    robot->root_part_->object_speed_[1] = q_dot[1];
    robot->root_part_->object_speed_[2] = q_dot[2];
    robot->root_part_->object_angular_speed_[0] = q_dot[3];
    robot->root_part_->object_angular_speed_[1] = q_dot[4];
    robot->root_part_->object_angular_speed_[2] = q_dot[5];
    robot->root_part_->object_local_inertial_frame_ = TransformFromDoubles(root_inertial_frame, root_inertial_frame+3);
    robot->root_part_->object_link_position_ = TransformFromDoubles(q, q+3);

    int status_type = b3GetStatusType(status_handle);
    if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED){
        printf("Query Position Failed!\n");
        return;
    }

    for (Object* part : robot->other_parts_)
    {
        struct b3LinkState link_state;
        if(!part) continue;
        if(part->bullet_link_id_ == -1) continue;

        b3GetLinkState(client_, status_handle, part->bullet_link_id_, &link_state);
        part->object_position_ = TransformFromDoubles(link_state.m_worldPosition,
                                                      link_state.m_worldOrientation);
        part->object_local_inertial_frame_ =
                TransformFromDoubles(link_state.m_localInertialPosition,
                                     link_state.m_localInertialOrientation);
        part->object_link_position_ =
                TransformFromDoubles(link_state.m_worldLinkFramePosition,
                                     link_state.m_worldLinkFrameOrientation);
        part->object_speed_[0] = link_state.m_worldLinearVelocity[0];
        part->object_speed_[1] = link_state.m_worldLinearVelocity[1];
        part->object_speed_[2] = link_state.m_worldLinearVelocity[2];
        part->object_angular_speed_[0] = link_state.m_worldAngularVelocity[0];
        part->object_angular_speed_[1] = link_state.m_worldAngularVelocity[1];
        part->object_angular_speed_[2] = link_state.m_worldAngularVelocity[2];
    }

    for (Joint * joint : robot->joints_list_)
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

void World::GetRootContactPoints(const Robot* robot, const Object* part,
        std::vector<ContactPoint>& contact_points)
{
    int bodyUniqueIdA = robot->root_part_->bullet_handle_;
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
            point.contact_normal = glm::vec3(normal_x, normal_y, normal_z);
            point.contact_distance = contact_point_data.m_contactPointData[i].m_contactDistance;
            point.bullet_id_a = contact_point_data.m_contactPointData[i].m_bodyUniqueIdA;
            point.bullet_id_b = contact_point_data.m_contactPointData[i].m_bodyUniqueIdB;
            contact_points.push_back(point);
        }
        //printf("contact: %d\n", contact_point_data.m_numContactPoints);
    }
}

void World::CharacterMove(Robot* robot, const glm::vec3 walk_move,
            const glm::vec3 walk_rotate, const float speed)
{
    btTransform robot_transform = robot->root_part_->object_position_;
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
    Robot* parent,
    const btVector3& parent_relative_position,
    const btVector3& child_relative_position,
    const btQuaternion& parent_relative_orientation,
    const btQuaternion& child_relative_orientation)
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
            parent->bullet_handle_,
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
    return -1;
}


void World::SetVelocity(
    Robot* robot,
    const btVector3& velocity)
{
    CommandHandle cmd_handle = b3CreatePoseCommandInit(client_, robot->bullet_handle_);

    double velocity_temp[3];
    velocity_temp[0] = velocity[0];
    velocity_temp[1] = velocity[1];
    velocity_temp[2] = velocity[2];
    b3CreatePoseCommandSetBaseLinearVelocity(cmd_handle, velocity_temp);
    b3SubmitClientCommandAndWaitStatus(client_, cmd_handle);
}

void World::SetTransformation(
    Robot* robot,
    const btTransform& tranform)
{
    assert(robot);
    CommandHandle cmd_handle = b3CreatePoseCommandInit(client_, robot->bullet_handle_);

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

Robot* World::LoadModelFromCache(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation) {

    Robot * robot = nullptr;

    if (recycle_robot_map_.find(filename) != recycle_robot_map_.end() && 
            !recycle_robot_map_[filename].empty()) {

        robot = recycle_robot_map_[filename].back();
        recycle_robot_map_[filename].pop_back();
        robot->recycle(false);

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
            Robot * object = bullet_handle_to_robot_map_[bullet_id];

            glm::vec3 root_aabb_min, root_aabb_max;
            object->root_part_->GetAABB(root_aabb_min, root_aabb_max);

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
            Robot * object = bullet_handle_to_robot_map_[bullet_id];

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

Robot* World::LoadURDFFile(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation,
        const float scale,
        const std::string& label,
        const bool fixed_base,
        const bool self_collision,
        const bool use_multibody) {
    Robot * robot = new Robot();
    robot->bullet_world_ = this;
    robot->urdf_name_ = filename;
    robot->path_ = filename;
   
    CommandHandle cmd_handle = b3LoadUrdfCommandInit(client_, filename.c_str());
    
    b3LoadUrdfCommandSetStartPosition(
            cmd_handle, position[0], position[1], position[2]);
    b3LoadUrdfCommandSetStartOrientation(
            cmd_handle, rotation[0], rotation[1], rotation[2], rotation[3]);

    b3LoadUrdfCommandSetUseFixedBase(cmd_handle, fixed_base);
    b3LoadUrdfCommandSetGlobalScaling(cmd_handle, scale);
    b3LoadUrdfCommandSetUseMultiBody(cmd_handle, use_multibody);

    if (self_collision) {
        b3LoadUrdfCommandSetFlags(
                cmd_handle, kURDFSelfCollision | kURDFSelfCollisionExParents);
    } else {
        b3LoadUrdfCommandSetFlags(cmd_handle, 0);
    }

    StatusHandle status_handle =
            b3SubmitClientCommandAndWaitStatus(client_, cmd_handle);
    int status_type = b3GetStatusType(status_handle);
    if (status_type != CMD_URDF_LOADING_COMPLETED) {
        fprintf(stderr, "Cannot load URDF file '%s'.\n", filename.c_str());
        return robot;
    }

    robot->bullet_handle_ = b3GetStatusBodyIndex(status_handle);

    LoadRobotJoint(robot, filename);
    LoadRobotShape(robot, scale);

    robot->root_part_->bullet_handle_ = robot->bullet_handle_;
    robot->root_part_->EnableSleeping();

    robot_list_.push_back(robot);
    bullet_handle_to_robot_map_[robot->bullet_handle_] = robot;

    AddObjectWithLabel(label, robot->bullet_handle_);

    robot->label_ = label;

    return robot;
}

Robot* World::LoadURDF(
    const std::string& filename,
    const btVector3 position,
    const btQuaternion rotation,
    const float scale,
    const std::string& label,
    const bool fixed_base)
{
    Robot * robot = LoadModelFromCache(filename, position, rotation);

    if(robot) {
        float mass;
        robot->root_part_->ChangeMass(robot->root_part_->GetMassOriginal());
        robot->root_part_->Wake();

        for (Joint * joint : robot->joints_list_) {
            if(joint) {
                joint->ResetJointState(0.0f, 0.005f);
            }
        }

        for (Object * part : robot->other_parts_) {
            part->ChangeMass(part->GetMassOriginal());
        }

        AddObjectWithLabel(label, robot->bullet_handle_);
    } else {
        robot = LoadURDFFile(
            filename, position, rotation, scale, label, fixed_base, false, true);
        robot->recycle(false);
    }

    return robot;
}

Robot* World::LoadOBJ(
    const std::string& filename,
    const btVector3 position,
    const btQuaternion rotation,
    const btVector3 scale,
    const std::string& label,
    const float mass,
    const bool flip,
    const bool concave)
{

    std::string filename_with_scale = filename + ":" + 
        std::to_string(scale[0]) + ":" + std::to_string(scale[1]) + ":" + std::to_string(scale[2]);

    Robot* robot = LoadModelFromCache(filename_with_scale, position, rotation);

    if(!robot) {
        robot = LoadOBJFile(
            filename,
            position,
            rotation,
            scale,
            label,
            mass,
            flip,
            concave
        );
    }
    return robot;
}

Robot* World::LoadOBJFile(
    const std::string& filename,
    const btVector3 position,
    const btQuaternion rotation,
    const btVector3 scale,
    const std::string& label,
    const float mass,
    const bool flip,
    const bool concave) {

    std::string filename_with_scale = filename + ":" + 
        std::to_string(scale[0]) + ":" + std::to_string(scale[1]) + ":" + std::to_string(scale[2]);

    Robot* robot = new Robot();
    robot->bullet_world_ = this;
    robot->urdf_name_ = filename_with_scale;
    robot->path_ = filename;
    robot->recycle(false);

    CommandHandle cmd_handle = b3LoadObjCommandInit(client_, filename.c_str());

    if (concave) {
        b3LoadObjCommandSetFlags(cmd_handle, kOBJConcave);
    }

    b3LoadObjCommandSetStartPosition(
            cmd_handle, position[0], position[1], position[2]);
    b3LoadObjCommandSetStartOrientation(
            cmd_handle, rotation[0], rotation[1], rotation[2], rotation[3]);
    b3LoadObjCommandSetStartScale(cmd_handle, scale[0], scale[1], scale[2]);
    b3LoadObjCommandSetMass(cmd_handle, mass);
    StatusHandle status_handle = b3SubmitClientCommandAndWaitStatus(client_, cmd_handle);

    int status_type = b3GetStatusType(status_handle);
    if (status_type != CMD_OBJ_LOADING_COMPLETED) {
        fprintf(stderr, "Cannot load OBJ file '%s'.\n", filename.c_str());
        return robot;
    }

    robot->bullet_handle_ = b3GetStatusBodyIndex(status_handle);

    b3BodyInfo root;
    b3GetBodyInfo(client_, robot->bullet_handle_, &root);

    robot->root_part_ = new Object();
    robot->root_part_->object_name_ = root.m_baseName;
    robot->root_part_->bullet_handle_ = robot->bullet_handle_;
    robot->root_part_->bullet_link_id_ = -1;
    robot->root_part_->bullet_world_ = this;

    bool reset = true;
    ModelData* model_data = FindInCache(filename, robot->root_part_->model_list_, reset);

    glm::mat4 transform = TransformToMat4(btTransform(rotation, position));

    OriginTransformation * origin_transform = new OriginTransformation();
    origin_transform->scale = 1.0f;
    origin_transform->local_scale = vec3(scale[0], scale[1], scale[2]);
    origin_transform->origin = transform;

    if (flip) {
        origin_transform->flip = -1.0f;
    } else {
        origin_transform->flip = 1.0f;
    }

    robot->root_part_->transform_list_.push_back(origin_transform);

    if (reset) {
        model_data->primitive_type_ = kMesh;
        model_data->directory_ = filename;
        model_data->Reset();
    }

    robot_list_.push_back(robot);
    bullet_handle_to_robot_map_[robot->bullet_handle_] = robot;

    AddObjectWithLabel(label, robot->bullet_handle_);

    robot->label_ = label;

    return robot;
}



void World::LoadRobotJoint(Robot* robot, const std::string &filename)
{
    b3BodyInfo root;
    b3GetBodyInfo(client_, robot->bullet_handle_, &root);

    robot->root_part_ = new Object();
    robot->root_part_->object_name_ = root.m_baseName;
    robot->root_part_->bullet_handle_ = robot->bullet_handle_;
    robot->root_part_->bullet_link_id_ = -1;
    robot->root_part_->bullet_world_ = this;

    robot->urdf_name_ = filename;

    int num_joints = b3GetNumJoints(client_, robot->bullet_handle_);
    robot->joints_list_.resize(num_joints);
    robot->other_parts_.resize(num_joints);

    for (unsigned int c = 0; c < num_joints; ++c)
    {
        struct b3JointInfo info;
        b3GetJointInfo(client_, robot->bullet_handle_, c, &info);

        if (info.m_jointType == kRevolute || info.m_jointType == kPrismatic) {
            Joint * joint = new Joint();
            joint->bullet_robot_ = robot;
            joint->bullet_world_ = this;
            joint->bullet_joint_id_ = c;
            joint->joint_name_ = info.m_jointName;
            joint->joint_type_ = info.m_jointType == kRevolute ? kRotationMotor : kLinearMotor;
            joint->bullet_q_index_ = info.m_qIndex;
            joint->bullet_u_index_ = info.m_uIndex;
            joint->joint_has_limits_ = info.m_jointLowerLimit < info.m_jointUpperLimit;
            joint->joint_limit_1_ = info.m_jointLowerLimit;
            joint->joint_limit_2_ = info.m_jointUpperLimit;
            joint->joint_max_force_ = info.m_jointMaxForce;
            joint->joint_max_velocity_ = info.m_jointMaxVelocity;
            robot->joints_list_[c] = joint;
        } else {
            robot->joints_list_[c] = nullptr;
        }

        Object * part = robot->other_parts_[c];
        part = new Object();
        part->bullet_handle_ = robot->bullet_handle_;
        part->bullet_link_id_ = c;
        part->bullet_world_ = this;
        part->object_name_ = info.m_linkName;
        robot->other_parts_[c] = part;
    }
}

void World::LoadRobotShape(Robot* robot, const float scale)
{
    CommandHandle cmd_handle =
            b3InitRequestVisualShapeInformation(client_, robot->bullet_handle_);
    StatusHandle status_handle =
            b3SubmitClientCommandAndWaitStatus(client_, cmd_handle);
    int status_type = b3GetStatusType(status_handle);
    if (status_type != CMD_VISUAL_SHAPE_INFO_COMPLETED) return;

    b3VisualShapeInformation visualShapeInfo;
    b3GetVisualShapeInformation(client_, &visualShapeInfo);

    int num_visualshape = visualShapeInfo.m_numVisualShapes;
    int model_index = 0;
    int last_link_index = -2;
    int i = 0;
    for (;i < num_visualshape; ++i)
    {
        int link_id = visualShapeInfo.m_visualShapeData[i].m_linkIndex;

        if(link_id < -1 || link_id > (int)robot->other_parts_.size())
            assert(0);

        if(link_id == last_link_index)
            model_index++;
        else
            model_index = 0;

        last_link_index = link_id;

        Object * part = nullptr;

        if(link_id == -1) 
            part = robot->root_part_;
        else 
            part = robot->other_parts_[link_id];
        
        std::string filename = visualShapeInfo.m_visualShapeData[i].m_meshAssetFileName;
        int geometry_type = visualShapeInfo.m_visualShapeData[i].m_visualGeometryType;

        if(filename == "")
            filename = std::to_string(geometry_type);

        bool reset = true;
        ModelData* model_data = FindInCache(filename, part->model_list_, reset);

        glm::vec4 color;
        color[0] = visualShapeInfo.m_visualShapeData[i].m_rgbaColor[0];
        color[1] = visualShapeInfo.m_visualShapeData[i].m_rgbaColor[1];
        color[2] = visualShapeInfo.m_visualShapeData[i].m_rgbaColor[2];
        color[3] = visualShapeInfo.m_visualShapeData[i].m_rgbaColor[3];
        
        glm::vec3 local_scale;
        local_scale[0] = visualShapeInfo.m_visualShapeData[i].m_dimensions[0];
        local_scale[1] = visualShapeInfo.m_visualShapeData[i].m_dimensions[1];
        local_scale[2] = visualShapeInfo.m_visualShapeData[i].m_dimensions[2];

        btVector3 position;
        position[0] = visualShapeInfo.m_visualShapeData[i].m_localVisualFrame[0];
        position[1] = visualShapeInfo.m_visualShapeData[i].m_localVisualFrame[1];
        position[2] = visualShapeInfo.m_visualShapeData[i].m_localVisualFrame[2];
        
        btQuaternion rotation;
        rotation[0] = visualShapeInfo.m_visualShapeData[i].m_localVisualFrame[3];
        rotation[1] = visualShapeInfo.m_visualShapeData[i].m_localVisualFrame[4];
        rotation[2] = visualShapeInfo.m_visualShapeData[i].m_localVisualFrame[5];
        rotation[3] = visualShapeInfo.m_visualShapeData[i].m_localVisualFrame[6];

        glm::mat4 transform = TransformToMat4(btTransform(rotation, position));

        OriginTransformation * origin_transform = new OriginTransformation();
        origin_transform->scale = scale;
        origin_transform->local_scale = local_scale;
        origin_transform->origin = transform;
        part->transform_list_.push_back(origin_transform);
        
        float ex = local_scale[0];
        float ey = local_scale[1];
        float ez = local_scale[2];

        if(reset){
            switch(geometry_type) {
                case 5: {
                    model_data->primitive_type_ = kMesh;
                    model_data->directory_ = filename;
                    model_data->Reset();
                    break;
                }
                case 2: {
                    model_data->primitive_type_ = kSphere;
                    origin_transform->local_scale = vec3(ex);
                    model_data->sphere_ = new Sphere(1);
                    model_data->Reset();
                    break;
                }
                case 3: {
                    model_data->primitive_type_ = kBox;
                    origin_transform->local_scale = vec3(ex, ey, ez);
                    model_data->box_ = new Box(1, 1, 1);
                    model_data->Reset();
                    break;
                }
                case 4: {
                    model_data->primitive_type_ = kCylinder;
                    origin_transform->local_scale = vec3(ey, ey, ex);
                    model_data->cylinder_ = new Cylinder(1, 1);
                    model_data->Reset();
                    break;
                }
                case 7: {
                    model_data->primitive_type_ = kCapsule;
                    origin_transform->local_scale = vec3(ey, ey, ex);
                    model_data->cylinder_ = new Cylinder(1, 1);
                    model_data->Reset();
                    break;
                }
                default: {
                    printf("Ignore unsuppoted URDF GEOM!\n");
                    break;
                }
            }
            
        } else {
            switch(geometry_type) {
                case 5: {
                    break;
                }
                case 2: {
                    model_data->primitive_type_ = kSphere;
                    origin_transform->local_scale = vec3(ex);
                    break;
                }
                case 3: {
                    model_data->primitive_type_ = kBox;
                    origin_transform->local_scale = vec3(ex, ey, ez);
                    break;
                }
                case 4: {
                    model_data->primitive_type_ = kCylinder;
                    origin_transform->local_scale = vec3(ey, ey, ex);
                    break;
                }
                case 7: {
                    model_data->primitive_type_ = kCapsule;
                    origin_transform->local_scale = vec3(ey, ey, ex);
                    break;
                }
                default: {
                    printf("Ignore unsuppoted URDF GEOM!\n");
                    break;
                }
            }
        }
        part->bullet_link_id_ = link_id;
        
    }
    
}

ModelData* World::FindInCache(const std::string &key, std::vector<ModelData *> &model_list, bool& reset)
{
    auto has_find = model_cache_.find(key);
    if (has_find != model_cache_.end())
    {
        ModelData* model_data = has_find->second;
        model_list.push_back(model_data);
        reset = false;
        if(model_data) return model_data;
    }
    else
    {
        ModelData * model_data  = new ModelData();
        model_list.push_back(model_data);
        model_cache_[key] = model_data;
        return model_data;
    }
    return nullptr;
}

void World::ClearCache()
{
    for (auto index = model_cache_.begin(); index != model_cache_.end(); index++)
        delete index->second;

    model_cache_.clear();
}

const RenderBody* World::render_body_ptr(const size_t i) const {
    assert(i < robot_list_.size());
    return static_cast<const RenderBody*>(robot_list_[i]);
}

RenderBody* World::render_body_ptr(const size_t i) {
    assert(i < robot_list_.size());
    return static_cast<RenderBody*>(robot_list_[i]);
}

}
