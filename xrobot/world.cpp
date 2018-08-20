#include "world.h"

namespace xrobot {

Joint::Joint() :
bullet_robot_(nullptr), bullet_world_(nullptr),
joint_name_(), joint_type_(NULL), 
bullet_joint_id_(-1), bullet_q_index_(-1), bullet_u_index_(-1),
joint_current_position_(0), joint_current_speed_(0),
joint_limit_1_(-1), joint_limit_2_(-2),
joint_max_force_(1.0f), joint_max_velocity_(1.0f),
joint_has_limits_(false) {}

Joint::~Joint() {}

void Joint::SetJointMotorControlTorque(const float torque)
{
    if(!bullet_robot_ || !bullet_world_) return;

    CommandHandle cmd_handle = b3JointControlCommandInit2(
        bullet_world_->client_,
        bullet_robot_->bullet_handle_,
        kTorque
    );

    b3JointControlSetDesiredForceTorque(cmd_handle, bullet_u_index_, torque);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle);
}

void Joint::SetJointMotorControlVelocity(
    const float speed,
    const float k_d,
    const float max_force)
{
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

void Joint::ResetJointState(
        const float pos,
        const float vel)
{
    if(!bullet_robot_ || !bullet_world_) return;

    CommandHandle cmd_handle = b3CreatePoseCommandInit(
        bullet_world_->client_, bullet_robot_->bullet_handle_);
    b3CreatePoseCommandSetJointPosition(bullet_world_->client_, cmd_handle, bullet_joint_id_, pos);
    b3CreatePoseCommandSetJointVelocity(bullet_world_->client_, cmd_handle, bullet_joint_id_, vel);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle);
}

Object::Object() :
bullet_world_(nullptr), object_name_(), model_list_(0), transformation_list_(0),
bullet_handle_(-1), bullet_link_id_(-1), object_mass_original_(0)
{
    object_position_.setIdentity();
    object_link_position_.setIdentity();
    object_local_inertial_frame_.setIdentity();
    object_speed_ = btVector3(0,0,0);
    object_angular_speed_ = btVector3(0,0,0);
}

Object::~Object()
{
    for (unsigned int i = 0; i < transformation_list_.size(); ++i)
    {
        delete transformation_list_[i];
    }
    transformation_list_.clear();
}

void Object::GetMass(float& mass)
{
    if(!bullet_world_ || bullet_handle_ < 0) return;

    CommandHandle cmd_handle = b3GetDynamicsInfoCommandInit(bullet_world_->client_, bullet_handle_, bullet_link_id_);
    StatusHandle status_handle = b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle);

    int status_type = b3GetStatusType(status_handle);
    if (status_type != CMD_GET_DYNAMICS_INFO_COMPLETED)
    {
        printf("Get Mass Failed!\n");
        return;
    }

    struct b3DynamicsInfo dynamics_info;
    b3GetDynamicsInfo(status_handle, &dynamics_info);

    mass = static_cast<float>(dynamics_info.m_mass);
}

void Object::GetAABB(glm::vec3& aabb_min, glm::vec3& aabb_max)
{
    if(!bullet_world_ || bullet_handle_ < 0) return;

    CommandHandle cmd_handle = b3RequestCollisionInfoCommandInit(bullet_world_->client_, bullet_handle_);
    StatusHandle status_handle = b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle);

    int status_type = b3GetStatusType(status_handle);
    if (status_type != CMD_REQUEST_COLLISION_INFO_COMPLETED)
    {
        printf("Get AABB Failed!\n");
        return;
    }

    double aabb_min_temp[3];
    double aabb_max_temp[3];
    b3GetStatusAABB(status_handle, bullet_link_id_, aabb_min_temp, aabb_max_temp);

    aabb_min = vec3(aabb_min_temp[0], aabb_min_temp[1], aabb_min_temp[2]);
    aabb_max = vec3(aabb_max_temp[0], aabb_max_temp[1], aabb_max_temp[2]);
}

void Object::ApplyForce(const float x, const float y, const float z, const int flags)
{
    if(!bullet_world_ || bullet_handle_ < 0) return;

    printf("Apply Force Has Not Been Implemented Yet!\n");
}

void Object::ApplyTorque(const float x, const float y, const float z, const int flags)
{
    if(!bullet_world_ || bullet_handle_ < 0) return;

    CommandHandle cmd_handle = b3ApplyExternalForceCommandInit(bullet_world_->client_);

    double torque_temp[3];
    torque_temp[0] = x;
    torque_temp[1] = y;
    torque_temp[2] = z;

    b3ApplyExternalTorque(cmd_handle, bullet_handle_, bullet_link_id_, torque_temp, flags);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle);
}

void Object::EnableSleeping()
{
    if(!bullet_world_ || bullet_handle_ < 0) return;

    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(bullet_world_->client_);

    b3ChangeDynamicsInfoSetActivationState(cmd_handle, bullet_handle_, eActivationStateEnableSleeping);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle); 
}

void Object::DisableSleeping()
{
    if(!bullet_world_ || bullet_handle_ < 0) return;

    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(bullet_world_->client_);

    b3ChangeDynamicsInfoSetActivationState(cmd_handle, bullet_handle_, eActivationStateDisableSleeping);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle); 
}

void Object::Sleep()
{
    if(!bullet_world_ || bullet_handle_ < 0) return;

    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(bullet_world_->client_);

    b3ChangeDynamicsInfoSetActivationState(cmd_handle, bullet_handle_, eActivationStateSleep);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle);
}

void Object::Wake()
{
    if(!bullet_world_ || bullet_handle_ < 0) return;

    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(bullet_world_->client_);

    b3ChangeDynamicsInfoSetActivationState(cmd_handle, bullet_handle_, eActivationStateWakeUp);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle);
}

void Object::ChangeMass(const float mass)
{
    if(!bullet_world_ || bullet_handle_ < 0) return;
    if(mass < 0) return;

    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(bullet_world_->client_);

    b3ChangeDynamicsInfoSetMass(cmd_handle, bullet_handle_, bullet_link_id_, mass);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle);
}

void Object::ChangeLinearDamping(const float damping)
{
    if(!bullet_world_ || bullet_handle_ < 0) return;
    if(damping < 0) return;

    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(bullet_world_->client_);

    b3ChangeDynamicsInfoSetLinearDamping(cmd_handle, bullet_handle_, damping);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle);
}

void Object::ChangeAngularDamping(const float damping)
{
    if(!bullet_world_ || bullet_handle_ < 0) return;
    if(damping < 0) return;

    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(bullet_world_->client_);

    b3ChangeDynamicsInfoSetAngularDamping(cmd_handle, bullet_handle_, damping);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle);
}


void Object::ChangeLateralFriction(const float friction)
{
    if(!bullet_world_ || bullet_handle_ < 0) return;
    if(friction < 0) return;

    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(bullet_world_->client_);

    b3ChangeDynamicsInfoSetLateralFriction(cmd_handle, bullet_handle_, bullet_link_id_, friction);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle);
}

void Object::ChangeSpinningFriction(const float friction)
{
    if(!bullet_world_ || bullet_handle_ < 0) return;
    if(friction < 0) return;

    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(bullet_world_->client_);

    b3ChangeDynamicsInfoSetSpinningFriction(cmd_handle, bullet_handle_, bullet_link_id_, friction);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle);
}

void Object::ChangeRollingFriction(float friction)
{
    if(!bullet_world_ || bullet_handle_ < 0) return;
    if(friction < 0) return;

    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(bullet_world_->client_);

    b3ChangeDynamicsInfoSetRollingFriction(cmd_handle, bullet_handle_, bullet_link_id_, friction);
    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, cmd_handle);
}

void Object::SetMassOriginal(const float mass)
{
    if(mass < 0) return;

    object_mass_original_ = mass;
}

float Object::GetMassOriginal()
{
    return object_mass_original_;
}

void Robot::CalculateInverseKinematics(
    const int end_index, 
    const btVector3 target_position,
    const btQuaternion target_orientation,
    double* joint_damping,
    double* ik_output_joint_pos,
    int &num_poses)
{
    const int solver = 0;
    const int num_joints = b3GetNumJoints(bullet_world_->client_, bullet_handle_);
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

    b3CalculateInverseKinematicsAddTargetPositionWithOrientation(cmd_handle,
        end_index, position_temp, orientation_temp);
    b3CalculateInverseKinematicsSetJointDamping(cmd_handle, dof, joint_damping);

    StatusHandle status_handle = b3SubmitClientCommandAndWaitStatus(bullet_world_->client_,
        cmd_handle);

    int result_body_index;
    int result = b3GetStatusInverseKinematicsJointPositions(status_handle,
        &result_body_index, &num_poses, 0);

    if(result && num_poses)
    {
        result = b3GetStatusInverseKinematicsJointPositions(status_handle,
            &result_body_index, &num_poses, ik_output_joint_pos);
    }
}

void Robot::RemoveRobotFromBullet()
{
    if(bullet_handle_ < 0) return;

    b3SubmitClientCommandAndWaitStatus(bullet_world_->client_, 
        b3InitRemoveBodyCommand(bullet_world_->client_,bullet_handle_)
    );
}

void Robot::DisableSleeping()
{
    if(bullet_handle_ < 0) return;

    if(root_part_)
        root_part_->DisableSleeping();

    for (Object * part : other_parts_list_)
    {
       if(part)
            part->DisableSleeping();
    }
}


Robot::Robot() :
bullet_world_(nullptr), urdf_name_("robot_unnamed"), bullet_handle_(-1), 
root_part_(nullptr), other_parts_list_(0), joints_list_(0), recycle_(false) {}

Robot::~Robot()
{
    for (unsigned int i = 0; i < other_parts_list_.size(); ++i)
    {
        delete other_parts_list_[i];
    }

    for (unsigned int i = 0; i < joints_list_.size(); ++i)
    {
        delete joints_list_[i];
    }

    if(root_part_) delete root_part_;
}

World::World() : camera_list_(0), attached_camera_to_robot_map_(),
robot_list_(0), bullet_handle_to_robot_map_(), client_(0), 
recycle_robot_map_(), model_cache_(), bullet_gravity_(0), bullet_timestep_(0),
bullet_timestep_sent_(0), bullet_skip_frames_sent_(0), bullet_ts_(0) {}

World::~World()
{
    CleanEverything();
    ClearCache();
    b3DisconnectSharedMemory(client_);
}

void World::ResetSimulation()
{
    b3SubmitClientCommandAndWaitStatus(
        client_, b3InitResetSimulationCommand(client_));
}

void World::RemoveRobot(Robot * robot)
{
    if(!robot) return;

    auto index = std::find(robot_list_.begin(), robot_list_.end(), robot);
    if(index != robot_list_.end())
    {
        int i = index - robot_list_.begin();
        Robot * robot = robot_list_[i];

        robot->root_part_->Sleep();
        for (int i = 0; i < robot->other_parts_list_.size(); ++i)
        {
            if(robot->other_parts_list_[i])
                robot->other_parts_list_[i]->Sleep();
        }

        delete robot_list_[i];
        robot_list_[i] = nullptr;
        robot_list_.erase(index);        
    }

    bullet_handle_to_robot_map_[robot->bullet_handle_] = nullptr;
}

void World::RemoveRobot2(Robot * robot)
{
    robot->recycle_ = true;

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
    for (Object * part : robot->other_parts_list_)
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
}

void World::CleanEverything2()
{
    for (Robot * robot : robot_list_)
    {
        if(!robot->recycle_)
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

    for(unsigned int i = 0; i < camera_list_.size(); ++i)
    {
        delete camera_list_[i];
    }
    camera_list_.clear();
    attached_camera_to_robot_map_.clear();
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
        if(robot_list_[i])
            delete robot_list_[i];
    }
    recycle_robot_map_.clear();
    robot_list_.clear();
    bullet_handle_to_robot_map_.clear();
    ResetSimulation();
}

Camera* World::AddCamera(const vec3 position, const vec3 offset, 
    const float aspect_ratio, const float fov, const float near, const float far)
{
    Camera * camera = new Camera(position);
    camera->Near = near;
    camera->Far = far;
    camera->Zoom = fov;
    camera->Offset = offset;
    camera->Aspect = aspect_ratio;
    camera_list_.push_back(camera);
    return camera;
}

void World::AttachCamera(Camera * camera, Robot * robot)
{
    if(!camera) return;
    attached_camera_to_robot_map_[camera] = robot;
}

void World::DeattachCamera(Camera * camera)
{
    if(!camera) return;
    attached_camera_to_robot_map_[camera] = nullptr;
}

void World::RemoveCamera(Camera * camera)
{
    if(!camera) return;
    auto index = std::find(camera_list_.begin(), camera_list_.end(), camera);
    if(index != camera_list_.end())
    {
        int i = index - camera_list_.begin();
        delete camera_list_[i];
        camera_list_[i] = nullptr;
        camera_list_.erase(index);
    }
    attached_camera_to_robot_map_.erase(camera);
}

void World::AttachCameraToRoot(Camera * camera, Robot * robot)
{
    if(!camera || !robot || !robot->root_part_) return;

    Object* root_part = robot->root_part_;
    btTransform position = root_part->object_position_;
    btTransform inertial_frame = root_part->object_local_inertial_frame_;

    auto base_position = position.getOrigin();
    auto base_orentation = position.getBasis();
    btVector3 base_front = base_orentation * btVector3(1, 0, 0); 

    vec3 offset = camera->Offset;
    camera->Front = glm::normalize(vec3(base_front[0], base_front[1], base_front[2]));
    camera->Right = glm::normalize(glm::cross(camera->Front, vec3(0,-1,0)));
    camera->Up    = glm::normalize(glm::cross(camera->Front,camera->Right));
    vec3 camera_aim(camera->Front * offset.x + camera->Up * offset.y + camera->Right * offset.z);

    btMatrix3x3 pre_orientation;
    pre_orientation.setIdentity();
    pre_orientation.setEulerYPR(0,glm::radians(camera->Pre_Pitch),0);

    base_front = base_orentation * pre_orientation * btVector3(1, 0, 0); 
    camera->Front = glm::normalize(vec3(base_front[0], base_front[1], base_front[2]));
    camera->Right = glm::normalize(glm::cross(camera->Front, vec3(0,-1,0)));
    camera->Up    = glm::normalize(glm::cross(camera->Front,camera->Right));
    camera->Position = vec3(base_position[0], base_position[1], base_position[2]) + camera_aim;
}

void World::RotateCamera(Camera * camera, const float pitch)
{
    if(!camera) return;
    camera->Pre_Pitch = pitch;
}

void World::BulletStep(const int skip_frames)
{
    float need_timestep = bullet_timestep_ * skip_frames;
    if (bullet_timestep_sent_ != need_timestep ||
        bullet_skip_frames_sent_ != skip_frames) {
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

    for (auto index = attached_camera_to_robot_map_.begin(); index != attached_camera_to_robot_map_.end(); index++)
    {
        auto camera = index->first;
        auto robot  = index->second;
        if(!camera || !robot) continue;
        AttachCameraToRoot(camera, robot);
    }
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

    for (Object* part : robot->other_parts_list_)
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
    const Robot* robot,
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
    const Robot* robot,
    const btTransform& tranform
)
{
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

int World::RayTest(const vec3 ray_from_position, const vec3 ray_to_position)
{
    CommandHandle cmd_handle = b3CreateRaycastBatchCommandInit(client_);
    b3RaycastBatchSetNumThreads(cmd_handle, 8);

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

Robot* World::LoadURDF(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation,
        const float scale,
        const bool fixed_base,
        const bool self_collision,
        const bool use_multibody)
{
    Robot * robot = new Robot();
    robot->bullet_world_ = this;
    robot->urdf_name_ = filename;
   
    CommandHandle cmd_handle = b3LoadUrdfCommandInit(client_, filename.c_str());
    
    b3LoadUrdfCommandSetStartPosition(cmd_handle, position[0], position[1], position[2]);
    b3LoadUrdfCommandSetStartOrientation(cmd_handle, rotation[0], rotation[1],
                                         rotation[2], rotation[3]);

    b3LoadUrdfCommandSetUseFixedBase(cmd_handle, fixed_base);
    b3LoadUrdfCommandSetGlobalScaling(cmd_handle, scale);
    b3LoadUrdfCommandSetUseMultiBody(cmd_handle, use_multibody);

    if (self_collision) {
        b3LoadUrdfCommandSetFlags(cmd_handle, kURDFSelfCollision | kURDFSelfCollisionExParents);
    }
    else 
    {
        b3LoadUrdfCommandSetFlags(cmd_handle, 0);
    }

    StatusHandle status_handle = b3SubmitClientCommandAndWaitStatus(client_, cmd_handle);
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
    return robot;
}

Robot* World::LoadURDF2(
    const std::string& filename,
    const btVector3 position,
    const btQuaternion rotation,
    const float scale,
    const bool fixed_base)
{
    Robot * robot = nullptr;

    if(recycle_robot_map_.find(filename) != recycle_robot_map_.end()
        && !recycle_robot_map_[filename].empty())
    {

        int recycled_robot_size = recycle_robot_map_[filename].size();

        robot = recycle_robot_map_[filename][recycled_robot_size - 1];
        recycle_robot_map_[filename].pop_back();
        robot->recycle_ = false;

        btTransform transform;
        transform.setIdentity();
        transform.setOrigin(position);
        transform.setRotation(rotation);

        SetTransformation(robot, transform);
        SetVelocity(robot, btVector3(0,0,0));

        float mass;
        robot->root_part_->ChangeMass(robot->root_part_->GetMassOriginal());

        for (Joint * joint : robot->joints_list_)
        {
            if(joint)
                joint->ResetJointState(0.0f, 0.005f);
        }

        for (Object * part : robot->other_parts_list_)
        {
            part->ChangeMass(part->GetMassOriginal());
        }


    }
    else
    {
        robot = LoadURDF(
            filename,
            position,
            rotation,
            scale,
            fixed_base,
            false,
            true
        );
        robot->recycle_ = false;
    }
    return robot;
}


Robot* World::LoadOBJ2(
    const std::string& filename,
    const btVector3 position,
    const btQuaternion rotation,
    const btVector3 scale,
    const bool flip,
    const bool concave)
{
    Robot * robot = nullptr;

    std::string filename_with_scale = filename + ":" + 
        to_string(scale[0]) + ":" + to_string(scale[1]) + ":" + to_string(scale[2]);

    if(recycle_robot_map_.find(filename_with_scale) != recycle_robot_map_.end()
        && !recycle_robot_map_[filename_with_scale].empty())
    {
        robot = recycle_robot_map_[filename_with_scale].back();
        recycle_robot_map_[filename_with_scale].pop_back();

        robot->recycle_ = false;

        btTransform transform;
        transform.setIdentity();
        transform.setOrigin(position);
        transform.setRotation(rotation);

        SetTransformation(robot, transform);
    }
    else
    {
        robot = LoadOBJ(
            filename,
            position,
            rotation,
            scale,
            flip,
            concave
        );
    }
    return robot;
}


Robot* World::LoadOBJ(
    const std::string& filename,
    const btVector3 position,
    const btQuaternion rotation,
    const btVector3 scale,
    const bool flip,
    const bool concave
)
{
    Robot * robot = new Robot();
    robot->bullet_world_ = this;
    robot->urdf_name_ = filename + ":" + 
        to_string(scale[0]) + ":" + to_string(scale[1]) + ":" + to_string(scale[2]);
    robot->recycle_ = false;

    CommandHandle cmd_handle = b3LoadObjCommandInit(client_, filename.c_str());

    if(concave)
        b3LoadObjCommandSetFlags(cmd_handle, kOBJConcave);

    b3LoadObjCommandSetStartPosition(cmd_handle, position[0], position[1], position[2]);
    b3LoadObjCommandSetStartOrientation(cmd_handle, rotation[0], rotation[1], rotation[2], rotation[3]);
    b3LoadObjCommandSetStartScale(cmd_handle, scale[0], scale[1], scale[2]);
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

    if(flip)
        origin_transform->flip = -1.0f;

    robot->root_part_->transformation_list_.push_back(origin_transform);

    if(reset)
    {
        model_data->primitive_type_ = kMesh;
        model_data->directory_ = filename;
        model_data->Reset();
    }

    robot_list_.push_back(robot);
    bullet_handle_to_robot_map_[robot->bullet_handle_] = robot;

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
    robot->other_parts_list_.resize(num_joints);

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

        Object * part = robot->other_parts_list_[c];
        part = new Object();
        part->bullet_handle_ = robot->bullet_handle_;
        part->bullet_link_id_ = c;
        part->bullet_world_ = this;
        part->object_name_ = info.m_linkName;
        robot->other_parts_list_[c] = part;
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

        if(link_id < -1 || link_id > (int)robot->other_parts_list_.size())
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
            part = robot->other_parts_list_[link_id];
        
        std::string filename = visualShapeInfo.m_visualShapeData[i].m_meshAssetFileName;
        int geometry_type = visualShapeInfo.m_visualShapeData[i].m_visualGeometryType;

        if(filename == "")
            filename = to_string(geometry_type);

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
        part->transformation_list_.push_back(origin_transform);
        
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

}