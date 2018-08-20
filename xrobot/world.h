#ifndef WORLD_H_
#define WORLD_H_

#include <map>
#include <string>
#include <vector>
#include <unistd.h>
#include <utility>

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/quaternion.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "bullet/LinearMath/btTransform.h"
#include "bullet/PhysicsClientC_API.h"

#include "render_engine/camera.h"
#include "render_engine/model.h"
#include "render_engine/render_world.h"

// TODO:
// - too many public attributes
// - move Camera parts of World to RenderWorld
// - what happens to Camera attached to a deleted Robot

namespace xrobot {

constexpr int kCacheSize = 32;

typedef b3SharedMemoryCommandHandle CommandHandle;
typedef b3SharedMemoryStatusHandle StatusHandle;

enum SleepState
{
    kSleep = eActivationStateSleep,
    kWakeUp = eActivationStateWakeUp,
    kDisableSleeping = eActivationStateDisableSleeping,
    kEnableSleeping = eActivationStateEnableSleeping
};

enum ComputeState
{
    kComputeVelocity = ACTUAL_STATE_COMPUTE_LINKVELOCITY
};

enum ControlModes
{
    kTorque = CONTROL_MODE_TORQUE,
    kVelocity = CONTROL_MODE_VELOCITY,
    kPositionVelocity = CONTROL_MODE_POSITION_VELOCITY_PD
};

enum ConstaintTypes
{
    kFixed = eFixedType,
    kRevolute = eRevoluteType,
    kPrismatic = ePrismaticType
};

enum JointTypes 
{
    kRotationMotor,
    kLinearMotor
};

enum LoadingFlags
{
    kOBJConcave = FORCE_CONCAVE,
    kURDFSelfCollision = URDF_USE_SELF_COLLISION,
    kURDFSelfCollisionExParents = URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
};

inline glm::mat4 TransformToMat4(const btTransform& transform) 
{
    btScalar mat[16];
    transform.getOpenGLMatrix(mat);
    return glm::make_mat4(mat);
}

inline btTransform TransformFromDoubles(const double* position, const double* orientation) {
    btVector3 position_temp;
    btQuaternion orientation_temp;
    {
        position_temp[0] = position[0];
        position_temp[1] = position[1];
        position_temp[2] = position[2];
        orientation_temp[0] = orientation[0];
        orientation_temp[1] = orientation[1];
        orientation_temp[2] = orientation[2];
        orientation_temp[3] = orientation[3];
    }
    return btTransform(orientation_temp, position_temp);
}

class Robot;
class World;

class Joint {
public:
    Joint();
    ~Joint();

    void ResetJointState(const float pos, const float vel);
    void SetJointMotorControlTorque(const float torque);
    void SetJointMotorControlVelocity(const float speed,
        const float k_d, const float max_force);
    void SetJointMotorControlPosition(const float target,
        const float k_p, const float k_d, const float max_force);

    Robot * bullet_robot_;
    World * bullet_world_;
    std::string joint_name_;
    int joint_type_;
    int bullet_joint_id_;
    int bullet_q_index_;
    int bullet_u_index_;
    float joint_current_position_;
    float joint_current_speed_;
    float joint_limit_1_;
    float joint_limit_2_;
    float joint_max_force_;
    float joint_max_velocity_;
    bool joint_has_limits_;
};

class Object : public render_engine::RenderPart {
public:
    Object();
    ~Object();

    void Sleep();
    void Wake();
    void EnableSleeping();
    void DisableSleeping();
    void ChangeMass(const float mass);
    void ChangeLinearDamping(const float damping);
    void ChangeAngularDamping(const float damping);
    void ChangeLateralFriction(const float friction);
    void ChangeSpinningFriction(const float friction);
    void ChangeRollingFriction(const float friction);
    void ApplyForce(const float x, const float y, const float z,
                    const int flags = EF_LINK_FRAME);
    void ApplyTorque(const float x, const float y, const float z, 
                     const int flags = EF_LINK_FRAME);
    void GetMass(float& mass);
    float GetMassOriginal();
    void SetMassOriginal(const float mass);

public:
    World * bullet_world_;
    std::string object_name_;
    
    int bullet_handle_;
    int bullet_link_id_;

    btTransform object_position_;
    btTransform object_link_position_;
    btTransform object_local_inertial_frame_;
    btVector3   object_speed_;
    btVector3   object_angular_speed_;

protected:
    float object_mass_original_;

// xrobot::render_engine::RenderPart
public:
    int id() const override { return bullet_handle_; }
    void GetAABB(glm::vec3& aabb_min, glm::vec3& aabb_max) override;
    glm::mat4 position() const override;
    glm::mat4 local_inertial_frame() const override;
};

class Robot : public render_engine::RenderBody {
    using RenderPart = render_engine::RenderPart;
public:
    Robot();

    ~Robot();

    void DisableSleeping();

    void RemoveRobotFromBullet();

    void CalculateInverseKinematics(
            const int end_index, 
            const btVector3 target_position,
            const btQuaternion target_orientation,
            double* joint_damping,
            double* ik_output_joint_pos,
            int &num_poses);

    World* bullet_world_;
    std::string urdf_name_;
    int bullet_handle_;
    Object* root_part_;
    std::vector<Object*> other_parts_;
    std::vector<Joint *> joints_list_;

// xrobot::render_engine::RenderBody
public:
    size_t size() const override { return other_parts_.size(); }
    RenderPart* render_root_ptr() override;
    const RenderPart* render_root_ptr() const override;
    const RenderPart* render_part_ptr(const size_t i) const override;
    RenderPart* render_part_ptr(const size_t i) override;
};

class World : public render_engine::RenderWorld {
    using Camera = xrobot::render_engine::Camera;
    using RenderBody = render_engine::RenderBody;
public:
    World();
    ~World();

    b3PhysicsClientHandle client_;
    std::vector<Robot*> robot_list_;
    std::map<Camera*, Robot*> attached_camera_to_robot_map_;
    std::map<int, Robot*> bullet_handle_to_robot_map_;
    std::map<std::string, std::vector<Robot *>> recycle_robot_map_;
    std::map<std::string, render_engine::ModelData *> model_cache_;

    float bullet_gravity_;
    float bullet_timestep_;
    float bullet_timestep_sent_;
    float bullet_skip_frames_sent_;
    double bullet_ts_;

    render_engine::ModelData * FindInCache(const std::string &key,
        std::vector<render_engine::ModelData*> &model_list, bool& reset);
    void ClearCache();
    void PrintCacheInfo();
    void BulletInit(const float gravity = 9.81f, const float timestep = 1.0/100.0);
    void BulletStep(const int skip_frames = 1);
    void QueryPositions();
    void QueryPosition(const Robot* robot);
    void LoadRobotShape(Robot* robot, const float scale);
    void LoadRobotJoint(Robot* robot, const std::string &filename);
    void RemoveRobot(Robot* robot);
    void RemoveRobot2(Robot* robot);
    void CleanEverything();
    void CleanEverything2();
    void ResetSimulation();
    Camera* AddCamera(glm::vec3 position,
                      glm::vec3 offset = glm::vec3(0),
                      float aspect_ratio = 4.0f / 3.0f,
                      float fov = 90.0f,
                      float near = 0.01f,
                      float far = 10.0f);
    void AttachCameraToRoot(Camera* camera, Robot* robot);
    void AttachCamera(Camera* camera, Robot* robot);
    void DeattachCamera(Camera* camera);
    void RemoveCamera(Camera* camera);
    void RotateCamera(Camera* camera, const float pitch);
    int RayTest(const glm::vec3 ray_from_position, const glm::vec3 ray_to_position);
    void SetTransformation(const Robot* robot, const btTransform& tranform);
    void SetVelocity(const Robot* robot, const btVector3& velocity);

    int CreateFixedRootToTargetConstraint(Robot* parent,
        const btVector3& parent_relative_position,
        const btVector3& child_relative_position,
        const btQuaternion& parent_relative_orientation = btQuaternion(),
        const btQuaternion& child_relative_orientation = btQuaternion()
    );

    void ChangeFixedRootToTargetConstraint(const int constraint_id,
        const btVector3& child_relative_position,
        const btQuaternion& child_relative_orientation,
        const float max_force = 500.0f
    );

    Robot* LoadURDF2(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation,
        const float scale = 1.0f,
        const bool fixed_base = false
    );

    Robot* LoadURDF(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation,
        const float scale = 1.0f,
        const bool fixed_base = false,
        const bool self_collision = false,
        const bool use_multibody = true
    );

    Robot* LoadOBJ2(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation,
        const btVector3 scale,
        const bool flip = false,
        const bool concave = false
    );


    Robot* LoadOBJ(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation,
        const btVector3 scale,
        const bool flip = false,
        const bool concave = false
    );
    
public:
    size_t size() const override { return robot_list_.size(); }
    const RenderBody* render_body_ptr(const size_t i) const override;
    RenderBody* render_body_ptr(const size_t i) override;
};

}

#endif // WORLD_H_