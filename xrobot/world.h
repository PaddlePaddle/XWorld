#ifndef WORLD_H_
#define WORLD_H_

#include <map>
#include <string>
#include <functional>
#include <vector>
#include <unordered_map>
#include <queue>
#include <unistd.h>
#include <utility>
#include <chrono>
#include <thread>

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/vector_angle.hpp"
#include "glm/gtx/quaternion.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "bullet/LinearMath/btTransform.h"
#include "bullet/PhysicsClientC_API.h"

#include "render_engine/model.h"
#include "render_engine/render_world.h"

#include "inventory.h"
#include "vendor/json.h"

namespace xrobot {

class Inventory;

constexpr int kCacheSize = 32;
const glm::vec3 kVec3Zero = glm::vec3(0); 
const glm::vec3 kVec3Up = glm::vec3(0,1,0);
const glm::vec3 kVec3Front = glm::vec3(1,0,0);
const glm::vec3 kVec3Right = glm::vec3(0,0,1);

typedef b3SharedMemoryCommandHandle CommandHandle;
typedef b3SharedMemoryStatusHandle StatusHandle;

enum SleepState
{
    kSleep = eActivationStateSleep,
    kWakeUp = eActivationStateWakeUp,
    kDisableSleeping = eActivationStateDisableSleeping,
    kEnableSleeping = eActivationStateEnableSleeping,
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

// http://www.opengl-tutorial.org
// Rotation
inline glm::quat RotationBetweenVectors(const glm::vec3 start, const glm::vec3 dest)
{
    glm::vec3 start_norm = glm::normalize(start);
    glm::vec3 dest_norm = glm::normalize(dest);

    float cosTheta = glm::dot(start_norm, dest_norm);
    glm::vec3 rotationAxis;

    if (cosTheta < -1 + 0.0001f){
        rotationAxis = glm::cross(glm::vec3(0.0f, 0.0f, 1.0f), start_norm);
        if (glm::length2(rotationAxis) < 0.001f)
            rotationAxis = glm::cross(glm::vec3(1.0f, 0.0f, 0.0f), start_norm);

        rotationAxis = glm::normalize(rotationAxis);
        return glm::angleAxis(glm::radians(180.0f), rotationAxis);
    }

    // Implementation from Stan Melax's Game Programming Gems 1 article
    rotationAxis = glm::cross(start_norm, dest_norm);

    float s = sqrt( (1+cosTheta)*2 );
    float invs = 1 / s;

    return glm::quat(
        s * 0.5f, 
        rotationAxis.x * invs,
        rotationAxis.y * invs,
        rotationAxis.z * invs
    );
}

inline int GetJsonArrayEntry(Json::Value *&result,
        Json::Value *array, unsigned int k, int expected_type = -1)
{
      // Check array type
  if (array->type() != Json::arrayValue) {
    fprintf(stderr, "JSON: not an array\n");
    return 0;
  }

  // Check array size
  if (array->size() <= k) {
    // fprintf(stderr, "JSON array has no member %d\n", k);
    return 0;
  }

  // Get entry
  result = &((*array)[k]);
  if (result->type() == Json::nullValue) {
    // fprintf(stderr, "JSON array has null member %d\n", k);
    return 0;
  }

  // Check entry type
  if (expected_type > 0) {
    if (result->type() != expected_type) {
      // fprintf(stderr, "JSON array entry %d has unexpected type %d (rather than %d)\n", k, result->type(), expected_type);
      return 0;
    }
  }
  
  // Return success
  return 1;
}

inline int GetJsonObjectMember(Json::Value *&result, Json::Value *object,
        const char *str, int expected_type = 0)
{
    // Check object type
  if (object->type() != Json::objectValue) {
    // fprintf(stderr, "JSON: not an object\n");
    return 0;
  }

  // Check object member
  if (!object->isMember(str)) {
    // fprintf(stderr, "JSON object has no member named %s\n", str);
    return 0;
  }

  // Get object member
  result = &((*object)[str]);
  if (result->type() == Json::nullValue) {
    // fprintf(stderr, "JSON object has null member named %s\n", str);
    return 0;
  }

  // Check member type
  if (expected_type > 0) {
    if (result->type() != expected_type) {
      // fprintf(stderr, "JSON object member %s has unexpected type %d (rather than %d)\n", str, result->type(), expected_type);
      return 0;
    }
  }
  
  // Check for empty strings
  if (result->type() == Json::stringValue) {
    if (result->asString().length() == 0) {
      // fprintf(stderr, "JSON object has zero length string named %s\n", str);
      return 0;
    }
  }

  // Return success
  return 1;
}

class RobotBase;
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

    RobotBase * bullet_robot_;
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

    float object_mass_original_;

// xrobot::render_engine::RenderPart
public:
    int id() const override { return bullet_handle_; }
    void GetAABB(glm::vec3& aabb_min, glm::vec3& aabb_max) override;
    glm::mat4 translation_matrix() const override;
    glm::mat4 local_inertial_frame() const override;
};

class RobotData {
public:
	RobotData();
	RobotData(btVector3 scale, bool fixed, float mass,
			std::string label, std::string urdf_name,
			std::string path, int bullet_handle,
			Object * root_part,
			std::vector<Object*> other_parts,
			std::vector<Joint*> joints_list);

	btVector3 scale_;
    bool fixed_;
    float mass_; // Only for .Obj
    std::string label_;
    std::string urdf_name_;
    std::string path_;
    int bullet_handle_;
    Object* root_part_;
    std::vector<Object*> other_parts_;
    std::vector<Joint *> joints_list_;
};

class RobotBase : public render_engine::RenderBody {
    using RenderPart = render_engine::RenderPart;
public:
    RobotBase(World * bullet_world);

    virtual ~RobotBase();

    virtual bool TakeAction(const int act_id);

    virtual void LoadConvertedObject(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation,
        const float scale = 1.0f,
        const std::string& label = ""
    );
    virtual void LoadAnimatedObject(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation,
        const float scale = 1.0f,
        const std::string& label = ""
    );

    void LoadOBJ(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation,
        const btVector3 scale,
        const std::string& label = "unlabeled",
        const float mass = 0,
        const bool flip = false,
        const bool concave = false
    );

    void LoadURDF(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation,
        const float scale = 1.0f,
        const std::string& label = "unlabeled",
        const bool fixed_base = false
    );

    void LoadURDFFile(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation,
        const float scale = 1.0f,
        const std::string& label = "unlabeled",
        const bool fixed_base = false,
        const bool self_collision = false,
        const bool use_multibody = true
    );


    void LoadOBJFile(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation,
        const btVector3 scale,
        const std::string& label = "unlabeled",
        const float mass = 0,
        const bool flip = false,
        const bool concave = false
    );

    void LoadRobotShape(const float scale);
    void LoadRobotJoint(const std::string &filename);

    void DisableSleeping();

    void Wake();

    void Sleep();

    virtual void RemoveRobotTemp();

    void RemoveRobotFromBullet();

    void SetJointVelocity(const int joint_id, const float speed,
                          const float k_d, const float max_force);

    void SetJointPosition(const int joint_id, const float target,
                          const float k_p, const float k_d,
                          const float max_force);

    void ResetJointState(const int joint_id, const float pos,
                         const float vel);

    virtual void Teleport2(const glm::vec3 walk_move,
                  const glm::vec3 walk_rotate,
                  const float speed, const bool remit = false); 

    virtual void Teleport(const glm::vec3 walk_move,
                  const glm::vec3 walk_rotate,
                  const float speed, const bool remit = false);

    virtual void Freeze(const bool remit = false);
    virtual void MoveForward(const float speed);
    virtual void MoveBackward(const float speed);
    virtual void TurnLeft(const float speed);
    virtual void TurnRight(const float speed);
    virtual void PickUp(Inventory * inventory,
            const glm::vec3 from, const glm::vec3 to);
    virtual void PutDown(Inventory * inventory, 
            const glm::vec3 from, const glm::vec3 to);
    virtual void RotateObject(const float rotate_angle_y,
            const glm::vec3 from, const glm::vec3 to);

    size_t size() const override { return robot_data_.other_parts_.size(); }
    RenderPart* render_root_ptr() override;
    const RenderPart* render_root_ptr() const override;
    const RenderPart* render_part_ptr(const size_t i) const override;
    RenderPart* render_part_ptr(const size_t i) override;
    void attach_camera(const glm::vec3& offset,
                       const float pitch,
                       glm::vec3& loc,
                       glm::vec3& front,
                       glm::vec3& right,
                       glm::vec3& up) override;

    World* bullet_world_;
    RobotData robot_data_;
};

class Robot : public xrobot::RobotBase {
public:
	Robot(World* bullet_world);
	~Robot();

    void CalculateInverseKinematics(
            const int end_index, 
            const btVector3 target_position,
            const btQuaternion target_orientation,
            double* joint_damping,
            double* ik_output_joint_pos,
            int &num_poses);
};

class RobotWithConvertion : public xrobot::RobotBase {
public:
    RobotWithConvertion(World* bullet_world);
    ~RobotWithConvertion();

    void LoadConvertedObject(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation,
        const float scale = 1.0f,
        const std::string& label = ""
    );

    bool TakeAction(const int act_id);
    void SetCycle(const bool cycle) { cycle_ = cycle; }
    void SetStatus(const int status) { status_ = status; }
    void RemoveRobotTemp();

    float scale_;
    std::string label_;
    std::string path_;
    std::vector<std::string> object_path_list_;
    std::vector<std::string> object_name_list_;

private:
    void Remove();

    int status_;
    bool cycle_;
};

class RobotWithAnimation : public xrobot::RobotBase {
public:
    RobotWithAnimation(World* bullet_world);
    ~RobotWithAnimation();

    void LoadAnimatedObject(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation,
        const float scale = 1.0f,
        const std::string& label = ""
    );

    bool TakeAction(const int act_id);
    void SetStatus(const int status) { status_ = status; }
    void SetJoint(const int joint) { joint_ = joint; }
    void RemoveRobotTemp();

    std::string path_;

private:
    int status_;
    int joint_;
    std::map<int, float> positions_; 
};

struct ContactPoint {
    glm::vec3 contact_normal;
    float contact_distance;
    int bullet_id_a;
    int bullet_id_b;
};

struct Ray {
    glm::vec3 from;
    glm::vec3 to;
};

struct RayTestInfo
{
    int bullet_id;
    glm::vec3 pos;
    glm::vec3 norm;
};

struct ObjectAttributes {
    glm::vec3 aabb_min;
    glm::vec3 aabb_max;
    int bullet_id;
};

struct ObjectDirections {
    float dirs[9];
    int bullet_id;
};

class World : public render_engine::RenderWorld {
    using RenderBody = render_engine::RenderBody;
public:
    World();
    ~World();

    b3PhysicsClientHandle client_;
    std::vector<RobotBase*> robot_list_;
    std::map<int, RobotBase*> bullet_handle_to_robot_map_;
    std::map<std::string, std::vector<RobotBase *>> recycle_robot_map_;
    std::map<std::string, render_engine::ModelData *> model_cache_;
    std::map<std::string, std::vector<int>> object_locations_;

    float bullet_gravity_;
    float bullet_timestep_;
    float bullet_timestep_sent_;
    float bullet_skip_frames_sent_;
    double bullet_ts_;
    int reset_count_;

    RobotBase* LoadRobot(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation,
        const btVector3 scale = btVector3(1,1,1),
        const std::string& label = "unlabeled",
        const bool fixed_base = false,
        const float mass = 0,
        const bool flip = false,
        const bool concave = false
    );

    RobotBase* LoadModelFromCache(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation
    );

    render_engine::ModelData * FindInCache(const std::string &key,
        std::vector<render_engine::ModelData*> &model_list, bool& reset);
    void ClearCache();
    void PrintCacheInfo();
    void BulletInit(const float gravity = 9.81f, const float timestep = 1.0/100.0);
    void BulletStep(const int skip_frames = 1);
    void QueryPositions();
    void QueryPosition(const RobotBase* robot);
    void RemoveRobot(RobotBase* robot);
    void CleanEverything();
    void CleanEverything2();
    void ResetSimulation();
    int RayTest(const glm::vec3 ray_from_position, const glm::vec3 ray_to_position);
    void BatchRayTest(const std::vector<Ray> rays, std::vector<RayTestInfo>& result,
        const int num_threads = 0);
    void SetTransformation(RobotBase* robot, const btTransform& tranform);
    void SetVelocity(RobotBase* robot, const btVector3& velocity);

    void GetRootContactPoints(const RobotBase* robot, const Object* part,
        std::vector<ContactPoint>& contact_points);

    int CreateFixedRootToTargetConstraint(RobotBase* parent,
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

    void CharacterMove(RobotBase* robot, const glm::vec3 walk_move,
        const glm::vec3 walk_rotate, const float speed);

    void AddObjectWithLabel(const std::string& label,
        const int id);
    void RemoveObjectWithLabel(const int id);
    void QueryObjectDirectionByLabel(const std::string& label, const glm::vec3 front,
        const glm::vec3 eye, std::vector<ObjectDirections>& result);
    void QueryObjectByLabel(const std::string& label,
        std::vector<ObjectAttributes>& result);

    
public:
    size_t size() const override { return robot_list_.size(); }

    const RenderBody* render_body_ptr(const size_t i) const override;

    RenderBody* render_body_ptr(const size_t i) override;
};

}

#endif // WORLD_H_
