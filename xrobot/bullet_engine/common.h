#ifndef BULLET_COMMON_H_
#define BULLET_COMMON_H_

#include <memory>

#include "bullet/LinearMath/btTransform.h"
#include "bullet/PhysicsClientC_API.h"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/vector_angle.hpp"
#include "glm/gtx/quaternion.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "../utils.h"

namespace xrobot {
    
class Inventory;
class RobotBase;
class World;

struct ContactPoint {
    glm::vec3 contact_normal;
    glm::vec3 contact_position_a;
    glm::vec3 contact_position_b;
    float contact_force;
    float contact_distance;
    int bullet_id_a;
    int bullet_id_b;
};

struct Ray {
    glm::vec3 from;
    glm::vec3 to;
};

struct RayTestInfo {
    int bullet_id;
    glm::vec3 pos;
    glm::vec3 norm;
};

namespace bullet_engine {

typedef std::shared_ptr<RobotBase> RobotBaseSPtr;
typedef std::weak_ptr<RobotBase> RobotBaseWPtr;
typedef std::shared_ptr<World> WorldSPtr;
typedef std::weak_ptr<World> WorldWPtr;

constexpr int kAllThreads = 0;
constexpr int kCacheSize = 32;
constexpr float kAngleThreshold = 0.1f;
constexpr float kDistanceThreshold = 3.0f;
const float kFloat3Zero[3] = { 0.0f };
const float kFloat4Zero[4] = { 0.0f };
const glm::vec3 kVec3Zero = glm::vec3(0); 
const glm::vec3 kVec3Up = glm::vec3(0,1,0);
const glm::vec3 kVec3Front = glm::vec3(1,0,0);
const glm::vec3 kVec3Right = glm::vec3(0,0,1);

typedef b3PhysicsClientHandle ClientHandle;
typedef b3SharedMemoryCommandHandle CommandHandle;
typedef b3SharedMemoryStatusHandle StatusHandle;

enum SleepState {
    kSleep = eActivationStateSleep,
    kWakeUp = eActivationStateWakeUp,
    kDisableSleeping = eActivationStateDisableSleeping,
    kEnableSleeping = eActivationStateEnableSleeping,
};

enum ComputeState {
    kComputeVelocity = ACTUAL_STATE_COMPUTE_LINKVELOCITY
};

enum ControlModes {
    kTorque = CONTROL_MODE_TORQUE,
    kVelocity = CONTROL_MODE_VELOCITY,
    kPositionVelocity = CONTROL_MODE_POSITION_VELOCITY_PD
};

enum ConstaintTypes {
    kFixed = eFixedType,
    kRevolute = eRevoluteType,
    kPrismatic = ePrismaticType
};

enum JointTypes {
    kRotationMotor,
    kLinearMotor
};

enum LoadingFlags {
    kOBJConcave = FORCE_CONCAVE,
    kURDFSelfCollision = URDF_USE_SELF_COLLISION,
    kURDFSelfCollisionExParents = URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS,
    kURDFEnableSleeping = URDF_ENABLE_SLEEPING
};


glm::mat4 TransformToMat4(const btTransform& transform);

template <typename T>
glm::mat4 TransformToMat4(const T* p, T* r);

btTransform TransformFromReals(const glm::vec3& p, const glm::vec4& r);

template <typename T>
btTransform TransformFromReals(const T* p, const T* r);

int get_num_joints(const ClientHandle client, const int id);

void set_pose(
        const ClientHandle client,
        const int id,
        const glm::vec3& pos,
        const glm::vec4& quat);

template <typename T>
void set_pose(
        const ClientHandle client,
        const int id,
        const T* pos,
        const T* quat = nullptr);

void rotate(const ClientHandle client, const int id, const glm::vec3 angle);

void set_velocity(const ClientHandle client, const int id, const float* v);
void set_velocity(const ClientHandle client, const int id, const double* v);
// http://www.opengl-tutorial.org
// Rotation
glm::quat RotationBetweenVectors(const glm::vec3 start, const glm::vec3 dest);

// ray-box-intersection (with direction)
// https://www.scratchapixel.com/lessons/3d-basic-rendering/
bool RayAABBIntersect(const Ray& r, 
                      const glm::vec3& aabb_min,
                      const glm::vec3& aabb_max);

}} // namespace xrobot::bullet_engine

#endif // BULLET_COMMON_H_
