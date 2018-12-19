#ifndef BULLET_OBJECT_H_
#define BULLET_OBJECT_H_

#include "common.h"

namespace xrobot {
namespace bullet_engine {

class BulletObject {
public:
    BulletObject();

    virtual ~BulletObject() {};

// protected:
    void sleep(const ClientHandle client, const int body_uid);

    void enable_sleeping(const ClientHandle client, const int body_uid);

    void disable_sleeping(const ClientHandle client, const int body_uid);

    void wake(const ClientHandle client, const int body_uid);

    void get_mass(const ClientHandle client, const int body_uid, xScalar& mass);

    void change_mass(
            const ClientHandle client, const int body_uid, const xScalar mass);

    void get_AABB(
            const ClientHandle client,
            const int body_uid,
            glm::vec3& aabb_min,
            glm::vec3& aabb_max);

    void change_linear_damping(
            const ClientHandle client, const int body_uid, const xScalar damping);

    void change_angular_damping(
            const ClientHandle client, const int body_uid, const xScalar damping);

    void change_lateral_friction(
            const ClientHandle client, const int body_uid, const xScalar friction);

    void change_spinning_friction(
            const ClientHandle client, const int body_uid, const xScalar friction);

    void change_rolling_friction(
            const ClientHandle client, const int body_uid, const xScalar friction);
    
    void apply_force(
            const ClientHandle client,
            const int body_uid,
            const xScalar x,
            const xScalar y,
            const xScalar z,
            const int flags = EF_LINK_FRAME);

    void apply_torque(
            const ClientHandle client,
            const int body_uid,
            const xScalar x,
            const xScalar y,
            const xScalar z, 
            const int flags = EF_LINK_FRAME);

    void detach() { attach_transform_ = btTransform(); }

    glm::mat4 translation_matrix() const;

    glm::mat4 local_inertial_frame() const;

    void pose(glm::vec3& pos, glm::vec4& quat);

    void pose(btVector3& pos, btQuaternion& quat);

public:
    int bullet_link_id_;

    std::string object_name_;
    btTransform object_position_;
    btTransform object_link_position_;
    btTransform object_local_inertial_frame_;
    btVector3   object_speed_;
    btVector3   object_angular_speed_;
    btTransform attach_transform_;
};

typedef std::shared_ptr<BulletObject> BulletObjectSPtr;
typedef std::weak_ptr<BulletObject>   BulletObjectWPtr;

}} // namespace xrobot::bullet_engine

#endif
