#ifndef BULLET_JOINT_H_
#define BULLET_JOINT_H_

#include "common.h"

namespace xrobot {
namespace bullet_engine {

class BulletJoint {
public:
    BulletJoint();

    virtual ~BulletJoint() {}

protected:
    void enable_sensor(
            const ClientHandle client, const int body_uid, const bool enable);

    void get_motor_state(
            const ClientHandle client,
            const int body_uid,
            glm::vec3& force,
            glm::vec3& torque);

    void reset_state(
            const ClientHandle client,
            const int body_uid,
            const xScalar pos,
            const xScalar vel);

    void set_motor_control_torque(
            const ClientHandle client, const int body_uid, const xScalar torque);

    void set_motor_control_velocity(
            const ClientHandle client,
            const int body_uid,
            const xScalar speed,
            const xScalar k_d,
            const xScalar max_force);

    void set_motor_control_position(
            const ClientHandle client,
            const int body_uid,
            const xScalar target,
            const xScalar k_p,
            const xScalar k_d,
            const xScalar max_force);

public:
    std::string joint_name_;
    int joint_type_;
    int bullet_joint_id_;
    int bullet_q_index_;
    int bullet_u_index_;
    xScalar joint_current_position_;
    xScalar joint_current_speed_;
    xScalar joint_limit_1_;
    xScalar joint_limit_2_;
    xScalar joint_max_force_;
    xScalar joint_max_velocity_;
    bool joint_has_limits_;

};

typedef std::shared_ptr<BulletJoint> BulletJointSPtr;

}} // namespace xrobot::bullet_engine

#endif // BULLET_JOINT_H_
