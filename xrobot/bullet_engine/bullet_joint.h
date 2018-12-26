#ifndef BULLET_JOINT_H_
#define BULLET_JOINT_H_

#include "common.h"

namespace xrobot {
namespace bullet_engine {

class BulletJoint {
public:
    BulletJoint();

    virtual ~BulletJoint() {}

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
            const double pos,
            const double vel);

    void set_motor_control_torque(
            const ClientHandle client, const int body_uid, const double torque);

    void set_motor_control_velocity(
            const ClientHandle client,
            const int body_uid,
            const double speed,
            const double k_d,
            const double max_force);

    void set_motor_control_position(
            const ClientHandle client,
            const int body_uid,
            const double target,
            const double k_p,
            const double k_d,
            const double max_force);

    std::string joint_name_;
    int joint_type_;
    int bullet_joint_id_;
    int bullet_q_index_;
    int bullet_u_index_;
    double joint_current_position_;
    double joint_current_speed_;
    double joint_limit_1_;
    double joint_limit_2_;
    double joint_max_force_;
    double joint_max_velocity_;
    bool joint_has_limits_;

};

typedef std::shared_ptr<BulletJoint> BulletJointSPtr;

}} // namespace xrobot::bullet_engine

#endif // BULLET_JOINT_H_
