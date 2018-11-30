#ifndef BULLET_JOINT_H_
#define BULLET_JOINT_H_

#include "common.h"

namespace xrobot {
namespace bullet_engine {

class BulletJoint {
public:
    BulletJoint();

    ~BulletJoint() {}

protected:
    void enable_sensor(const ClientHandle client,
                       const int body_uid,
                       const bool enable);

    void get_motor_state(const ClientHandle client,
                         const int body_uid,
                         glm::vec3& force,
                         glm::vec3& torque);

    void reset_state(const ClientHandle client,
                     const int body_uid,
                     const float pos,
                     const float vel);

    void set_motor_control_torque(const ClientHandle client,
                                  const int body_uid,
                                  const float torque);

    void set_motor_control_velocity(const ClientHandle client,
                                    const int body_uid,
                                    const float speed,
                                    const float k_d,
                                    const float max_force);

    void set_motor_control_position(const ClientHandle client,
                                    const int body_uid,
                                    const float target,
                                    const float k_p,
                                    const float k_d,
                                    const float max_force);

public:
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

}} // namespace xrobot::bullet_engine

#endif // BULLET_JOINT_H_
