#include "bullet_joint.h"

namespace xrobot {
namespace bullet_engine {

BulletJoint::BulletJoint() : joint_name_(""),
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

void BulletJoint::enable_sensor(const ClientHandle client,
                                const int body_uid,
                                const bool enable) {
    CommandHandle cmd_handle = b3CreateSensorCommandInit(client, body_uid);
    b3CreateSensorEnable6DofJointForceTorqueSensor(
            cmd_handle, bullet_joint_id_, enable);
    b3SubmitClientCommandAndWaitStatus(client, cmd_handle);
}

void BulletJoint::get_motor_state(const ClientHandle client,
                                  const int body_uid,
                                  glm::vec3& force,
                                  glm::vec3& torque) {
    CommandHandle cmd_handle = b3RequestActualStateCommandInit(client, body_uid);
    StatusHandle status_handle = 
            b3SubmitClientCommandAndWaitStatus(client, cmd_handle);

    struct b3JointSensorState sensorState;
    b3GetJointState(client, status_handle, bullet_joint_id_, &sensorState);

    force.x = sensorState.m_jointForceTorque[0];
    force.y = sensorState.m_jointForceTorque[1];
    force.z = sensorState.m_jointForceTorque[2];
    torque.x = sensorState.m_jointForceTorque[3];
    torque.y = sensorState.m_jointForceTorque[4];
    torque.z = sensorState.m_jointForceTorque[5];
}

void BulletJoint::reset_state(const ClientHandle client,
                              const int body_uid,
                              const float pos, const float vel) {
    CommandHandle cmd_handle = b3CreatePoseCommandInit(client, body_uid);
    b3CreatePoseCommandSetJointPosition(
            client, cmd_handle, bullet_joint_id_, pos);
    b3CreatePoseCommandSetJointVelocity(
            client, cmd_handle, bullet_joint_id_, vel);
    b3SubmitClientCommandAndWaitStatus(client, cmd_handle);
}


void BulletJoint::set_motor_control_torque(const ClientHandle client,
                                           const int body_uid,
                                           const float torque) {
    CommandHandle cmd_handle =
            b3JointControlCommandInit2(client, body_uid, kTorque);
    b3JointControlSetDesiredForceTorque(cmd_handle, bullet_u_index_, torque);
    b3SubmitClientCommandAndWaitStatus(client, cmd_handle);
}

void BulletJoint::set_motor_control_velocity(const ClientHandle client,
                                             const int body_uid,
                                             const float speed,
                                             const float k_d,
                                             const float max_force) {
    CommandHandle cmd_handle = b3JointControlCommandInit2(
            client,
            body_uid,
            kVelocity);
    b3JointControlSetDesiredVelocity(cmd_handle, bullet_u_index_, speed);
    b3JointControlSetKd(cmd_handle, bullet_u_index_, k_d);
    b3JointControlSetMaximumForce(cmd_handle, bullet_u_index_, max_force);
    b3SubmitClientCommandAndWaitStatus(client, cmd_handle);
}

void BulletJoint::set_motor_control_position(
        const ClientHandle client,
        const int body_uid,
        const float target,
        const float k_p,
        const float k_d,
        const float max_force) {
    CommandHandle cmd_handle = b3JointControlCommandInit2(
            client,
            body_uid,
            kPositionVelocity);

    b3JointControlSetDesiredPosition(cmd_handle, bullet_q_index_, target);
    b3JointControlSetKd(cmd_handle, bullet_u_index_, k_d);
    b3JointControlSetKp(cmd_handle, bullet_u_index_, k_p);
    b3JointControlSetMaximumForce(cmd_handle, bullet_u_index_, max_force);
    b3SubmitClientCommandAndWaitStatus(client, cmd_handle);
}

}} // namespace xrobot::bullet_engine
