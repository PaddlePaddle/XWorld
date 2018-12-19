#include "bullet_object.h"

namespace xrobot {
namespace bullet_engine {

BulletObject::BulletObject() : bullet_link_id_(-1), object_name_() {
    object_position_.setIdentity();
    object_link_position_.setIdentity();
    object_local_inertial_frame_.setIdentity();
    object_speed_ = btVector3(0,0,0);
    object_angular_speed_ = btVector3(0,0,0);
    attach_transform_.setIdentity();
}

void BulletObject::sleep(const ClientHandle client, const int body_uid) {
    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(client);
    b3ChangeDynamicsInfoSetActivationState(
            cmd_handle, body_uid, eActivationStateSleep);
    b3SubmitClientCommandAndWaitStatus(client, cmd_handle);
}

void BulletObject::enable_sleeping(
        const ClientHandle client, const int body_uid) {
    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(client);
    b3ChangeDynamicsInfoSetActivationState(
            cmd_handle, body_uid, eActivationStateEnableSleeping);
    b3SubmitClientCommandAndWaitStatus(client, cmd_handle);
}

void BulletObject::disable_sleeping(
        const ClientHandle client, const int body_uid) {
    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(client);
    b3ChangeDynamicsInfoSetActivationState(
            cmd_handle, body_uid, eActivationStateDisableSleeping);
    b3SubmitClientCommandAndWaitStatus(client, cmd_handle);
}

void BulletObject::wake(const ClientHandle client, const int body_uid) {
    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(client);
    b3ChangeDynamicsInfoSetActivationState(
                cmd_handle, body_uid, eActivationStateWakeUp);
    b3SubmitClientCommandAndWaitStatus(client, cmd_handle);
}

void BulletObject::get_mass(
        const ClientHandle client, const int body_uid, xScalar& mass) {
    CommandHandle cmd_handle = b3GetDynamicsInfoCommandInit(
            client, body_uid, bullet_link_id_);
    StatusHandle status_handle =
            b3SubmitClientCommandAndWaitStatus(client, cmd_handle);
    int status_type = b3GetStatusType(status_handle);
    if (status_type != CMD_GET_DYNAMICS_INFO_COMPLETED) {
        //printf("Get Mass Failed! Or Could Be Static Object\n");
        mass = 0.0f;
    } else {
        struct b3DynamicsInfo dynamics_info;
        b3GetDynamicsInfo(status_handle, &dynamics_info);
        mass = static_cast<xScalar>(dynamics_info.m_mass);
    }
}

void BulletObject::change_mass(
        const ClientHandle client, const int body_uid, const xScalar mass) {
    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(client);
    b3ChangeDynamicsInfoSetMass(
            cmd_handle, body_uid, bullet_link_id_, mass);
    b3SubmitClientCommandAndWaitStatus(client, cmd_handle);
}

void BulletObject::get_AABB(
        const ClientHandle client,
        const int body_uid,
        glm::vec3& aabb_min,
        glm::vec3& aabb_max) {
    CommandHandle cmd_handle =
            b3RequestCollisionInfoCommandInit(client, body_uid);
    StatusHandle status_handle =
            b3SubmitClientCommandAndWaitStatus(client, cmd_handle);

    int status_type = b3GetStatusType(status_handle);
    if (status_type != CMD_REQUEST_COLLISION_INFO_COMPLETED) {
        printf("Get AABB Failed!\n");
        return;
    }

    xScalar aabb_min_temp[3];
    xScalar aabb_max_temp[3];
    b3GetStatusAABB(
            status_handle, bullet_link_id_, aabb_min_temp, aabb_max_temp);
    aabb_min = glm::vec3(aabb_min_temp[0], aabb_min_temp[1], aabb_min_temp[2]);
    aabb_max = glm::vec3(aabb_max_temp[0], aabb_max_temp[1], aabb_max_temp[2]);
}

void BulletObject::change_linear_damping(
        const ClientHandle client, const int body_uid, const xScalar damping) {
    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(client);
    b3ChangeDynamicsInfoSetLinearDamping(cmd_handle, body_uid, damping);
    b3SubmitClientCommandAndWaitStatus(client, cmd_handle);
}

void BulletObject::change_angular_damping(
        const ClientHandle client, const int body_uid, const xScalar damping) {
    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(client);
    b3ChangeDynamicsInfoSetAngularDamping(cmd_handle, body_uid, damping);
    b3SubmitClientCommandAndWaitStatus(client, cmd_handle);
}

void BulletObject::change_lateral_friction(
        const ClientHandle client, const int body_uid, const xScalar friction) {
    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(client);
    b3ChangeDynamicsInfoSetLateralFriction(
            cmd_handle, body_uid, bullet_link_id_, friction);
    b3SubmitClientCommandAndWaitStatus(client, cmd_handle);
}

void BulletObject::change_spinning_friction(
        const ClientHandle client, const int body_uid, const xScalar friction) {
    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(client);
    b3ChangeDynamicsInfoSetSpinningFriction(
            cmd_handle, body_uid, bullet_link_id_, friction);
    b3SubmitClientCommandAndWaitStatus(client, cmd_handle);
}

void BulletObject::change_rolling_friction(
        const ClientHandle client, const int body_uid, xScalar friction) {
    CommandHandle cmd_handle = b3InitChangeDynamicsInfo(client);
    b3ChangeDynamicsInfoSetRollingFriction(
            cmd_handle, body_uid, bullet_link_id_, friction);
    b3SubmitClientCommandAndWaitStatus(client, cmd_handle);
}

void BulletObject::apply_force(
        const ClientHandle client,
        const int body_uid,
        const xScalar x,
        const xScalar y,
        const xScalar z,
        const int flags) {
    printf("BulletObject::apply_force not implemented!\n");
    assert(false);
}

void BulletObject::apply_torque(
        const ClientHandle client,
        const int body_uid,
        const xScalar x,
        const xScalar y,
        const xScalar z,
        const int flags) {
    CommandHandle cmd_handle = 
            b3ApplyExternalForceCommandInit(client);

    xScalar torque_temp[3];
    torque_temp[0] = x;
    torque_temp[1] = y;
    torque_temp[2] = z;

    b3ApplyExternalTorque(
            cmd_handle, body_uid, bullet_link_id_, torque_temp, flags);
    b3SubmitClientCommandAndWaitStatus(client, cmd_handle);
}

glm::mat4 BulletObject::translation_matrix() const {
    return TransformToMat4(object_position_);
}

glm::mat4 BulletObject::local_inertial_frame() const {
    return TransformToMat4(object_local_inertial_frame_);
}

void BulletObject::pose(btVector3& pos, btQuaternion& quat) {
    pos = object_position_.getOrigin();
    quat = object_position_.getRotation();
}

void BulletObject::pose(glm::vec3& pos, glm::vec4& quat) {
    auto p = object_position_.getOrigin();
    auto q = object_position_.getRotation();
    pos[0] = p[0];
    pos[1] = p[1];
    pos[2] = p[2];
    quat[0] = q[0];
    quat[1] = q[1];
    quat[2] = q[2];
    quat[3] = q[3];
}

}} // namespace xrobot::bullet_engine
