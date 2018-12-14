#include "common.h"

namespace xrobot {
namespace bullet_engine {

template <typename T>
btTransform TransformFromReals(const T* p, const T* r) {
    btVector3 pp(p[0], p[1], p[2]);
    btQuaternion rr(r[0], r[1], r[2], r[3]);
    
    return btTransform(rr, pp);
}

template btTransform TransformFromReals<float>(const float*, const float*);
template btTransform TransformFromReals<double>(const double*, const double*);

glm::mat4 TransformToMat4(const btTransform& transform) {
    xScalar mat[16];
    transform.getOpenGLMatrix(mat);
    return glm::make_mat4(mat);
}

template <typename T>
glm::mat4 TransformToMat4(const T* p, const T* r) {
    return TransformToMat4(TransformFromReals(p, r));
}

template glm::mat4 TransformToMat4<float>(const float*, const float*);
template glm::mat4 TransformToMat4<double>(const double*, const double*);

int get_num_joints(const ClientHandle client, const int id) {
    return b3GetNumJoints(client, id);
}

template <typename T>
void set_vel(
        const ClientHandle client,
        const int id,
        const T* vel) {
    CommandHandle cmd_handle = b3CreatePoseCommandInit(client, id);

    double velocity[3];
    velocity[0] = vel[0];
    velocity[1] = vel[1];
    velocity[2] = vel[2];

    b3CreatePoseCommandSetBaseLinearVelocity(cmd_handle, velocity);
    b3SubmitClientCommandAndWaitStatus(client, cmd_handle);
}

template void set_vel<float>(const ClientHandle, const int, const float*);
template void set_vel<double>(const ClientHandle, const int, const double*);


template <typename T>
void set_pose(
        const ClientHandle client,
        const int id,
        const T* pos,
        const T* quat) {
    CommandHandle cmd_handle = b3CreatePoseCommandInit(client, id);
    if (pos) {
        b3CreatePoseCommandSetBasePosition(cmd_handle, pos[0], pos[1], pos[2]);
    }
    if (quat) {
        b3CreatePoseCommandSetBaseOrientation(
                cmd_handle, quat[0], quat[1], quat[2], quat[3]);
    }
    b3SubmitClientCommandAndWaitStatus(client, cmd_handle);
}

template void set_pose<float>(const ClientHandle, const int, const float*, const float*);
template void set_pose<double>(const ClientHandle, const int, const double*, const double*);

void rotate(const ClientHandle client, const int id, const glm::vec3 angle) {
    btQuaternion quat(angle.x, angle.y, angle.z);
    xScalar q[4];
    q[0] = quat[0];
    q[1] = quat[1];
    q[2] = quat[2];
    q[3] = quat[3];
    set_pose(client, id, (xScalar*)NULL, q);
}

template <typename T>
void set_velocity(const ClientHandle client, const int id, const T* velocity) {
    CommandHandle cmd_handle = b3CreatePoseCommandInit(client, id);
    xScalar v[3];
    v[0] = velocity[0];
    v[1] = velocity[1];
    v[2] = velocity[2];
    b3CreatePoseCommandSetBaseLinearVelocity(cmd_handle, v);
    b3SubmitClientCommandAndWaitStatus(client, cmd_handle);
}

template void set_velocity<float>(const ClientHandle, const int, const float*);
template void set_velocity<double>(const ClientHandle, const int, const double*);

// http://www.opengl-tutorial.org
// Rotation
glm::quat RotationBetweenVectors(
        const glm::vec3 start, const glm::vec3 dest) {
    glm::vec3 start_norm = glm::normalize(start);
    glm::vec3 dest_norm = glm::normalize(dest);

    xScalar cosTheta = glm::dot(start_norm, dest_norm);
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

}} // namespace xrobot::bullet_engine
