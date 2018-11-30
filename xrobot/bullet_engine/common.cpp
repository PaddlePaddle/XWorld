#include "common.h"

namespace xrobot {
namespace bullet_engine {

glm::mat4 TransformToMat4(const btTransform& transform) {
    btScalar mat[16];
    transform.getOpenGLMatrix(mat);
    return glm::make_mat4(mat);
}

btTransform TransformFromDoubles(
        const double* position, const double* orientation) {
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
glm::quat RotationBetweenVectors(
        const glm::vec3 start, const glm::vec3 dest) {
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

}} // namespace xrobot::bullet_engine
