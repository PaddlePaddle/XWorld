#include "frustum.h"

namespace xrobot {
namespace render_engine {

void CameraFrustum::Update(Camera* camera) {
    float tan = 2.0 * std::tan(camera->GetFOVRadius() * 0.5);
    float nearHeight = tan * camera->GetNear();
    float nearWidth  = nearHeight * camera->GetAspect();
    float farHeight  = tan * camera->GetFar();
    float farWidth   = farHeight * camera->GetAspect();

    glm::vec3 nearCenter = camera->position_ + camera->front_ * camera->GetNear();
    glm::vec3 farCenter  = camera->position_ + camera->front_ * camera->GetFar();

    glm::vec3 v;
    // left plane
    v = (nearCenter + camera->right_ * nearWidth * 0.5f) - camera->position_;
    Planes[0].SetNormalD(glm::cross(glm::normalize(v), camera->up_), nearCenter - camera->right_ * nearWidth * 0.5f);
    // right plane
    v = (nearCenter - camera->right_ * nearWidth  * 0.5f) - camera->position_;
    Planes[1].SetNormalD(glm::cross(camera->up_, glm::normalize(v)), nearCenter + camera->right_ * nearWidth * 0.5f);
    // top plane
    v = (nearCenter + camera->up_ * nearHeight * 0.5f) - camera->position_;
    Planes[2].SetNormalD(glm::cross(glm::normalize(v), -camera->right_), nearCenter + camera->up_ * nearHeight * 0.5f);
    // bottom plane
    v = (nearCenter - camera->up_ * nearHeight * 0.5f) - camera->position_;
    Planes[3].SetNormalD(glm::cross(-camera->right_, glm::normalize(v)), nearCenter - camera->up_ * nearHeight * 0.5f);
    // near plane
    Planes[4].SetNormalD(camera->front_, nearCenter);
    // far plane
    Planes[5].SetNormalD(-camera->front_, farCenter);
}

bool CameraFrustum::Intersect(glm::vec3 point) {
    for (int i = 0; i < 6; ++i)
    {
        if (Planes[i].Distance(point) < 0)
        {
            return false;
        }
    }
    return true;
}

bool CameraFrustum::Intersect(glm::vec3 point, float radius) {
    for (int i = 0; i < 6; ++i)
    {
        if (Planes[i].Distance(point) < -radius)
        {
            return false;
        }
    }
    return true;
}

bool CameraFrustum::Intersect(glm::vec3 boxMin, glm::vec3 boxMax) {
    for (int i = 0; i < 6; ++i)
    {
        glm::vec3 positive = boxMin;
        if (Planes[i].Normal.x >= 0)
        {
            positive.x = boxMax.x;
        }
        if (Planes[i].Normal.y >= 0)
        {
            positive.y = boxMax.y;
        }
        if (Planes[i].Normal.z >= 0)
        {
            positive.z = boxMax.z;
        }
        if(Planes[i].Distance(positive) < 0)
        {
            return false;
        }
    }
    return true;
}

}
}