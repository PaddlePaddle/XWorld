#ifndef RENDER_ENGINE_FRUSTUM_H_
#define RENDER_ENGINE_FRUSTUM_H_

#include <chrono>
#include <ctime>
#include <ratio>
#include <stdio.h>
#include <string>
#include <vector>

#include "camera.h"
#include "gl_context.h"

namespace xrobot {
namespace render_engine {

struct FrustumPlane {
	glm::vec3 Normal;
	float     D;

	void SetNormalD(glm::vec3 normal, glm::vec3 point) {
	    Normal = glm::normalize(normal);
	    D      = -glm::dot(Normal, point);
	}

	float Distance(glm::vec3 point) {
	    return glm::dot(Normal, point) + D;
	}
};

class CameraFrustum {
public:
	FrustumPlane Planes[6];

public:
    CameraFrustum() {}

    void Update(Camera* camera);
    bool Intersect(glm::vec3 point);
    bool Intersect(glm::vec3 point, float radius);
    bool Intersect(glm::vec3 boxMin, glm::vec3 boxMax);
};

}
}

#endif // RENDER_ENGINE_FRUSTUM_H_
