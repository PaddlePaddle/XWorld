#ifndef LIDAR_H_
#define LIDAR_H_

#include <vector>

#include "glm/glm.hpp"

#include "world.h"

namespace xrobot {

class Lidar {
public:
    Lidar(World* world, const int number_of_rays, const float max_distance);

    ~Lidar();

    void Update(
            const glm::vec3& front,
            const glm::vec3& up,
            const glm::vec3& center);

    std::vector<RayTestInfo> GetResult() const;

    int GetNumRays() const { return num_rays_; }
    int GetMaxDistance() const { return max_distance_; }

private:
    World * world_;
    int num_rays_;
    float max_distance_;
    float horizontal_segment_angle_;
    std::vector<RayTestInfo> batch_ray_result_;
};

}

#endif // LIDAR_H_
