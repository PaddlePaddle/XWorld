#include "lidar.h"

namespace xrobot {
	Lidar::Lidar(World * world, const int number_of_rays,
	             const float max_distance)
				 : world_(world),
				   num_rays_(number_of_rays),
				   max_distance_(max_distance),
				   horizontal_segment_angle_(glm::radians(360.0f / number_of_rays)),
				   batch_ray_result_(0)
	{
		assert(number_of_rays > 0);
	}

	Lidar::~Lidar() {}

	void Lidar::Update(const glm::vec3 front,
	                   const glm::vec3 up,
	                   const glm::vec3 center)
	{
		batch_ray_result_.clear();

		std::vector<Ray> batch_rays(num_rays_);

		for (int i = 0; i < num_rays_; ++i)
	    {
	        glm::quat lidar_orentation = glm::angleAxis(horizontal_segment_angle_ * i,
	        		up);
	        glm::vec3 lidar_ray_direction = glm::rotate(lidar_orentation,
	                front);
	        glm::vec3 lidar_far_position = lidar_ray_direction * max_distance_ +
	                center;
	        batch_rays[i] = {center, lidar_far_position};
	    }

	    world_->BatchRayTest(batch_rays, batch_ray_result_, 0);
	}

	std::vector<RayTestInfo> Lidar::GetResult() const
	{
		return batch_ray_result_;
	}
}