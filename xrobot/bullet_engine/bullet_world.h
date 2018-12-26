#ifndef BULLET_WORLD_H_
#define BULLET_WORLD_H_

#include "common.h"

namespace xrobot {
namespace bullet_engine {

class BulletWorld {
public:
	BulletWorld();

	virtual ~BulletWorld();

	void init(const double gravity = 9.8f, const double timestep = 0.005f);

	void reset();

	void step();

	void cast_rays(const std::vector<Ray>& rays, 
				   std::vector<RayTestInfo>& results,
				   const unsigned int threads = kAllThreads);

	void cast_ray(const Ray& ray, RayTestInfo& result);

public:
	ClientHandle client_;

	double bullet_gravity_;
	double bullet_timestep_;
    double bullet_ts_;
};

}} // namespace xrobot::bullet_engine

#endif
