#ifndef BULLET_WORLD_H_
#define BULLET_WORLD_H_

#include "common.h"

namespace xrobot {
namespace bullet_engine {

class BulletWorld {
public:
	BulletWorld();

	virtual ~BulletWorld();

	void init(const xScalar gravity = 9.8f, const xScalar timestep = 0.01f);

	void reset();

	void step();

	void cast_rays(const std::vector<Ray>& rays, 
				   std::vector<RayTestInfo>& results,
				   const unsigned int threads = kAllThreads);

	void cast_ray(const Ray& ray, RayTestInfo& result);

public:
	ClientHandle client_;

	xScalar bullet_gravity_;
	xScalar bullet_timestep_;
    xScalar bullet_ts_;
};

}} // namespace xrobot::bullet_engine

#endif