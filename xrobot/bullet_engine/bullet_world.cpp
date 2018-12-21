#include "bullet_world.h"

namespace xrobot {
namespace bullet_engine {

BulletWorld::BulletWorld() : client_(0),
							 bullet_gravity_(0),
							 bullet_timestep_(0),
							 bullet_ts_(0) {}

BulletWorld::~BulletWorld() {
	b3DisconnectSharedMemory(client_);
}

void BulletWorld::init(const double gravity, const double timestep) {
	client_ = b3ConnectPhysicsDirect();

	bullet_gravity_ = gravity;
    bullet_timestep_ = timestep;

    CommandHandle cmd_handle = b3InitPhysicsParamCommand(client_);
    b3PhysicsParamSetEnableFileCaching(cmd_handle, 1);
    b3PhysicsParamSetGravity(cmd_handle, 0, bullet_gravity_, 0);
	b3PhysicsParamSetDefaultContactERP(cmd_handle, 0.2);
	b3PhysicsParamSetTimeStep(cmd_handle, bullet_timestep_);
    b3SubmitClientCommandAndWaitStatus(client_, cmd_handle);
}

void BulletWorld::step() {
	bullet_ts_ += bullet_timestep_;

    CommandHandle cmd_handle = b3InitPhysicsParamCommand(client_);
    b3PhysicsParamSetGravity(cmd_handle, 0, bullet_gravity_, 0);
    b3PhysicsParamSetTimeStep(cmd_handle, bullet_timestep_);
    b3SubmitClientCommandAndWaitStatus(client_, cmd_handle);

	cmd_handle = b3InitStepSimulationCommand(client_);
    b3SubmitClientCommandAndWaitStatus(client_, cmd_handle);
}

void BulletWorld::reset() {
	CommandHandle cmd_handle = b3InitResetSimulationCommand(client_);
	b3SubmitClientCommandAndWaitStatus(client_, cmd_handle);
}

void BulletWorld::cast_rays(const std::vector<Ray>& rays, 
				   	   std::vector<RayTestInfo>& results,
				       const unsigned int threads) {
	CommandHandle cmd_handle = b3CreateRaycastBatchCommandInit(client_);
    b3RaycastBatchSetNumThreads(cmd_handle, threads);

    double ray_from_position_temp[rays.size() * 3];
    double ray_to_position_temp[rays.size() * 3];
    for (int i = 0; i < rays.size(); ++i)
    {
        ray_from_position_temp[3 * i + 0] = rays[i].from.x;
        ray_from_position_temp[3 * i + 1] = rays[i].from.y;
        ray_from_position_temp[3 * i + 2] = rays[i].from.z;

        ray_to_position_temp[3 * i + 0] = rays[i].to.x;
        ray_to_position_temp[3 * i + 1] = rays[i].to.y;
        ray_to_position_temp[3 * i + 2] = rays[i].to.z;
    }

    b3RaycastBatchAddRays(client_, 
    	cmd_handle, ray_from_position_temp, ray_to_position_temp, rays.size());

    b3SubmitClientCommandAndWaitStatus(client_, cmd_handle);

    struct b3RaycastInformation raycast_info;
    b3GetRaycastInformation(client_, &raycast_info);

    for (int i = 0; i < raycast_info.m_numRayHits; ++i)
    {
        RayTestInfo res;
        res.bullet_id = raycast_info.m_rayHits[i].m_hitObjectUniqueId;

        glm::vec3 pos_temp;
        pos_temp[0] = raycast_info.m_rayHits[i].m_hitPositionWorld[0];
        pos_temp[1] = raycast_info.m_rayHits[i].m_hitPositionWorld[1];
        pos_temp[2] = raycast_info.m_rayHits[i].m_hitPositionWorld[2];
        res.pos = pos_temp;

        glm::vec3 norm_temp;
        norm_temp[0] = raycast_info.m_rayHits[i].m_hitNormalWorld[0];
        norm_temp[1] = raycast_info.m_rayHits[i].m_hitNormalWorld[1];
        norm_temp[2] = raycast_info.m_rayHits[i].m_hitNormalWorld[2];
        res.norm = norm_temp;
        
        results.push_back(res);
    }
}

void BulletWorld::cast_ray(const Ray& ray, RayTestInfo& result) {
    std::vector<Ray> rays;
    std::vector<RayTestInfo> results(0);

    rays.push_back(ray);
    
    cast_rays(rays, results, 1);
    result = results[0];
}

}} // namespace xrobot::bullet_engine
