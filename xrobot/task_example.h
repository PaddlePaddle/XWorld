#ifndef TASK_EXAMPLE_H_
#define TASK_EXAMPLE_H_

#include <vector>
#include <functional>
#include <unordered_map>

#include "render_engine/render.h"
#include "map.h"
#include "map_suncg.h"
#include "lidar.h"
#include "task.h"

namespace xrobot
{
	typedef std::unordered_map<std::string,
							   std::function<std::string()>> TaskStages;

	class Task_FollowRobot : public TaskInterface {
	public:

		Task_FollowRobot(render_engine::Render * renderer, Map * map);
		~Task_FollowRobot();

		std::string Start();
		std::string NavTarget();
		TaskStages GetStages();

		int iterations_;
		Map * scene_;
		RobotBase * agent_;
		RobotBase * target_;
		render_engine::GLContext * ctx_;
		render_engine::Render * renderer_;
		render_engine::Camera * main_camera_;
		float cam_pitch_;
	};

	class Task_NavToLargeCrate : public TaskInterface {
	public:

		Task_NavToLargeCrate(render_engine::Render * renderer, Map * map);
		~Task_NavToLargeCrate();

		std::string Start();
		std::string NavTarget();
		TaskStages GetStages();

		int iterations_;
		Map * scene_;
		RobotBase * agent_;
		Lidar * lidar_;
		render_engine::GLContext * ctx_;
		render_engine::Render * renderer_;
		render_engine::Camera * main_camera_;
		float cam_pitch_;
	};

	class Task_NavToSmallCrate : public TaskInterface {
	public:

		Task_NavToSmallCrate(render_engine::Render * renderer, Map * map);
		~Task_NavToSmallCrate();

		std::string Start();
		std::string NavTarget();
		TaskStages GetStages();

		int iterations_;
		Map * scene_;
		RobotBase * agent_;
		Lidar * lidar_;
		render_engine::GLContext * ctx_;
		render_engine::Render * renderer_;
		render_engine::Camera * main_camera_;
		float cam_pitch_;
	};

	class Task_NavToFruitBowl : public TaskInterface {
	public:

		Task_NavToFruitBowl(render_engine::Render * renderer, MapSuncg * map); 
		~Task_NavToFruitBowl();

		std::string Start();
		std::string NavTarget();
		TaskStages GetStages();

		int iterations_;
		MapSuncg * scene_;
		RobotBase * agent_;
		float cam_pitch_;
		render_engine::GLContext * ctx_;
		render_engine::Render * renderer_;
		render_engine::Camera * main_camera_;

		float pos_0_ = 0.0f;
	    float pos_1_ = 0.0f;
	    float pos_2_ = 0.0f;
	    float pos_3_ = 0.0f;
	    float pos_4_ = 0.0f;
	    float pos_5_ = 0.0f;
	    float pos_6_ = 0.0f;
	    float pos_7_ = 0.0f;
	};

	class Task_TouchPan : public TaskInterface {
	public:

		Task_TouchPan(render_engine::Render * renderer, MapSuncg * map);
		~Task_TouchPan();

		std::string Start();
		std::string NavTarget();
		TaskStages GetStages();
		
		int iterations_;
		MapSuncg * scene_;
		RobotBase * agent_;
		float cam_pitch_;
		render_engine::GLContext * ctx_;
		render_engine::Render * renderer_;
		render_engine::Camera * main_camera_;

		float pos_0_ = 0.0f;
	    float pos_1_ = 0.0f;
	    float pos_2_ = 0.0f;
	    float pos_3_ = 0.0f;
	    float pos_4_ = 0.0f;
	    float pos_5_ = 0.0f;
	    float pos_6_ = 0.0f;
	    float pos_7_ = 0.0f;
	};

	class Task_NewFeatures : public TaskInterface {
	public:

		Task_NewFeatures(render_engine::Render * renderer, Map * map);
		~Task_NewFeatures();

		std::string Start();
		std::string NavTarget();
		TaskStages GetStages();
		
		int iterations_;
		Inventory * inventory_;
		Map * scene_;
		RobotBase * door_;
		RobotBase * agent_;
		RobotBase * target_;
		RobotBase * obj_conv_;
		RobotBase * door_anim_;
		render_engine::GLContext * ctx_;
		render_engine::Render * renderer_;
		render_engine::Camera * main_camera_;
		float cam_pitch_;
		float door_angle_;
	};
}

#endif // TASK_EXAMPLE_H_