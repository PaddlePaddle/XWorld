#ifndef TASK_EXAMPLE_H_
#define TASK_EXAMPLE_H_

#include <vector>
#include <functional>
#include <unordered_map>

#include "render_engine/render.h"
#include "map_grid.h"
#include "map_suncg.h"
#include "lidar.h"
#include "task.h"
#include "navigation.h"

namespace xrobot
{
	typedef std::unordered_map<std::string,
							   std::function<std::string()>> TaskStages;

	
	class Task_FollowRobot : public TaskInterface {
	public:

		Task_FollowRobot(std::shared_ptr<render_engine::Render> renderer,
				   		 std::shared_ptr<MapGrid> map);
		~Task_FollowRobot();

		std::string Start();
		std::string NavTarget();
		TaskStages GetStages();

		int iterations_;
		float cam_pitch_;

		std::weak_ptr<RobotBase> agent_;
		std::weak_ptr<RobotBase> target_;
		std::shared_ptr<MapGrid> scene_;
		std::shared_ptr<render_engine::Render> renderer_;

		render_engine::GLContext * ctx_;
		render_engine::Camera * main_camera_;
	};

	
	class Task_NavToLargeCrate : public TaskInterface {
	public:

		Task_NavToLargeCrate(std::shared_ptr<render_engine::Render> renderer,
				   		 	 std::shared_ptr<MapGrid> map);
		~Task_NavToLargeCrate();

		std::string Start();
		std::string NavTarget();
		TaskStages GetStages();

		int iterations_;
		float cam_pitch_;

		std::weak_ptr<RobotBase> agent_;
		std::shared_ptr<MapGrid> scene_;
		std::shared_ptr<Lidar> lidar_;
		std::shared_ptr<render_engine::Render> renderer_;

		render_engine::GLContext * ctx_;
		render_engine::Camera * main_camera_;
	};

	
	class Task_NavToSmallCrate : public TaskInterface {
	public:

		Task_NavToSmallCrate(std::shared_ptr<render_engine::Render> renderer,
				   		 	 std::shared_ptr<MapGrid> map);
		~Task_NavToSmallCrate();

		std::string Start();
		std::string NavTarget();
		TaskStages GetStages();

		int iterations_;
		float cam_pitch_;
		
		std::weak_ptr<RobotBase> agent_;
		std::shared_ptr<MapGrid> scene_;
		std::shared_ptr<Lidar> lidar_;
		std::shared_ptr<render_engine::Render> renderer_;

		render_engine::GLContext * ctx_;
		render_engine::Camera * main_camera_;
	};

	
	class Task_NavToFruitBowl : public TaskInterface {
	public:

		Task_NavToFruitBowl(std::shared_ptr<render_engine::Render> renderer,
				   		 	std::shared_ptr<MapSuncg> map); 
		~Task_NavToFruitBowl();

		std::string Start();
		std::string NavTarget();
		TaskStages GetStages();

		int iterations_;
		float cam_pitch_;

		std::shared_ptr<Inventory> inventory_;
		std::shared_ptr<Lidar> lidar_;
		std::weak_ptr<RobotBase> agent_;
		std::weak_ptr<RobotBase> obj_conv_;
		std::shared_ptr<MapSuncg> scene_;
		std::shared_ptr<render_engine::Render> renderer_;

		render_engine::GLContext * ctx_;
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

		Task_TouchPan(std::shared_ptr<render_engine::Render> renderer,
				   	  std::shared_ptr<MapSuncg> map);
		~Task_TouchPan();

		std::string Start();
		std::string NavTarget();
		TaskStages GetStages();
		
		int iterations_;
		float cam_pitch_;

		std::shared_ptr<Lidar> lidar_;
		std::weak_ptr<RobotBase> agent_;
		std::shared_ptr<MapSuncg> scene_;
		std::shared_ptr<render_engine::Render> renderer_;
		
		render_engine::GLContext * ctx_;
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

		Task_NewFeatures(std::shared_ptr<render_engine::Render> renderer,
				   		 std::shared_ptr<MapGrid> map);
		~Task_NewFeatures();

		std::string Start();
		std::string NavTarget();
		TaskStages GetStages();
		
		int iterations_;
		float cam_pitch_;
		
		std::weak_ptr<RobotBase> agent_;
		std::weak_ptr<RobotBase> obj_conv_;
		std::weak_ptr<RobotBase> door_anim_;
		std::shared_ptr<render_engine::Render> renderer_;
		std::shared_ptr<Inventory> inventory_;
		std::shared_ptr<MapGrid> scene_;

		render_engine::GLContext * ctx_;
		render_engine::Camera * main_camera_;
	};

	class Task_Crowd : public TaskInterface {
	public:

		Task_Crowd(std::shared_ptr<render_engine::Render> renderer,
				   std::shared_ptr<MapGrid> map);
		~Task_Crowd();

		std::string Start();
		std::string NavTarget();
		TaskStages GetStages();
		
		int iterations_;
		float cam_pitch_;

		std::weak_ptr<RobotBase> agent_;
		std::shared_ptr<MapGrid> scene_;
		std::shared_ptr<Navigation> crowd_;
		std::shared_ptr<render_engine::Render> renderer_;
		
		render_engine::GLContext * ctx_;
		render_engine::Camera * main_camera_;
	};
}

#endif // TASK_EXAMPLE_H_