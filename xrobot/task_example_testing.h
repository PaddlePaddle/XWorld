#ifndef TASK_EXAMPLE_TESTING_H_
#define TASK_EXAMPLE_TESTING_H_

#include <vector>
#include <functional>
#include <unordered_map>

#include "render_engine/render.h"
#include "map.h"
#include "map_suncg.h"
#include "lidar.h"
#include "task.h"
#include "task_example.h"
#include "inventory.h"

namespace xrobot
{
	typedef std::unordered_map<std::string,
							   std::function<std::string()>> TaskStages;


	class Task_GetAndPut : public TaskInterface {
	public:

		Task_GetAndPut(render_engine::Render * renderer, MapSuncg * map);
		~Task_GetAndPut();

		std::string Start();
		std::string NavTarget();
		TaskStages GetStages();
		
		int iterations_;
		Inventory * inventory_;
		MapSuncg * scene_;
		Robot * agent_;
		float cam_pitch_;
		render_engine::GLContext * ctx_;
		render_engine::Render * renderer_;
		render_engine::Camera * main_camera_;

		bool put_box_ = false;
		float speed_ = 1.0f;
	    float vel_front_left_wheel_ = 0.0f;
	    float vel_front_right_wheel_ = 0.0f;
	    float vel_rear_left_wheel_ = 0.0f;
	    float vel_rear_right_wheel_ = 0.0f;
	};

	class Task_LoadScene : public TaskInterface {
	public:

		Task_LoadScene(render_engine::Render * renderer, Map * map);
		~Task_LoadScene();

		std::string Start();
		std::string NavTarget();
		TaskStages GetStages();
		
		int iterations_;
		Map * scene_;
		Robot * agent_;
		float cam_pitch_;
		render_engine::GLContext * ctx_;
		render_engine::Render * renderer_;
		render_engine::Camera * main_camera_;

		float speed_ = 1.0f;
	    float vel_front_left_wheel_ = 0.0f;
	    float vel_front_right_wheel_ = 0.0f;
	    float vel_rear_left_wheel_ = 0.0f;
	    float vel_rear_right_wheel_ = 0.0f;
	};

	class Task_FollowRobot2 : public TaskInterface {
	public:

		Task_FollowRobot2(render_engine::Render * renderer, Map * map);
		~Task_FollowRobot2();

		std::string Start();
		std::string NavTarget();
		TaskStages GetStages();

		int iterations_;
		Inventory * inventory_;
		Map * scene_;
		Robot * door_;
		Robot * agent_;
		Robot * target_;
		render_engine::GLContext * ctx_;
		render_engine::Render * renderer_;
		render_engine::Camera * main_camera_;
		float cam_pitch_;
		float door_angle_;
	};

}
#endif // TASK_EXAMPLE_TESTING_H_