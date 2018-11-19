#ifndef TASK_EXAMPLE_NAVIGATION_IN_MAZE_H_
#define TASK_EXAMPLE_NAVIGATION_IN_MAZE_H_

#include <vector>
#include <functional>
#include <unordered_map>

#include "render_engine/render.h"
#include "map_grid.h"
#include "map_suncg.h"
#include "lidar.h"
#include "task.h"
#include "navigation.h"
#include "assets.h"

namespace xrobot
{
	typedef std::unordered_map<std::string,
							   std::function<std::string()>> TaskStages;

	class Task_NavToObject : public TaskInterface {
	public:

		Task_NavToObject(std::shared_ptr<render_engine::Render> renderer,
				   		 std::shared_ptr<MapGrid> map);
		~Task_NavToObject();

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
}

#endif // TASK_EXAMPLE_NAVIGATION_IN_MAZE_H_