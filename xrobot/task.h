#ifndef TASK_H_
#define TASK_H_

#include <functional>
#include <vector>
#include <unordered_map>

#include "render_engine/gl_context.h"
#include "render_engine/render.h"
#include "map.h"
#include "map_suncg.h"
#include "lidar.h"

static std::string door0 = "./door0/door.urdf";
static std::string wall = "./wall/floor.urdf";
static std::string floor_test = "./floor/floor.urdf";
static std::string floor0 = "./floor0/floor.urdf";
static std::string floor1 = "./floor1/floor.urdf";
static std::string floor2 = "./floor2/floor.urdf";
static std::string crate1 = "./crate_1/crate.urdf";
static std::string crate03 = "./crate_0.3/crate.urdf";
static std::string apple = "./apple/apple.urdf";
static std::string data_dir = "/home/ziyuli/Desktop/suncg";
static std::string metadata_models = "/home/ziyuli/Desktop/suncg/metadata/ModelCategoryMapping.csv";
static std::string house0 = "/home/ziyuli/Desktop/suncg/house/7c16efebdfe46f3f14fa81abe500589c/house.json";

namespace xrobot
{
	class NavTask;
	typedef std::unordered_map<std::string,
							   std::function<std::string()>> TaskStages;

	class TaskBase {
	public:
		TaskBase() {};
		virtual ~TaskBase() {};
		virtual TaskStages GetStages() = 0;

		render_engine::GLContext * ctx_;
		render_engine::Render * renderer_;
		render_engine::Camera * c0_;
	};

	class NavTask0 : public TaskBase {
	public:

		NavTask0();
		NavTask0(render_engine::GLContext * ctx, 
				 render_engine::Render * rendere,
				 Map * map); // Keyboard Control
		~NavTask0();

		std::string Start();
		std::string NavTarget();
		TaskStages GetStages();

		int iterations_;
		int reset_count_;
		Map * scene_;
		Robot * agent_;
		Robot * target_;
		float cam_pitch_;
		render_engine::GLContext * ctx_;
		render_engine::Render * renderer_;
		render_engine::Camera * c0_;
	};

	class NavTask1 : public TaskBase {
	public:

		NavTask1();
		NavTask1(render_engine::GLContext * ctx, 
				 render_engine::Render * rendere,
				 Map * map); // Keyboard Control
		~NavTask1();

		std::string Start();
		std::string NavTarget();
		TaskStages GetStages();

		int iterations_;
		int reset_count_;
		Map * scene_;
		Robot * agent_;
		Lidar * lidar_;
		float cam_pitch_;
		render_engine::GLContext * ctx_;
		render_engine::Render * renderer_;
		render_engine::Camera * c0_;
	};

	class NavTask2 : public TaskBase {
	public:

		NavTask2();
		NavTask2(render_engine::GLContext * ctx,
		 		 render_engine::Render * renderer,
		 		 Map * map); // Keyboard Control
		~NavTask2();

		std::string Start();
		std::string NavTarget();
		TaskStages GetStages();

		int iterations_;
		int reset_count_;
		Map * scene_;
		Robot * agent_;
		Lidar * lidar_;
		float cam_pitch_;
		render_engine::GLContext * ctx_;
		render_engine::Render * renderer_;
		render_engine::Camera * c0_;
	};

	class NavTask3 : public TaskBase {
	public:

		NavTask3();
		NavTask3(render_engine::GLContext * ctx,
		 		 render_engine::Render * renderer,
		 		 Map * map); // Keyboard Control
		~NavTask3();

		std::string Start();
		std::string NavTarget();
		TaskStages GetStages();

		int iterations_;
		int reset_count_;
		Map * scene_;
		Robot * agent_;
		float cam_pitch_;
		render_engine::GLContext * ctx_;
		render_engine::Render * renderer_;
		render_engine::Camera * c0_;
	};

	class NavTask4 : public TaskBase {
	public:

		NavTask4();
		NavTask4(render_engine::GLContext * ctx,
		 		 render_engine::Render * renderer,
		 		 MapSuncg * map); // Keyboard Control
		~NavTask4();

		std::string Start();
		std::string NavTarget();
		TaskStages GetStages();

		int iterations_;
		int reset_count_;
		MapSuncg * scene_;
		Robot * agent_;
		float cam_pitch_;
		render_engine::GLContext * ctx_;
		render_engine::Render * renderer_;
		render_engine::Camera * c0_;

		float pos_0_ = 0.0f;
	    float pos_1_ = 0.0f;
	    float pos_2_ = 0.0f;
	    float pos_3_ = 0.0f;
	    float pos_4_ = 0.0f;
	    float pos_5_ = 0.0f;
	    float pos_6_ = 0.0f;
	    float pos_7_ = 0.0f;
	};

	class NavTask5 : public TaskBase {
	public:

		NavTask5();
		NavTask5(render_engine::GLContext * ctx,
		 		 render_engine::Render * renderer,
		 		 MapSuncg * map); // Keyboard Control
		~NavTask5();

		std::string Start();
		std::string NavTarget();
		TaskStages GetStages();
		
		int iterations_;
		int reset_count_;
		MapSuncg * scene_;
		Robot * agent_;
		float cam_pitch_;
		render_engine::GLContext * ctx_;
		render_engine::Render * renderer_;
		render_engine::Camera * c0_;

		float pos_0_ = 0.0f;
	    float pos_1_ = 0.0f;
	    float pos_2_ = 0.0f;
	    float pos_3_ = 0.0f;
	    float pos_4_ = 0.0f;
	    float pos_5_ = 0.0f;
	    float pos_6_ = 0.0f;
	    float pos_7_ = 0.0f;
	};
}

#endif // TASK_H_