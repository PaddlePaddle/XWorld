#ifndef TASK_H_
#define TASK_H_

#include <functional>
#include <unordered_map>

#include "render_engine/gl_context.h"
#include "render_engine/render.h"

namespace xrobot
{
	typedef std::unordered_map<std::string,
							   std::function<std::string()>> TaskStages;

	class TaskInterface {
	public:
		virtual ~TaskInterface() {};
		virtual TaskStages GetStages() = 0;
		
		render_engine::GLContext * ctx_;
		render_engine::Render * renderer_;
		render_engine::Camera * main_camera_;
	};

}

#endif // TASK_H_