#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

#include <functional>
#include <vector>
#include <unordered_map>

#include "task.h"
#include "map.h"
#include "render_engine/gl_context.h"
#include "render_engine/render.h"

namespace xrobot
{
	class Task
	{
	public:
		Task(const std::string& name, TaskInterface& task);
		~Task();
		
		void RunStage();
		void RegisterStage(const std::string& stage, std::function<std::string()> func);
		void Reset();
		std::string GetCurrentStage() const { return current_stage_; };
		std::string Name() { return name_; }
		bool IsIdle() { return current_stage_ == "idle"; }
	private:
		std::unordered_map<std::string, std::function<std::string()>> stages_;
		std::string name_;
		std::string current_stage_;
	};

	typedef std::shared_ptr<Task> TaskPtr;

	class TaskGroup
	{
	public:
		TaskGroup(const std::string& name, const std::string& schedule = "");
		~TaskGroup();

		void AddTask(const std::string& task, TaskInterface& task_stage, double weight = 0.0);
		bool IsIdle();
		void Reset();
		void RunStage();
		std::string Name() { return name_; }
	private:
		TaskPtr busy_task_;
		std::string name_;
		std::string schedule_;
		std::vector<TaskPtr> task_list_;
		std::vector<double> task_weights_;
	};
}

#endif // STATE_MACHINE_H_