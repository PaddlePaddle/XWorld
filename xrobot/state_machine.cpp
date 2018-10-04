#include "state_machine.h"

namespace xrobot 
{
	Task::Task(const std::string& name, TaskInterface& task)
	: name_(name) {

		auto stages = task.GetStages();
		for(auto it = stages.begin(); it != stages.end(); ++it)
		{
			stages_[it->first] = it->second;
		}

		current_stage_ = "idle";
	}

	Task::~Task() {

	}

	void Task::RunStage() {
		// TODO: why this asserts not triggered when current_stage_ is ""
		assert(stages_.find(current_stage_) != stages_.end());
		// assert(current_stage_ != "idle");

		current_stage_ = stages_[current_stage_]();		
	}

	void Task::RegisterStage(const std::string& stage, std::function<std::string()> func) {
		assert(!stage.empty());
		stages_[stage] = func;
	}

	void Task::Reset() {
		current_stage_ = "idle";
	}

	TaskGroup::TaskGroup(const std::string& name, const std::string& schedule) 
	: name_(name),
	  schedule_(schedule),
	  busy_task_(nullptr),
	  task_weights_(0),
	  task_list_(0) {}

	TaskGroup::~TaskGroup() {}

	void TaskGroup::AddTask(const std::string& task, TaskInterface& task_stage, double weight) {
		TaskPtr task_ptr = std::make_shared<Task>(task, task_stage);
		task_list_.push_back(task_ptr);	
		if(task_weights_.empty()) {
			task_weights_.push_back(weight);
	    } else {
	        task_weights_.push_back(task_weights_.back() + weight);
	    }
	}

	bool TaskGroup::IsIdle() {
		if(!busy_task_) {
			return true;
		} else {
			if(busy_task_->IsIdle()) {
				busy_task_ = nullptr;
				return true;
			} else {
				return false;
			}
		}
	}

	void TaskGroup::Reset() {
		busy_task_ = nullptr;
	}

	void TaskGroup::RunStage() {
		auto sample_task = [&]() {
			srand(time(NULL));
			int idx = -1;
	        if (schedule_ == "weighted") {
	        	double r = ((double) rand() / (RAND_MAX));
	        	idx = task_weights_.size() - 1;
	        	for (int i = 0; i < task_weights_.size(); ++i)
	        	{
	        		if(r > task_weights_[i])
	        			idx = i;
	        	}
	        } else {  // random
	            idx = rand() % task_list_.size();
	        }
	        return idx;
		};

		if(IsIdle()) {			
			// srand(time(NULL));

			busy_task_ = task_list_[sample_task()];
			busy_task_->Reset();
		}
		busy_task_->RunStage();
	}

}