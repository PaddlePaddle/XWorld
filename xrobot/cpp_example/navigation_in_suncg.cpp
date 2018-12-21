#include "navigation_in_suncg.h"

namespace xrobot
{
	Task_SUNCG::Task_SUNCG(
		std::shared_ptr<render_engine::Render> renderer,
		std::shared_ptr<MapSuncg> map) : iterations_(0),
								        scene_(map),
								        agent_(),
								        lidar_(nullptr),
								        inventory_(nullptr),
								        renderer_(renderer),
								        ctx_(renderer->GetContext()),
								        main_camera_(nullptr),
								        cam_pitch_(0) {
		// Initialize Inventory and Lidar
		inventory_= std::make_shared<Inventory>(1);
		lidar_ = std::make_shared<Lidar>(map->world_.get(), 90, 4.0f);

		// Init Visualization for Lidar
		renderer->InitDrawBatchRays(90);
	}

	Task_SUNCG::~Task_SUNCG() {}

	TaskStages Task_SUNCG::GetStages() {
		TaskStages stages;
		stages["idle"] = std::bind(&Task_SUNCG::Start, this);
		stages["NavTarget"] = std::bind(&Task_SUNCG::NavTarget, this);
		return stages;
	}

	std::string Task_SUNCG::Start() {

        // Reset
        iterations_ = 0;
		scene_->ResetMap();

		// Reset Joint Pose
	    pos_0_ = 0.0f;
	    pos_1_ = 0.0f;
	    pos_2_ = 0.0f;
	    pos_3_ = 0.0f;
	    pos_4_ = 0.0f;
	    pos_5_ = 0.0f;
	    pos_6_ = 0.0f;
	    pos_7_ = 0.0f;

		// Load Assets and Scene
		scene_->SetRemoveAll( kRemoveStairs );
        scene_->LoadCategoryCSV(suncg_meta.c_str());
        scene_->world_->UpdatePickableList("kettle", true);
	    scene_->world_->UpdatePickableList("knife_rack", true);
	    scene_->world_->UpdatePickableList("coffee_machine", true);
        scene_->AddPhysicalProperties("chair", {100, false});
	    scene_->AddPhysicalProperties("fruit_bowl", {100, false});
	    scene_->AddPhysicalProperties("trash_can", {100, false});
	    scene_->AddPhysicalProperties("coffee_machine", {100, false});
	    scene_->AddPhysicalProperties("knife_rack", {100, false});
	    scene_->AddPhysicalProperties("knife", {10, false});
	    scene_->AddPhysicalProperties("kettle", {100, false});
	    scene_->LoadJSON(suncg_house.c_str(), suncg_dir.c_str(), true);
	    scene_->SetMapSize(-8, -8, 6, 6);

		// Spawn Agent
	   	agent_ = scene_->world_->LoadRobot(
	        husky_kuka,
	        glm::vec3(-6,0.21,-1),
	        glm::vec3(-1,0,0),
            1.57,
	        glm::vec3(0.6f, 0.6f, 0.6f),
	        "agent",
	        true
	    );

	   	if(auto agent_sptr = agent_.lock()) 
	   	{
	   		agent_sptr->ignore_baking(true);
	    	agent_sptr->DisableSleeping();

		   	// Create a Camera and Attach to the Agent
		    main_camera_ = scene_->world_->add_camera(vec3(0, 0, 0),
		    		vec3(0.3,1.3,0.0), (float) 4 / 3);
		    scene_->world_->attach_camera(main_camera_, agent_sptr.get());
		}

	    scene_->world_->BulletStep();
	    renderer_->BakeGI(scene_->world_.get());

	    return "NavTarget";
	}

	std::string Task_SUNCG::NavTarget() {

		if(auto agent_sptr = agent_.lock()) {
			if(ctx_->GetKeyPressUp())
	            agent_sptr->MoveForward(2);

	        if(ctx_->GetKeyPressDown())
	            agent_sptr->MoveBackward(2);

	        if(ctx_->GetKeyPressLeft())
	            agent_sptr->TurnLeft(2);

	        if(ctx_->GetKeyPressRight())
	            agent_sptr->TurnRight(2);

	        // Pick
		    if(ctx_->GetKeyPressKP1()) {
		    	glm::vec3 fromPosition = main_camera_->position_;
		    	glm::vec3 toPosition = main_camera_->front_ * 3.0f + fromPosition;
		    	agent_sptr->PickUp(inventory_, fromPosition, toPosition);
		    	renderer_->BakeGI(scene_->world_.get());
		    }

		    // Put
		    if(ctx_->GetKeyPressKP2()) {
		    	glm::vec3 fromPosition = main_camera_->position_;
		    	glm::vec3 toPosition = main_camera_->front_ * 3.0f + fromPosition;
		    	agent_sptr->PutDown(inventory_, fromPosition, toPosition);
		    	renderer_->BakeGI(scene_->world_.get());
		    }
    	}

        if(ctx_->GetKeyPressKP9())
            cam_pitch_ += 0.1f;

        if(ctx_->GetKeyPressKP6())
            cam_pitch_ -= 0.1f;

        if(ctx_->GetKeyPress1())
            pos_0_ += 0.0025f;

        if(ctx_->GetKeyPress2())
            pos_0_ -= 0.0025f;

        if(ctx_->GetKeyPress3())
            pos_1_ += 0.0025f;

        if(ctx_->GetKeyPress4())
            pos_1_ -= 0.0025f;

        if(ctx_->GetKeyPress5())
            pos_2_ += 0.0025f;

        if(ctx_->GetKeyPress6())
            pos_2_ -= 0.0025f;

        if(ctx_->GetKeyPress7())
            pos_3_ += 0.0025f;

        if(ctx_->GetKeyPress8())
            pos_3_ -= 0.0025f;

        if(ctx_->GetKeyPress9())
            pos_4_ += 0.0025f;

        if(ctx_->GetKeyPress0())
            pos_4_ -= 0.0025f;

        if(ctx_->GetKeyPressKP7())
            pos_7_ = 0.05f;

        if(ctx_->GetKeyPressKP8())
            pos_7_ = 0.005f;

        cam_pitch_ = glm::clamp(cam_pitch_, -45.0f, 45.0f);
        scene_->world_->rotate_camera(main_camera_, cam_pitch_);

        pos_0_ = glm::clamp(pos_0_, -1.0f, 1.0f);
        pos_1_ = glm::clamp(pos_1_, -1.0f, 1.0f);
        pos_2_ = glm::clamp(pos_2_, -1.0f, 1.0f);
        pos_3_ = glm::clamp(pos_3_, -1.0f, 1.0f);
        pos_4_ = glm::clamp(pos_4_, -1.0f, 1.0f);
        pos_5_ = glm::clamp(pos_5_, -1.0f, 1.0f);
        pos_6_ = glm::clamp(pos_6_, -1.0f, 1.0f);

        // Control Desired Joints
        std::shared_ptr<Joint> j;
        if(auto agent_sptr = agent_.lock()) 
	   	{

	        j = agent_sptr->joints_[2];
	        j->SetJointMotorControlPosition(pos_0_, 0.1f, 1.0f,1000000.0f);
	        j = agent_sptr->joints_[3];
	        j->SetJointMotorControlPosition(pos_1_, 0.1f, 1.0f,1000000.0f);
	        j = agent_sptr->joints_[4];
	        j->SetJointMotorControlPosition(pos_2_, 0.1f, 1.0f,1000000.0f);
	        j = agent_sptr->joints_[5];
	        j->SetJointMotorControlPosition(pos_3_, 0.1f, 1.0f,1000000.0f);
	        j = agent_sptr->joints_[6];
	        j->SetJointMotorControlPosition(pos_4_, 0.1f, 1.0f,1000000.0f);
	        j = agent_sptr->joints_[7];
	        j->SetJointMotorControlPosition(pos_5_, 0.1f, 1.0f,1000000.0f);
	        j = agent_sptr->joints_[8];
	        j->SetJointMotorControlPosition(pos_6_, 0.1f, 1.0f,1000000.0f);
	        j = agent_sptr->joints_[10];
	        j->SetJointMotorControlPosition(pos_7_, 0.1f, 1.0f,1000000.0f);
	        j = agent_sptr->joints_[12];
	        j->SetJointMotorControlPosition(pos_7_, 0.1f, 1.0f,1000000.0f);
	    }

        // Find Front Direction in World Coordinate
        glm::vec3 front_vector = main_camera_->front_;
        front_vector.y = 0;
        front_vector = glm::normalize(front_vector);

        // Update Lidar
        std::vector<RayTestInfo> batch_raycast_result;
        lidar_->Update(front_vector,
                      glm::vec3(0,1,0),
                      main_camera_->position_ - glm::vec3(0.0f,0.8f,0));
        batch_raycast_result = lidar_->GetResult();

        // Update Results in Renderer
        for (int i = 0; i < batch_raycast_result.size(); ++i)
        {
            if(batch_raycast_result[i].bullet_id < 0)
            {
                renderer_->UpdateRay(i, glm::vec3(0), glm::vec3(0));
            } else {
                renderer_->UpdateRay(i, 
                		main_camera_->position_ - glm::vec3(0.0f,0.8f,0),
                        batch_raycast_result[i].pos);
            }
        }

        if(ctx_->GetKeyPressSpace()) {
        	ctx_->PollEvent();
        	return "idle";
        }

        // Step Simulation and Renderer
        scene_->world_->BulletStep();   
        renderer_->StepRender(scene_->world_.get());
        ctx_->SwapBuffer();
        ctx_->PollEvent();

        return "NavTarget";
	}
}
