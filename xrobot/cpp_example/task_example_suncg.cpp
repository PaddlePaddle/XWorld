#include "task_example_suncg.h"

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
								        ctx_(renderer->ctx_),
								        main_camera_(nullptr),
								        cam_pitch_(0) {
		// Initialize Inventory and Lidar
		inventory_= std::make_shared<Inventory>(1);
		lidar_ = std::make_shared<Lidar>(map->world_.get(), 180, 4.0f);

		// Init Visualization for Lidar
		renderer->InitDrawBatchRay(180);
	}

	Task_SUNCG::~Task_SUNCG() {}

	TaskStages Task_SUNCG::GetStages() {
		TaskStages stages;
		stages["idle"] = std::bind(&Task_SUNCG::Start, this);
		stages["NavTarget"] = std::bind(&Task_SUNCG::NavTarget, this);
		return stages;
	}

	std::string Task_SUNCG::Start() {

		// Adjust Lighting
		renderer_->sunlight_.direction = glm::vec3(0.3, 1, 1);
        renderer_->lighting_.exposure = 1.0f;
        renderer_->lighting_.indirect_strength = 1.5f;
        renderer_->lighting_.traceshadow_distance = 0.3f;
        renderer_->lighting_.propagation_distance = 0.3f;
        renderer_->lighting_.force_disable_shadow = true;
        renderer_->lighting_.use_ssr = false;

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
	        btVector3(-6,0.21,-1),
	        btQuaternion(btVector3(-1,0,0),1.57),
	        btVector3(0.6f, 0.6f, 0.6f),
	        "agent",
	        true
	    );

	   	// Spawn Object
	    obj_conv_ = scene_->world_->LoadRobot(
	    	object_with_action_0,
	    	btVector3(-5, 1, -2),
	        btQuaternion(btVector3(1,0,0),0),
	        btVector3(1, 1, 1),
	        "OBJ_Conv",
	        false
	    );

	   	if(auto agent_sptr = agent_.lock()) 
	   	{
	   		agent_sptr->move(true);
	    	agent_sptr->DisableSleeping();

		   	// Create a Camera and Attach to the Agent
		    main_camera_ = scene_->world_->add_camera(vec3(0, 0, 0),
		    		vec3(0.3,1.3,0.0), (float) 4 / 3);
		    scene_->world_->attach_camera(main_camera_, agent_sptr.get());
		}

	    renderer_->Init(main_camera_);
	    scene_->world_->BulletStep();
	    renderer_->BakeScene(scene_->world_.get());

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
		    	glm::vec3 fromPosition = main_camera_->Position;
		    	glm::vec3 toPosition = main_camera_->Front * 3.0f + fromPosition;
		    	agent_sptr->PickUp(inventory_, fromPosition, toPosition);
		    	renderer_->BakeScene(scene_->world_.get());
		    }

		    // Put
		    if(ctx_->GetKeyPressKP2()) {
		    	glm::vec3 fromPosition = main_camera_->Position;
		    	glm::vec3 toPosition = main_camera_->Front * 3.0f + fromPosition;
		    	agent_sptr->PutDown(inventory_, fromPosition, toPosition);
		    	renderer_->BakeScene(scene_->world_.get());
		    }
    	}

    	if(auto obj_conv_sptr = obj_conv_.lock()) {
	        if(ctx_->GetKeyPressKP4())
	            obj_conv_sptr->TakeAction(0);

	        if(ctx_->GetKeyPressKP5())
	            obj_conv_sptr->TakeAction(1);
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

	        j = agent_sptr->robot_data_.joints_list_[2];
	        j->SetJointMotorControlPosition(pos_0_, 0.1f, 1.0f,1000000.0f);
	        j = agent_sptr->robot_data_.joints_list_[3];
	        j->SetJointMotorControlPosition(pos_1_, 0.1f, 1.0f,1000000.0f);
	        j = agent_sptr->robot_data_.joints_list_[4];
	        j->SetJointMotorControlPosition(pos_2_, 0.1f, 1.0f,1000000.0f);
	        j = agent_sptr->robot_data_.joints_list_[5];
	        j->SetJointMotorControlPosition(pos_3_, 0.1f, 1.0f,1000000.0f);
	        j = agent_sptr->robot_data_.joints_list_[6];
	        j->SetJointMotorControlPosition(pos_4_, 0.1f, 1.0f,1000000.0f);
	        j = agent_sptr->robot_data_.joints_list_[7];
	        j->SetJointMotorControlPosition(pos_5_, 0.1f, 1.0f,1000000.0f);
	        j = agent_sptr->robot_data_.joints_list_[8];
	        j->SetJointMotorControlPosition(pos_6_, 0.1f, 1.0f,1000000.0f);
	        j = agent_sptr->robot_data_.joints_list_[10];
	        j->SetJointMotorControlPosition(pos_7_, 0.1f, 1.0f,1000000.0f);
	        j = agent_sptr->robot_data_.joints_list_[12];
	        j->SetJointMotorControlPosition(pos_7_, 0.1f, 1.0f,1000000.0f);
	    }

        // Check In-Range
        std::vector<ObjectAttributes> temp;
        scene_->world_->QueryObjectByLabel("fruit_bowl", temp);

        for(int i = 0; i < temp.size(); ++i)
        {
        	glm::vec3 aabb_min = temp[i].aabb_min - glm::vec3(1);
        	glm::vec3 aabb_max = temp[i].aabb_max + glm::vec3(1);

        	if(aabb_min.x < main_camera_->Position.x &&
        	   aabb_max.x > main_camera_->Position.x && 
        	   aabb_min.z < main_camera_->Position.z &&
        	   aabb_max.z > main_camera_->Position.z) 
        	{
        		// Check Object Center is Within 30 deg Viewing Angle
        		std::vector<ObjectDirections> temp_dir;
        		scene_->world_->QueryObjectDirectionByLabel("fruit_bowl", 
        				main_camera_->Front, main_camera_->Position, temp_dir);
        		if(temp_dir[i].dirs[0] < 0.52f)
        			return "idle";
        	}
        }

        // Find Front Direction in World Coordinate
        glm::vec3 front_vector = main_camera_->Front;
        front_vector.y = 0;
        front_vector = glm::normalize(front_vector);

        // Update Lidar
        std::vector<RayTestInfo> batch_raycast_result;
        lidar_->Update(front_vector,
                      glm::vec3(0,1,0),
                      main_camera_->Position - glm::vec3(0.0f,0.8f,0));
        batch_raycast_result = lidar_->GetResult();

        // Update Results in Renderer
        for (int i = 0; i < batch_raycast_result.size(); ++i)
        {
            if(batch_raycast_result[i].bullet_id < 0)
            {
                renderer_->UpdateRay(i, glm::vec3(0), glm::vec3(0));
            } else {
                renderer_->UpdateRay(i, 
                		main_camera_->Position - glm::vec3(0.0f,0.8f,0),
                        batch_raycast_result[i].pos);
            }
        }

        if(ctx_->GetKeyPressSpace()) {
        	ctx_->PollEvent();
        	return "idle";
        }

        // Reset After N Steps
        if(iterations_++ > 12000) 
        	return "idle";

        // Step Simulation and Renderer
        scene_->world_->BulletStep();   
        renderer_->StepRender(scene_->world_.get());
        ctx_->SwapBuffer();
        ctx_->PollEvent();

        return "NavTarget";
	}
}