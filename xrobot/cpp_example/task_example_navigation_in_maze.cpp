#include "task_example_navigation_in_maze.h"

namespace xrobot
{
	Task_NavToObject::Task_NavToObject(
		std::shared_ptr<render_engine::Render> renderer,
		std::shared_ptr<MapGrid> map) : iterations_(0),
								        scene_(map),
								        agent_(),
								        lidar_(nullptr),
								        renderer_(renderer),
								        ctx_(renderer->GetContext()),
								        main_camera_(nullptr),
								        cam_pitch_(0) {

		// Initialize Lidar
		lidar_ = std::make_shared<Lidar>(map->world_.get(), 180, 4.0f);

		// Initialize Visualization for Lidar
		renderer->InitDrawBatchRay(180);						  
	}

	Task_NavToObject::~Task_NavToObject() {}

	TaskStages Task_NavToObject::GetStages() {
		TaskStages stages;
		stages["idle"] = std::bind(&Task_NavToObject::Start, this);
		stages["NavTarget"] = std::bind(&Task_NavToObject::NavTarget, this);
		return stages;
	}

	std::string Task_NavToObject::Start() {

		// Adjust Lighting
		renderer_->sunlight_.ambient = glm::vec3(0.05,0.05,0.05);
		renderer_->lighting_.use_ssr = false;
        renderer_->lighting_.exposure = 0.4f;
        renderer_->lighting_.indirect_strength = 0.2f;

        // Reset
        iterations_ = 0;
		scene_->ResetMap();

		// Load Assets
		scene_->world_->LoadMetadata(test_meta.c_str());
		scene_->world_->AssignTag(apple, "apple");
		scene_->CreateAndLoadTileURDF(test_floor);
		scene_->LoadWallURDF(test_wall);
		scene_->CreateAndLoadKeyURDF(key_red, "red");
		scene_->CreateAndLoadKeyURDF(key_blue, "blue");
		scene_->CreateAndLoadKeyURDF(key_yellow, "yellow");
		scene_->LoadUnlockedDoorJSON(door_green);
		scene_->CreateAndLoadLockedDoorJSON(door_red);
		scene_->CreateAndLoadLockedDoorJSON(door_blue);
	    scene_->CreateAndLoadLockedDoorJSON(door_yellow);
	    scene_->CreateAndLoadObjectFILE(crate1, "crate");
	    scene_->CreateAndLoadObjectFILE(crate03, "crate");
	    scene_->CreateAndLoadObjectFILE(apple, "apple");

	    // Generate Maze Layout
	    glm::vec3 startPosition = scene_->GenerateLayout(5, 5, 3, 2);
	    scene_->ResolvePath();

	    // Spawn Object in Maze
	    for(int i = 0; i < 5; ++i) 
	    	scene_->SpawnSingleObject(0);
	    for(int i = 0; i < 5; ++i) 
	    	scene_->SpawnSingleObject(1);
	    for(int i = 0; i < 5; ++i) 
	    	scene_->SpawnStackOfObjects(1,0,2);
	    scene_->GenerateObjects();

	    // Spawn Agent
	    agent_ = scene_->world_->LoadRobot(
	        husky,
	        glm::vec3(startPosition.x,0.01,startPosition.z),
            glm::vec3(-1, 0, 0), 
            1.57,
	        glm::vec3(1,1,1),
	        "Husky",
	        true
	    );

	    // Create a Camera and Attach to the Agent
		if(auto agent_sptr = agent_.lock()) {
			agent_sptr->ignore_baking(true);
			agent_sptr->DisableSleeping();

		    main_camera_ = scene_->world_->add_camera(vec3(0, 0, 0),
		    		vec3(0.3,1.5,0.0), (float) 4 / 3);
		    scene_->world_->attach_camera(main_camera_, agent_sptr.get());
		}

		// Initialize the Scene
		renderer_->Init(main_camera_);
	    scene_->world_->BulletStep();
	    renderer_->BakeScene(scene_->world_.get());

	    return "NavTarget";
	}

	std::string Task_NavToObject::NavTarget() {

		ctx_->PollEvent();
		if(ctx_->GetKeyPressSpace()) {
			return "idle";
		}

		if(auto agent_sptr = agent_.lock()) {
			if(ctx_->GetKeyPressUp())
	            agent_sptr->MoveForward(2);

	        if(ctx_->GetKeyPressDown())
	            agent_sptr->MoveBackward(2);

	        if(ctx_->GetKeyPressLeft())
	            agent_sptr->TurnLeft(2);

	        if(ctx_->GetKeyPressRight())
	            agent_sptr->TurnRight(2);
    	}

        if(ctx_->GetKeyPressKP9())
            cam_pitch_ += 0.1f;

        if(ctx_->GetKeyPressKP6())
            cam_pitch_ -= 0.1f;

        cam_pitch_ = glm::clamp(cam_pitch_, -45.0f, 45.0f);
        scene_->world_->rotate_camera(main_camera_, cam_pitch_);

        // Check In-Range
        std::vector<ObjectAttributes> temp;
        scene_->world_->QueryObjectByLabel("apple", temp);

        for(int i = 0; i < temp.size(); ++i)
        {
        	glm::vec3 aabb_min = temp[i].aabb_min - glm::vec3(1);
        	glm::vec3 aabb_max = temp[i].aabb_max + glm::vec3(1);

        	if(aabb_min.x < main_camera_->Position.x &&
        	   aabb_max.x > main_camera_->Position.x && 
        	   aabb_min.z < main_camera_->Position.z &&
        	   aabb_max.z > main_camera_->Position.z) 
        	{
        		// Check Aim
        		glm::vec3 fromPosition = main_camera_->Position;
        		glm::vec3 toPosition = main_camera_->Front * 3.0f + fromPosition;

        		RayTestInfo res;
        		scene_->world_->RayTest(fromPosition, toPosition, res);

        		if(res.bullet_id == temp[i].bullet_id)
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

        // Reset After N Steps
        // if(iterations_++ > 12000) 
        // 	return "idle";

        // Step Simulation and Renderer
        scene_->world_->BulletStep();   
        renderer_->StepRender(scene_->world_.get());
        ctx_->SwapBuffer();
        ctx_->PollEvent();

        return "NavTarget";
	}
}
