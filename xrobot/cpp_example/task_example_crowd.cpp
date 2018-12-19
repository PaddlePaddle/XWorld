#include "task_example_crowd.h"

namespace xrobot
{
	Task_Crowd::Task_Crowd(
		std::shared_ptr<render_engine::Render> renderer,
		std::shared_ptr<MapGrid> map) : iterations_(0),
								        scene_(map),
								        renderer_(renderer),
								        ctx_(renderer->ctx_),
								        main_camera_(),
								        cam_pitch_(0) {
			// Initialize Navigation Feature
			crowd_ = std::make_shared<Navigation>(renderer->ctx_, map->world_.get());
		}

	Task_Crowd::~Task_Crowd() {}

	TaskStages Task_Crowd::GetStages() {
		TaskStages stages;
		stages["idle"] = std::bind(&Task_Crowd::Start, this);
		stages["NavTarget"] = std::bind(&Task_Crowd::NavTarget, this);
		return stages;
	}

	std::string Task_Crowd::Start() {

		// Adjust Lighting
		renderer_->sunlight_.ambient = glm::vec3(0.02,0.02,0.02);
		renderer_->lighting_.use_ssr = false;
        renderer_->lighting_.exposure = 0.4f;
        renderer_->lighting_.indirect_strength = 0.2f;

        // Reset
        iterations_ = 0;
		crowd_->Reset();
		scene_->ResetMap();

		// Load Assets and Scene
		scene_->LoadWallURDF(test_wall);
		scene_->CreateAndLoadTileURDF(test_floor);
		scene_->GenerateArena(5, 5);

		// Spawn Target
		std::weak_ptr<RobotBase> obj;
		agent_ = scene_->world_->LoadRobot(
	        husky,
	        glm::vec3(0,0.001,2),
	        glm::vec3(-1,0,0),
            1.57,
	        glm::vec3(1, 1, 1),
	        "husky",
	        true
	    );

	    obj = scene_->world_->LoadRobot(
	        crate1,
	        glm::vec3(1, 0, 0),
	        glm::vec3(1,0,0),
            0.0,
	        glm::vec3(1, 1, 1),
	        "crate1",
	        true
	    );
	    if(auto obj_sptr = obj.lock())
	   		obj_sptr->ignore_baking(false);


	    // Create a Camera and Attach to the Agent
		if(auto agent_sptr = agent_.lock()) {
	   		agent_sptr->ignore_baking(true);
	   		agent_sptr->DisableSleeping();

	    	// Create a Camera and Attach to the Agent
	    	main_camera_ = scene_->world_->add_camera(vec3(0, 0, 0),
	    		vec3(0.3,1.3,0.0), (float) 4 / 3);
	    	scene_->world_->attach_camera(main_camera_, agent_sptr.get());

		    renderer_->Init(main_camera_);
		    scene_->world_->BulletStep();

		    agent_sptr->UnFreeze();
		}

	    // Bake NavMesh
	    crowd_->SetBakeArea(glm::vec3(-2,-1,-2), glm::vec3(10,5,10));
	    crowd_->BakeNavMesh();
	    crowd_->carve_shapes_.resize(1);

		// Initialize the Scene
		renderer_->Init(main_camera_);
	    scene_->world_->BulletStep();
	    renderer_->BakeScene(scene_->world_.get());

	    return "NavTarget";
	}

	std::string Task_Crowd::NavTarget() {

		CarveShape cs;
		cs.position = glm::vec2(main_camera_->Position.x, main_camera_->Position.z);
		cs.radius = 1.3f;
		cs.moving = false;

		if(ctx_->GetKeyPressUp()) {
			if(auto agent_sptr = agent_.lock()) {
	            agent_sptr->MoveForward(1.0f);
	            cs.moving = true;
	        }
		}

        if(ctx_->GetKeyPressDown()) {
        	if(auto agent_sptr = agent_.lock()) {
	            agent_sptr->MoveBackward(1.0f);
	            cs.moving = true;
	        }
        }

        if(ctx_->GetKeyPressLeft()) {
        	if(auto agent_sptr = agent_.lock()) {
	            agent_sptr->TurnLeft(1.0f);
	        	cs.moving = true;
	        }
        }

        if(ctx_->GetKeyPressRight()) {
        	if(auto agent_sptr = agent_.lock()) {
	            agent_sptr->TurnRight(1.0f);
	        	cs.moving = true;
	        }
        }

        if(ctx_->GetKeyPressKP9())
            cam_pitch_ += 0.1f;

        if(ctx_->GetKeyPressKP6())
            cam_pitch_ -= 0.1f;

        cam_pitch_ = glm::clamp(cam_pitch_, -45.0f, 45.0f);
        scene_->world_->rotate_camera(main_camera_, cam_pitch_);

        // Generate New Agent
		if(iterations_ % 400 == 0) {
			crowd_->SpawnAgent(glm::vec3(0,0,0), glm::quat(1,0,0,0), crate03, "moving_agent");
			crowd_->crowd_.back().AssignTarget(glm::vec3(8,0,8));
		}

		crowd_->carve_shapes_[0] = cs;
		crowd_->Update();
		crowd_->KillAgentOnceArrived();

        // This is just for testing
        if(ctx_->GetKeyPressSpace()) {
            ctx_->PollEvent();
            printf("Reset the Scene : %d\n", scene_->world_->reset_count_);
            return "idle";
        }

        // Step Simulation and Renderer
        scene_->world_->BulletStep();   
        renderer_->StepRender(scene_->world_.get());
        ctx_->SwapBuffer();
        ctx_->PollEvent();


        // Reset After N Steps
        if(iterations_++ > 12000) 
        	return "idle";

        return "NavTarget";
	}
}
