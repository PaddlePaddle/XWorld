#include "task_example_obj.h"

namespace xrobot
{
	Task_TestScene::Task_TestScene(
		std::shared_ptr<render_engine::Render> renderer,
		std::shared_ptr<MapGrid> map) : iterations_(0),
								        scene_(map),
								        agent_(),
								        renderer_(renderer),
								        ctx_(renderer->GetContext()),
								        main_camera_(nullptr),
								        cam_pitch_(0) {}

	Task_TestScene::~Task_TestScene() {}

	TaskStages Task_TestScene::GetStages() {
		TaskStages stages;
		stages["idle"] = std::bind(&Task_TestScene::Start, this);
		stages["NavTarget"] = std::bind(&Task_TestScene::NavTarget, this);
		return stages;
	}

	std::string Task_TestScene::Start() {

		// Adjust Lighting
		renderer_->sunlight_.direction = glm::vec3(-2.5,1.5,2.5);
		renderer_->sunlight_.ambient = glm::vec3(0.2,0.2,0.2) * 2.0f;
		renderer_->sunlight_.diffuse = glm::vec3(255.0,230.0,150.0) * (1.5f / 255.0f);
		renderer_->lighting_.use_ssr = true;
        renderer_->lighting_.exposure = 1.5f;
        renderer_->lighting_.boost_ambient = 0.03f;
        renderer_->lighting_.indirect_strength = 1.0f;

        // Reset
        iterations_ = 0;
		scene_->ResetMap();

		// Load Scene
		auto obj = scene_->world_->LoadRobot(
	        bistro_in,
	        btVector3(0, 0, 0),
	        btQuaternion(btVector3(1,0,0),0),
	        btVector3(0.015f,0.015f,0.015f),
	        "scene",
	        true,0,true,true
	    );
	    scene_->world_->set_world_size(-2,-20,18,7);

	    // Spawn Agent
	    agent_ = scene_->world_->LoadRobot(
	        husky,
	        btVector3(0,0.55,0),
	        btQuaternion(btVector3(-1,0,0),1.57),
	        btVector3(1,1,1),
	        "Husky",
	        true
	    );

	    // Create a Camera and Attach to the Agent
		if(auto agent_sptr = agent_.lock()) {
			agent_sptr->move(true);
			agent_sptr->DisableSleeping();

		    main_camera_ = scene_->world_->add_camera(vec3(0, 0, 0),
		    		vec3(0.3,2.0,0.0), (float) 4 / 3);
		    scene_->world_->attach_camera(main_camera_, agent_sptr.get());
		}

		// Initialize the Scene
		renderer_->Init(main_camera_);
	    scene_->world_->BulletStep();
	    renderer_->BakeScene(scene_->world_.get());

	    return "NavTarget";
	}

	std::string Task_TestScene::NavTarget() {

		if(auto agent_sptr = agent_.lock()) {
			if(ctx_->GetKeyPressUp())
	            agent_sptr->MoveForward(5);

	        if(ctx_->GetKeyPressDown())
	            agent_sptr->MoveBackward(5);

	        if(ctx_->GetKeyPressLeft())
	            agent_sptr->TurnLeft(5);

	        if(ctx_->GetKeyPressRight())
	            agent_sptr->TurnRight(5);
    	}

        if(ctx_->GetKeyPressKP9())
            cam_pitch_ += 0.1f;

        if(ctx_->GetKeyPressKP6())
            cam_pitch_ -= 0.1f;

        cam_pitch_ = glm::clamp(cam_pitch_, -45.0f, 45.0f);
        scene_->world_->rotate_camera(main_camera_, cam_pitch_);

        // Reset After N Steps
        if(iterations_++ > 120000) 
        	return "idle";

        // Step Simulation and Renderer
        scene_->world_->BulletStep();   
        renderer_->StepRender(scene_->world_.get());
        ctx_->SwapBuffer();
        ctx_->PollEvent();

        return "NavTarget";
	}
}