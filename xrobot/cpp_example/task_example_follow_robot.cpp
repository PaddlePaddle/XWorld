#include "task_example_follow_robot.h"

namespace xrobot
{
	Task_FollowRobot::Task_FollowRobot(
		std::shared_ptr<render_engine::Render> renderer,
		std::shared_ptr<MapGrid> map) : iterations_(0),
								        scene_(map),
								        agent_(),
								        target_(),
								        renderer_(renderer),
								        ctx_(renderer->GetContext()),
								        main_camera_(nullptr),
								        cam_pitch_(0) {}

	Task_FollowRobot::~Task_FollowRobot() {}

	TaskStages Task_FollowRobot::GetStages() {
		TaskStages stages;
		stages["idle"] = std::bind(&Task_FollowRobot::Start, this);
		stages["NavTarget"] = std::bind(&Task_FollowRobot::NavTarget, this);
		return stages;
	}

	std::string Task_FollowRobot::Start() {

		// Adjust Lighting
		renderer_->sunlight_.ambient = glm::vec3(0.05,0.05,0.05);
		renderer_->lighting_.use_ssr = false;
        renderer_->lighting_.exposure = 0.4f;
        renderer_->lighting_.indirect_strength = 0.2f;

        // Reset
        iterations_ = 0;
		scene_->ResetMap();

		// Load Assets and Scene
		scene_->LoadWallURDF(test_wall);
		scene_->CreateAndLoadTileURDF(test_floor);
		scene_->GenerateArena(5, 5);

		// Spawn Target
		scene_->world_->AssignTag(r2d2, "r2d2");
		target_ = scene_->world_->LoadRobot(
	        r2d2,
	        glm::vec3(2, 0.01, 0),
            glm::vec3(1, 0, 0),
            0.0,
	        glm::vec3(0.01f,0.01f,0.01f),
	        "r2d2",
	        true
	    );
	    if(auto target_sptr = target_.lock()) {
	    	target_sptr->ignore_baking(true);
	   		target_sptr->DisableSleeping();
	    }

	    // Spawn Agent
	    agent_ = scene_->world_->LoadRobot(
	        husky,
	        glm::vec3(0,0.01,0),
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

	std::string Task_FollowRobot::NavTarget() {

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
        scene_->world_->QueryObjectByLabel("r2d2", temp);

        for(int i = 0; i < temp.size(); ++i)
        {
        	glm::vec3 aabb_min = temp[i].aabb_min - glm::vec3(1);
        	glm::vec3 aabb_max = temp[i].aabb_max + glm::vec3(1);

        	if(aabb_min.x < main_camera_->Position.x &&
        	   aabb_max.x > main_camera_->Position.x && 
        	   aabb_min.z < main_camera_->Position.z &&
        	   aabb_max.z > main_camera_->Position.z) 
        	{
        		return "idle";
        	}
        }

        // Move Target Robot
        float s = 3 * sin(iterations_ * 0.002f) + 4;
        float c = 3 * cos(iterations_ * 0.002f) + 4;

        btTransform transform;
        transform.setIdentity();
        transform.setOrigin(btVector3(c, 0, s));
        transform.setRotation(btQuaternion(btVector3(0,1,0), -iterations_ * 0.002f));
        scene_->world_->SetTransformation(target_, transform);


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
