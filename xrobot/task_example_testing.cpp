#include "task_example_testing.h"

static std::string data_dir = "/home/ziyuli/Desktop/suncg";
static std::string metadata_models = "/home/ziyuli/Desktop/suncg/metadata/ModelCategoryMapping.csv";
static std::string house1 = "/home/ziyuli/Desktop/suncg/house/0911a85dffc1286db3a7aa1c9e4e2476/house.json";
static std::string crate03 = "./crate_0.3/crate.urdf";
static std::string door0 = "./door0/door.urdf";
static std::string door1 = "/home/ziyuli/model/door1/door.urdf";
static std::string wall = "./wall/floor.urdf";
static std::string wall1 = "/home/ziyuli/model/wall1/floor.urdf";
static std::string floor_test = "/floor/floor.urdf";
static std::string floor0 = "./floor0/floor.urdf";
static std::string floor1 = "/home/ziyuli/model/floor/floor.urdf";
static std::string floor2 = "./floor2/floor.urdf";
static std::string crate1 = "./crate_1/crate.urdf";
static std::string apple = "./apple/apple.urdf";
static std::string door_ani = "/home/ziyuli/model/door.urdf";
namespace xrobot
{


	Task_FollowRobot2::Task_FollowRobot2(render_engine::Render * renderer,
			Map * map) : iterations_(0),
					     scene_(map),
					     agent_(nullptr),
					     target_(nullptr),
					     renderer_(renderer),
					     ctx_(renderer->ctx_),
					     main_camera_(nullptr),
					     cam_pitch_(0),
					     inventory_(new Inventory(10)) {}

	Task_FollowRobot2::~Task_FollowRobot2() {}

	TaskStages Task_FollowRobot2::GetStages() {
		TaskStages stages;
		stages["idle"] = std::bind(&Task_FollowRobot2::Start, this);
		stages["NavTarget"] = std::bind(&Task_FollowRobot2::NavTarget, this);
		return stages;
	}

	std::string Task_FollowRobot2::Start() {
		renderer_->sunlight_.ambient = glm::vec3(0.02,0.02,0.02);
        renderer_->lighting_.exposure = 0.7f;
        renderer_->lighting_.indirect_strength = 0.25f;
        renderer_->lighting_.traceshadow_distance = 0.3f;
        renderer_->lighting_.propagation_distance = 0.3f;
        renderer_->lighting_.sample_factor = 0.9f;
        renderer_->lighting_.boost_ambient = 0.01f;
        renderer_->lighting_.shadow_bias_scale = 0.0003f;
        renderer_->lighting_.linear_voxelize = true;

		iterations_ = 0;
		scene_->ResetMap();
		scene_->ClearRules();
		scene_->CreateLabel("/home/ziyuli/model/pica_robot.urdf", "pica_robot");
		scene_->CreateLabel("/home/ziyuli/model/pica_obj.urdf", "stack");
		scene_->CreateLabel("/home/ziyuli/model/m0.urdf", "m0");
		scene_->CreateLabel("/home/ziyuli/model/m1.urdf", "m1");
		scene_->CreateLabel("/home/ziyuli/model/m2.urdf", "m2");
		scene_->CreateLabel("/home/ziyuli/model/m3.urdf", "m3");
		scene_->CreateLabel(door_ani, "door");
		scene_->CreateLabel(crate03, "crate");
		scene_->CreateSectionType(floor1, wall1, door0);
		scene_->GenerateTestFloorPlan(5, 5);

		inventory_->ResetNonPickableObjectTag();
	    inventory_->AddNonPickableObjectTag("Wall");
	    inventory_->AddNonPickableObjectTag("Floor");
	    inventory_->AddNonPickableObjectTag("Ceiling");
	    inventory_->AddNonPickableObjectTag("stack");
	    inventory_->AddNonPickableObjectTag("m0");
	    inventory_->AddNonPickableObjectTag("m1");
	    inventory_->AddNonPickableObjectTag("m2");
	    inventory_->AddNonPickableObjectTag("m3");

		// Load Obj
	    door_ = scene_->world_->LoadURDF(
	        door_ani,
	        btVector3(3, 0.1, 6),
	        btQuaternion(btVector3(0,0,1),1.57),
	        1.0f,
	        "door",
	        false
	    );
	   	door_->move(true);
	   	door_->DisableSleeping();

		Robot* obj = scene_->world_->LoadURDF(
	        crate03,
	        btVector3(2, 0, 2),
	        btQuaternion(btVector3(1,0,0),0),
	        1.0f,
	        "crate",
	        true
	    );
	   	obj->move(false);

	   	obj = scene_->world_->LoadURDF(
	        "/home/ziyuli/model/pica_obj.urdf",
	        btVector3(4, 0, 4),
	        btQuaternion(btVector3(1,0,0),0),
	        0.1f,
	        "stack",
	        true
	    );
	   	obj->move(false);

	   	obj = scene_->world_->LoadURDF(
	        "/home/ziyuli/model/m0.urdf",
	        btVector3(1, 0, 4),
	        btQuaternion(btVector3(1,0,0),0),
	        0.1f,
	        "m0",
	        true
	    );
	   	obj->move(false);

	   	obj = scene_->world_->LoadURDF(
	        "/home/ziyuli/model/m1.urdf",
	        btVector3(4, 0, 1),
	        btQuaternion(btVector3(1,0,0),0),
	        0.1f,
	        "m1",
	        true
	    );
	   	obj->move(false);

	   	obj = scene_->world_->LoadURDF(
	        "/home/ziyuli/model/m2.urdf",
	        btVector3(7, 0, 7),
	        btQuaternion(btVector3(1,0,0),0),
	        0.1f,
	        "m2",
	        true
	    );
	   	obj->move(false);

	   	obj = scene_->world_->LoadURDF(
	        "/home/ziyuli/model/m3.urdf",
	        btVector3(8, 0, 2),
	        btQuaternion(btVector3(1,0,0),0),
	        0.1f,
	        "m3",
	        true
	    );
	   	obj->move(false);

		// Load Target Robot
		target_ = scene_->world_->LoadURDF(
	        "/home/ziyuli/model/pica_robot.urdf",
	        btVector3(2, 0.01, 0),
	        btQuaternion(btVector3(1,0,0),0),
	        0.1f,
	        "pica_robot",
	        true
	    );
	   	target_->move(false);
	   	//target_->DisableSleeping();

	   	// Load Agent
	    agent_ = scene_->world_->LoadURDF(
	        "husky/husky.urdf",
	        btVector3(0,0.01,0),
	        btQuaternion(btVector3(-1,0,0),1.57)
	    );
	    agent_->move(true);
	    agent_->DisableSleeping();

	    // Create a Camera and Attach to the Agent
	    main_camera_ = scene_->world_->add_camera(vec3(0, 0, 0),
	    		vec3(0.3,1.3,0.0), (float) 4 / 3);
	    scene_->world_->attach_camera(main_camera_, agent_);
	    renderer_->Init(main_camera_);
	    scene_->world_->BulletStep();

	    door_angle_ = 0;

	    return "NavTarget";
	}

	std::string Task_FollowRobot2::NavTarget() {
		// TODO
		// Remove Freeze Parameter
		agent_->Freeze(!iterations_);

		if(ctx_->GetKeyPressUp())
            agent_->MoveForward(0.01f);

        if(ctx_->GetKeyPressDown())
            agent_->MoveBackward(0.01f);

        if(ctx_->GetKeyPressLeft())
            agent_->TurnLeft(0.01f);

        if(ctx_->GetKeyPressRight())
            agent_->TurnRight(0.01f);

        if(ctx_->GetKeyPressKP9())
            cam_pitch_ += 0.1f;

        if(ctx_->GetKeyPressKP6())
            cam_pitch_ -= 0.1f;

        if(ctx_->GetKeyPress0()) {
        	door_angle_ = glm::clamp(door_angle_ + 0.01f, 0.0f, 1.5f);
        	printf("angle: %f\n", door_angle_);
        }
        if(ctx_->GetKeyPress9()) {
        	door_angle_ = glm::clamp(door_angle_ - 0.01f, 0.0f, 1.5f);
        }
        door_->SetJointPosition(1, door_angle_, 1.0f, 1.0f, 20000.0f);

        // Pick
	    if(ctx_->GetKeyPress1()) {
	    	glm::vec3 fromPosition = main_camera_->Position;
	    	glm::vec3 toPosition = main_camera_->Front * 3.0f + fromPosition;
	    	agent_->PickUp(inventory_, fromPosition, toPosition);
	    	renderer_->BakeScene(scene_->world_);
	    }

	    // Put
	    if(ctx_->GetKeyPress2()) {
	    	glm::vec3 fromPosition = main_camera_->Position;
	    	glm::vec3 toPosition = main_camera_->Front * 3.0f + fromPosition;
	    	agent_->PutDown(inventory_, fromPosition, toPosition);
	    	renderer_->BakeScene(scene_->world_);
	    }

	    // Rotate
	    if(ctx_->GetKeyPress3()) {
	    	glm::vec3 fromPosition = main_camera_->Position;
	    	glm::vec3 toPosition = main_camera_->Front * 3.0f + fromPosition;
	    	agent_->RotateObject(1.57f, fromPosition, toPosition);
	    	renderer_->BakeScene(scene_->world_);
	    }


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
        // float s = 3 * sin(0 * 0.002f) + 4;
        // float c = 3 * cos(0 * 0.002f) + 4;

        // btTransform transform;
        // transform.setIdentity();
        // transform.setOrigin(btVector3(c, 0, s));
        // transform.setRotation(btQuaternion(btVector3(0,1,0), -iterations_ * 0.002f));
        // scene_->world_->SetTransformation(target_, transform);

        iterations_++;

        // Reset After N Steps
        // if(iterations_++ > 12000) 
        // 	return "idle";


        // Step Simulation and Renderer
        scene_->world_->BulletStep();   
        renderer_->StepRender(scene_->world_);
        ctx_->SwapBuffer();
        ctx_->PollEvent();

        return "NavTarget";
	}









	Task_GetAndPut::Task_GetAndPut(render_engine::Render * renderer,
			MapSuncg * map) : iterations_(0),
						     scene_(map),
						     agent_(nullptr),
						     renderer_(renderer),
						     ctx_(renderer->ctx_),
						     main_camera_(nullptr),
						     cam_pitch_(0),
						     inventory_(new Inventory(10)) {}

	Task_GetAndPut::~Task_GetAndPut() {
		delete inventory_;
	}

	TaskStages Task_GetAndPut::GetStages() {
		TaskStages stages;
		stages["idle"] = std::bind(&Task_GetAndPut::Start, this);
		stages["NavTarget"] = std::bind(&Task_GetAndPut::NavTarget, this);
		return stages;
	}

	std::string Task_GetAndPut::Start() {

		inventory_->ClearInventory();

		// Setup Lighting
		renderer_->sunlight_.direction = glm::normalize(glm::vec3(0.3, 1, 1));
        renderer_->lighting_.exposure = 0.7f;
        renderer_->lighting_.indirect_strength = 0.6f;
        renderer_->lighting_.traceshadow_distance = 0.3f;
        renderer_->lighting_.propagation_distance = 0.1f;
        renderer_->lighting_.boost_ambient = 0.2f;
        renderer_->lighting_.force_disable_shadow = true;

        // Load SUNCG Scene
        put_box_ = false;
        iterations_ = 0;
        scene_->ResetMap();
       	scene_->SetRemoveAll( kRemoveStairs | kRemoveDoor );
        scene_->LoadCategoryCSV(metadata_models.c_str());
       	scene_->AddPhysicalProperties("chair", {100, false});
	    scene_->AddPhysicalProperties("fruit_bowl", {100, false});
	    scene_->AddPhysicalProperties("trash_can", {100, false});
	    scene_->AddPhysicalProperties("coffee_machine", {100, false});
	    scene_->AddPhysicalProperties("bottle", {50, false});
	    scene_->AddPhysicalProperties("cup", {50, false});
	    scene_->LoadJSON(house1.c_str(), data_dir.c_str(), true);
	    scene_->SetMapSize(-8, -8, 6, 6);

	    // Config Inventory
	    inventory_->ResetNonPickableObjectTag();
	    inventory_->AddNonPickableObjectTag("Wall");
	    inventory_->AddNonPickableObjectTag("Ceiling");
	    inventory_->AddNonPickableObjectTag("Floor");

	   	// Load Agent
	    agent_ = scene_->world_->LoadURDF(
	        "husky/husky.urdf",
	        btVector3(-1,0.06,-1),
	        btQuaternion(btVector3(-1,0,0),1.57),
	        0.6f,
	        "agent",
	        true
	    );
	    agent_->root_part_->ChangeLinearDamping(0.01f);
	    agent_->move(true);
	    agent_->DisableSleeping();

	    // Create a Camera and Attach to the Agent
	    main_camera_ = scene_->world_->add_camera(vec3(0, 0, 0),
	    		vec3(0,1.3,0.0), (float) 4 / 3, 70, 0.02, 30);
	    scene_->world_->attach_camera(main_camera_, agent_);
	    renderer_->Init(main_camera_);
	    scene_->world_->BulletStep();

	    return "NavTarget";
	}

	std::string Task_GetAndPut::NavTarget() {
		// TODO
		// Remove Freeze Parameter
	    agent_->Freeze(!iterations_);
		if(ctx_->GetKeyPressUp())
            agent_->MoveForward(0.01f);

        if(ctx_->GetKeyPressDown())
            agent_->MoveBackward(0.01f);

        if(ctx_->GetKeyPressLeft())
            agent_->TurnLeft(0.01f);

        if(ctx_->GetKeyPressRight())
            agent_->TurnRight(0.01f);

	    if(ctx_->GetKeyPress0() && put_box_) {
	    	put_box_ = false;
	    }

	    if(ctx_->GetKeyPress9())
            scene_->world_->PrintCacheInfo();

	    if(ctx_->GetKeyPressKP0()) {
	    	inventory_->PrintInventory();
	    }

	    // Pick
	    if(ctx_->GetKeyPress1()) {
	    	glm::vec3 fromPosition = main_camera_->Position;
	    	glm::vec3 toPosition = main_camera_->Front * 4.0f + fromPosition;
	    	agent_->PickUp(inventory_, fromPosition, toPosition);
	    	renderer_->BakeScene(scene_->world_);
	    }

	    // Put
	    if(ctx_->GetKeyPress2()) {
	    	glm::vec3 fromPosition = main_camera_->Position;
	    	glm::vec3 toPosition = main_camera_->Front * 4.0f + fromPosition;
	    	agent_->PutDown(inventory_, fromPosition, toPosition);
	    	renderer_->BakeScene(scene_->world_);
	    }

	    // Rotate
	    if(ctx_->GetKeyPress3()) {
	    	glm::vec3 fromPosition = main_camera_->Position;
	    	glm::vec3 toPosition = main_camera_->Front * 4.0f + fromPosition;
	    	agent_->RotateObject(1.57f, fromPosition, toPosition);
	    	renderer_->BakeScene(scene_->world_);
	    }


        if(ctx_->GetKeyPressKP9())
            cam_pitch_ += 0.1f;

        if(ctx_->GetKeyPressKP6())
            cam_pitch_ -= 0.1f;

        cam_pitch_ = glm::clamp(cam_pitch_, -45.0f, 45.0f);
        scene_->world_->rotate_camera(main_camera_, cam_pitch_);
        iterations_++;

        // Step Simulation and Renderer
        scene_->world_->BulletStep();   
        renderer_->StepRender(scene_->world_);
        ctx_->SwapBuffer();
        ctx_->PollEvent();

        return "NavTarget";
	}





	Task_LoadScene::Task_LoadScene(render_engine::Render * renderer,
			Map * map) : iterations_(0),
					     scene_(map),
					     agent_(nullptr),
					     renderer_(renderer),
					     ctx_(renderer->ctx_),
					     main_camera_(nullptr),
					     cam_pitch_(0) {}

	Task_LoadScene::~Task_LoadScene() {}

	TaskStages Task_LoadScene::GetStages() {
		TaskStages stages;
		stages["idle"] = std::bind(&Task_LoadScene::Start, this);
		stages["NavTarget"] = std::bind(&Task_LoadScene::NavTarget, this);
		return stages;
	}

	std::string Task_LoadScene::Start() {
		renderer_->sunlight_.direction = glm::normalize(glm::vec3(0.5, 2.0, 1));
		renderer_->sunlight_.diffuse = glm::vec3(1.5,1.5,1.5);
		renderer_->sunlight_.ambient = glm::vec3(0.02,0.02,0.02);
        renderer_->lighting_.exposure = 3.0f;
        renderer_->lighting_.indirect_strength = 2.0f;

		iterations_ = 0;
		scene_->ResetMap();
		scene_->world_->set_world_size(-15, -7, 15, 7);

		// Load Scene
		scene_->world_->LoadOBJ(
	        "/home/ziyuli/model/s/sponza.obj",
	        btVector3(0,0,0),
	        btQuaternion(btVector3(1,0,0),0),
	        btVector3(0.007,0.007,0.007),
	        "scene",
	        0,
	        false,
	        true
	    );

	   	// Load Agent
	    agent_ = scene_->world_->LoadURDF(
	        "husky/husky.urdf",
	        btVector3(0,3.0,4),
	        btQuaternion(btVector3(-1,0,0),1.57),
	        2.0f
	    );
	    agent_->move(true);
	    agent_->Wake();

	    // Create a Camera and Attach to the Agent
	    main_camera_ = scene_->world_->add_camera(vec3(0, 0, 0),
	    		vec3(0,1.3,0.0), (float) 4 / 3, 90, 0.02, 50);
	    scene_->world_->attach_camera(main_camera_, agent_);
	    renderer_->Init(main_camera_);
	    scene_->world_->BulletStep();

	    return "NavTarget";
	}

	std::string Task_LoadScene::NavTarget() {
		// TODO
		// Remove Freeze Parameter
		speed_ = 5.0f;
	    vel_front_left_wheel_ = 0.0f;
	    vel_front_right_wheel_ = 0.0f;
	    vel_rear_left_wheel_ = 0.0f;
	    vel_rear_right_wheel_ = 0.0f;

		if(ctx_->GetKeyPressUp()) {
			vel_front_left_wheel_  += speed_;
            vel_front_right_wheel_ += speed_;
            vel_rear_left_wheel_   += speed_;
            vel_rear_right_wheel_  += speed_;
            agent_->Wake();
		}

        if(ctx_->GetKeyPressDown()) {
			vel_front_left_wheel_  -= speed_;
            vel_front_right_wheel_ -= speed_;
            vel_rear_left_wheel_   -= speed_;
            vel_rear_right_wheel_  -= speed_;
            agent_->Wake();
        }

        if(ctx_->GetKeyPressLeft()) {
			vel_front_left_wheel_  -= speed_;
            vel_front_right_wheel_ += speed_;
            vel_rear_left_wheel_   -= speed_;
            vel_rear_right_wheel_  += speed_;
            agent_->Wake();
        }

        if(ctx_->GetKeyPressRight()) {
			vel_front_left_wheel_  += speed_;
            vel_front_right_wheel_ -= speed_;
            vel_rear_left_wheel_   += speed_;
            vel_rear_right_wheel_  -= speed_;
            agent_->Wake();
        }

        cam_pitch_ = glm::clamp(cam_pitch_, -45.0f, 45.0f);
        scene_->world_->rotate_camera(main_camera_, cam_pitch_);

        Joint *j;
        j = agent_->joints_list_[2];
        j->SetJointMotorControlVelocity(vel_front_left_wheel_, 0.99, 200.0);
        j = agent_->joints_list_[3];
        j->SetJointMotorControlVelocity(vel_front_right_wheel_, 0.99, 200.0);
        j = agent_->joints_list_[4];
        j->SetJointMotorControlVelocity(vel_rear_left_wheel_, 0.99, 200.0);
        j = agent_->joints_list_[5];
        j->SetJointMotorControlVelocity(vel_rear_right_wheel_, 0.99, 200.0);

        // THIS IS ONLY FOR TESTING! 
        // Force to Switch a New Scene
        // if(ctx_->GetKeyPressSpace()) {
        //     ctx_->PollEvent();
        //     return "idle";
        // }


        // Step Simulation and Renderer
        scene_->world_->BulletStep();   
        renderer_->StepRender(scene_->world_);
        ctx_->SwapBuffer();
        ctx_->PollEvent();

        return "NavTarget";
	}
}