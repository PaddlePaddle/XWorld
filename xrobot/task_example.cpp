#include "task_example.h"

static std::string door0 = "./door0/door.urdf";
static std::string door1 = "/home/ziyuli/model/door1/door.urdf";
static std::string wall = "./wall/floor.urdf";
static std::string wall1 = "/home/ziyuli/model/wall1/floor.urdf";
static std::string floor_test = "/floor/floor.urdf";
static std::string floor0 = "./floor0/floor.urdf";
static std::string floor1 = "/home/ziyuli/model/floor/floor.urdf";
static std::string floor2 = "./floor2/floor.urdf";
static std::string crate1 = "./crate_1/crate.urdf";
static std::string crate03 = "./crate_0.3/crate.urdf";
static std::string apple = "./apple/apple.urdf";
static std::string data_dir = "/home/ziyuli/Desktop/suncg";
static std::string metadata_models = "/home/ziyuli/Desktop/suncg/metadata/ModelCategoryMapping.csv";
static std::string house0 = "/home/ziyuli/Desktop/suncg/house/7c16efebdfe46f3f14fa81abe500589c/house.json";
static std::string door_ani = "/home/ziyuli/model/door.urdf";
static std::string conv_action = "/home/ziyuli/model/test.json";
static std::string anim_action = "/home/ziyuli/model/door.json";

namespace xrobot
{

	//=======================================Task 0=============================

	Task_FollowRobot::Task_FollowRobot(render_engine::Render * renderer,
			Map * map) : iterations_(0),
					     scene_(map),
					     agent_(nullptr),
					     target_(nullptr),
					     renderer_(renderer),
					     ctx_(renderer->ctx_),
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
		renderer_->sunlight_.ambient = glm::vec3(0.05,0.05,0.05);
        renderer_->lighting_.exposure = 1.0f;
        renderer_->lighting_.indirect_strength = 0.3f;
		iterations_ = 0;
		scene_->ResetMap();
		scene_->ClearRules();
		scene_->CreateLabel("r2d2/r2d2.urdf", "r2d2");
		scene_->CreateSectionType(floor1, wall1, door0);
		scene_->GenerateTestFloorPlan(5, 5);

		// Load Target Robot
		target_ = scene_->world_->LoadRobot(
	        "r2d2/r2d2.urdf",
	        btVector3(2, 0.01, 0),
	        btQuaternion(btVector3(1,0,0),0),
	        btVector3(0.01f,0.01f,0.01f),
	        "r2d2",
	        true
	    );
	   	target_->move(true);
	   	target_->DisableSleeping();

	   	// Load Agent
	   	agent_ = scene_->world_->LoadRobot(
	        "husky/husky.urdf",
	        btVector3(0,0.01,0),
	        btQuaternion(btVector3(-1,0,0),1.57),
	        btVector3(1,1,1),
	        "husky",
	        true
	    );
	    agent_->move(true);
	    agent_->DisableSleeping();

	    // Create a Camera and Attach to the Agent
	    main_camera_ = scene_->world_->add_camera(vec3(0, 0, 0),
	    		vec3(0.3,1.3,0.0), (float) 4 / 3);
	    scene_->world_->attach_camera(main_camera_, agent_);
	    renderer_->Init(main_camera_);
	    scene_->world_->BulletStep();

	    return "NavTarget";
	}

	std::string Task_FollowRobot::NavTarget() {
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
        renderer_->StepRender(scene_->world_);
        ctx_->SwapBuffer();
        ctx_->PollEvent();

        return "NavTarget";
	}


	//=======================================Task 1=============================

	Task_NavToLargeCrate::Task_NavToLargeCrate(render_engine::Render * renderer,
			Map * map) : iterations_(0),
					     scene_(map),
					     agent_(nullptr),
					     lidar_(new Lidar(map->world_, 180, 4.0f)),
					     renderer_(renderer),
					     ctx_(renderer->ctx_),
					     main_camera_(nullptr),
					     cam_pitch_(0) 
	{
		// Init Visualization for Lidar
		renderer->InitDrawBatchRay(180);
	}

	Task_NavToLargeCrate::~Task_NavToLargeCrate() {}

	TaskStages Task_NavToLargeCrate::GetStages() {
		TaskStages stages;
		stages["idle"] = std::bind(&Task_NavToLargeCrate::Start, this);
		stages["NavTarget"] = std::bind(&Task_NavToLargeCrate::NavTarget, this);
		return stages;
	}

	std::string Task_NavToLargeCrate::Start() {
		iterations_ = 0;
		scene_->ResetMap();
		scene_->ClearRules();
		scene_->world_->PrintCacheInfo();

		scene_->CreateLabel(crate1, "crate1");
	    scene_->CreateSectionType(floor0, wall, door0);
	    scene_->CreateSpawnOnFloor(crate1);
	    scene_->CreateSpawnOnFloor(crate03);
	    scene_->CreateSpawnOnObject(crate03);

	    // Generata Scene
	    glm::vec3 startPosition = scene_->GenerateFloorPlan(10, 10);
	    scene_->Spawn(3, 0, 0);

	   	// Load Agent
		agent_ = scene_->world_->LoadRobot(
	        "husky/husky.urdf",
	        btVector3(startPosition.x,0.01,startPosition.z),
	        btQuaternion(btVector3(-1,0,0),1.57),
	        btVector3(1,1,1),
	        "Husky",
	        true
	    );
	    agent_->move(true);
	    agent_->DisableSleeping();

	    // Create a Camera and Attach to the Agent
	    main_camera_ = scene_->world_->add_camera(vec3(0, 0, 0),
	    		vec3(0.3,1.3,0.0), (float) 4 / 3);
	    scene_->world_->attach_camera(main_camera_, agent_);
	    renderer_->Init(main_camera_);
	    scene_->world_->BulletStep();

	    return "NavTarget";
	}

	std::string Task_NavToLargeCrate::NavTarget() {
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

        cam_pitch_ = glm::clamp(cam_pitch_, -45.0f, 45.0f);
        scene_->world_->rotate_camera(main_camera_, cam_pitch_);

        // Check In-Range
        std::vector<ObjectAttributes> temp;
        scene_->world_->QueryObjectByLabel("crate1", temp);

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
        		glm::vec3 toPosition = main_camera_->Front * 2.0f + fromPosition;
        		int res = scene_->world_->RayTest(fromPosition, toPosition);

        		if(res == temp[i].bullet_id)
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
        if(iterations_++ > 12000) 
        	return "idle";

        // THIS IS ONLY FOR TESTING! 
        // Force to Switch a New Scene
        if(ctx_->GetKeyPressSpace()) {
        	ctx_->PollEvent();
        	printf("count: %d\n", scene_->world_->reset_count_);
        	return "idle";
        }

        // Step Simulation and Renderer
        scene_->world_->BulletStep();   
        renderer_->StepRender(scene_->world_);
        ctx_->SwapBuffer();
        ctx_->PollEvent();

        return "NavTarget";
	}

	//=======================================Task 2=============================

	Task_NavToSmallCrate::Task_NavToSmallCrate(render_engine::Render * renderer,
			Map * map) : iterations_(0),
					     scene_(map),
					     agent_(nullptr),
					     lidar_(new Lidar(map->world_, 180, 4.0f)),
					     renderer_(renderer),
					     ctx_(renderer->ctx_),
					     main_camera_(nullptr),
					     cam_pitch_(0) 
	{
		// Init Visualization for Lidar
		renderer->InitDrawBatchRay(180);
	}

	Task_NavToSmallCrate::~Task_NavToSmallCrate() {}


	TaskStages Task_NavToSmallCrate::GetStages() {
		TaskStages stages;
		stages["idle"] = std::bind(&Task_NavToSmallCrate::Start, this);
		stages["NavTarget"] = std::bind(&Task_NavToSmallCrate::NavTarget, this);
		return stages;
	}

	std::string Task_NavToSmallCrate::Start() {
		iterations_ = 0;
		scene_->ResetMap();
		scene_->ClearRules();
		scene_->CreateLabel(crate03, "crate03");
	    scene_->CreateSectionType(floor2, wall, door0);
	    scene_->CreateSpawnOnFloor(crate03);
	    scene_->CreateSpawnOnObject(crate03);

	    // Generata Scene
	    glm::vec3 startPosition = scene_->GenerateFloorPlan(10, 10);
	    scene_->Spawn(8, 2, 0);

	   	// Load Agent
	   	agent_ = scene_->world_->LoadRobot(
	        "husky/husky.urdf",
	        btVector3(startPosition.x,0.01,startPosition.z),
	        btQuaternion(btVector3(-1,0,0),1.57),
	        btVector3(1,1,1),
	        "Husky",
	        true
	    );
	    agent_->move(true);
	    agent_->DisableSleeping();

	    // Create a Camera and Attach to the Agent
	    main_camera_ = scene_->world_->add_camera(vec3(0, 0, 0),
	    		vec3(0.3,1.3,0.0), (float) 4 / 3);
	    scene_->world_->attach_camera(main_camera_, agent_);
	    renderer_->Init(main_camera_);
	    scene_->world_->BulletStep();

	    return "NavTarget";
	}

	std::string Task_NavToSmallCrate::NavTarget() {
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

        cam_pitch_ = glm::clamp(cam_pitch_, -45.0f, 45.0f);
        scene_->world_->rotate_camera(main_camera_, cam_pitch_);

        // Check In-Range
        std::vector<ObjectAttributes> temp;
        scene_->world_->QueryObjectByLabel("crate03", temp);

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
        		scene_->world_->QueryObjectDirectionByLabel("crate03", 
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

        // Reset After N Steps
        if(iterations_++ > 12000) 
        	return "idle";

        // THIS IS ONLY FOR TESTING! 
        // Force to Switch a New Scene
        if(ctx_->GetKeyPressSpace()) {
        	ctx_->PollEvent();
        	printf("count: %d\n", scene_->world_->reset_count_);
        	return "idle";
        }

        // Step Simulation and Renderer
        scene_->world_->BulletStep();   
        renderer_->StepRender(scene_->world_);
        ctx_->SwapBuffer();
        ctx_->PollEvent();

        return "NavTarget";
	}


	//=======================================Task 3=============================


	Task_NavToFruitBowl::Task_NavToFruitBowl(render_engine::Render * renderer,
			MapSuncg * map) : iterations_(0),
						      scene_(map),
						      agent_(nullptr),
						      renderer_(renderer),
						      ctx_(renderer->ctx_),
						      main_camera_(nullptr),
						      cam_pitch_(0) {}

	Task_NavToFruitBowl::~Task_NavToFruitBowl() {};

	TaskStages Task_NavToFruitBowl::GetStages() {
		TaskStages stages;
		stages["idle"] = std::bind(&Task_NavToFruitBowl::Start, this);
		stages["NavTarget"] = std::bind(&Task_NavToFruitBowl::NavTarget, this);
		return stages;
	}

	std::string Task_NavToFruitBowl::Start() {
		// Setup Lighting
		renderer_->sunlight_.direction = glm::vec3(0.3, 1, 1);
        renderer_->lighting_.exposure = 1.0f;
        renderer_->lighting_.indirect_strength = 1.5f;
        renderer_->lighting_.traceshadow_distance = 0.3f;
        renderer_->lighting_.propagation_distance = 0.3f;
        renderer_->lighting_.force_disable_shadow = true;

        // Load SUNCG Scene
        iterations_ = 0;
        scene_->ResetMap();
       	scene_->SetRemoveAll( kRemoveStairs );
        scene_->LoadCategoryCSV(metadata_models.c_str());
        scene_->AddPhysicalProperties("chair", {100, false});
	    scene_->AddPhysicalProperties("fruit_bowl", {100, false});
	    scene_->AddPhysicalProperties("trash_can", {100, false});
	    scene_->AddPhysicalProperties("coffee_machine", {100, false});
	    scene_->AddPhysicalProperties("knife_rack", {100, false});
	    scene_->AddPhysicalProperties("knife", {10, false});
	    scene_->AddPhysicalProperties("teapot", {100, false});
	    scene_->LoadJSON(house0.c_str(), data_dir.c_str(), true);
	    scene_->SetMapSize(-8, -8, 6, 6);

	   	// Load Agent
	   	agent_ = scene_->world_->LoadRobot(
	        "husky/robot_kuka.urdf",
	        btVector3(-6,0.21,-1),
	        btQuaternion(btVector3(-1,0,0),1.57),
	        btVector3(0.6f, 0.6f, 0.6f),
	        "agent",
	        true
	    );
	    agent_->move(true);
	    agent_->DisableSleeping();

	    // Reset Joint Pose
	    pos_0_ = 0.0f;
	    pos_1_ = 0.0f;
	    pos_2_ = 0.0f;
	    pos_3_ = 0.0f;
	    pos_4_ = 0.0f;
	    pos_5_ = 0.0f;
	    pos_6_ = 0.0f;
	    pos_7_ = 0.0f;

	   	// Create a Camera and Attach to the Agent
	    main_camera_ = scene_->world_->add_camera(vec3(0, 0, 0),
	    		vec3(0.3,1.3,0.0), (float) 4 / 3);
	    scene_->world_->attach_camera(main_camera_, agent_);
	    renderer_->Init(main_camera_);
	    scene_->world_->BulletStep();

	    return "NavTarget";
	}

	std::string Task_NavToFruitBowl::NavTarget() {
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

        if(ctx_->GetKeyPressKP1())
            pos_5_ += 0.0025f;

        if(ctx_->GetKeyPressKP2())
            pos_5_ -= 0.0025f;

        if(ctx_->GetKeyPressKP4())
            pos_6_ += 0.0025f;

        if(ctx_->GetKeyPressKP5())
            pos_6_ -= 0.0025f;

        if(ctx_->GetKeyPressKP7())
            pos_7_ = 0.05f;

        if(ctx_->GetKeyPressKP8())
            pos_7_ = 0.005f;

        cam_pitch_ = glm::clamp(cam_pitch_, -45.0f, 45.0f);
        pos_0_ = glm::clamp(pos_0_, -1.0f, 1.0f);
        pos_1_ = glm::clamp(pos_1_, -1.0f, 1.0f);
        pos_2_ = glm::clamp(pos_2_, -1.0f, 1.0f);
        pos_3_ = glm::clamp(pos_3_, -1.0f, 1.0f);
        pos_4_ = glm::clamp(pos_4_, -1.0f, 1.0f);
        pos_5_ = glm::clamp(pos_5_, -1.0f, 1.0f);
        pos_6_ = glm::clamp(pos_6_, -1.0f, 1.0f);

        // Control Desired Joints
        Joint * j;
        j = agent_->robot_data_.joints_list_[2];
        j->SetJointMotorControlPosition(pos_0_, 0.1f, 1.0f,1000.0f);
        j = agent_->robot_data_.joints_list_[3];
        j->SetJointMotorControlPosition(pos_1_, 0.1f, 1.0f,1000.0f);
        j = agent_->robot_data_.joints_list_[4];
        j->SetJointMotorControlPosition(pos_2_, 0.1f, 1.0f,1000.0f);
        j = agent_->robot_data_.joints_list_[5];
        j->SetJointMotorControlPosition(pos_3_, 0.1f, 1.0f,1000.0f);
        j = agent_->robot_data_.joints_list_[6];
        j->SetJointMotorControlPosition(pos_4_, 0.1f, 1.0f,1000.0f);
        j = agent_->robot_data_.joints_list_[7];
        j->SetJointMotorControlPosition(pos_5_, 0.1f, 1.0f,1000.0f);
        j = agent_->robot_data_.joints_list_[8];
        j->SetJointMotorControlPosition(pos_6_, 0.1f, 1.0f,1000.0f);
        j = agent_->robot_data_.joints_list_[10];
        j->SetJointMotorControlPosition(pos_7_, 0.1f, 1.0f,50.0f);
        j = agent_->robot_data_.joints_list_[12];
        j->SetJointMotorControlPosition(pos_7_, 0.1f, 1.0f,50.0f);
        scene_->world_->rotate_camera(main_camera_, cam_pitch_);

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

        if(ctx_->GetKeyPressSpace()) {
        	ctx_->PollEvent();
        	printf("count: %d\n", scene_->world_->reset_count_);
        	return "idle";
        }

        // Reset After N Steps
        if(iterations_++ > 12000) 
        	return "idle";

        // Step Simulation and Renderer
        scene_->world_->BulletStep();   
        renderer_->StepRender(scene_->world_);
        ctx_->SwapBuffer();
        ctx_->PollEvent();

        return "NavTarget";
	}

	//=======================================Task 4=============================


	Task_TouchPan::Task_TouchPan(render_engine::Render * renderer, 
			MapSuncg * map) : iterations_(0),
						      scene_(map),
						      agent_(nullptr),
						      renderer_(renderer),
						      ctx_(renderer->ctx_),
						      main_camera_(nullptr),
						      cam_pitch_(0) {}

	Task_TouchPan::~Task_TouchPan() {}

	TaskStages Task_TouchPan::GetStages() {
		TaskStages stages;
		stages["idle"] = std::bind(&Task_TouchPan::Start, this);
		stages["NavTarget"] = std::bind(&Task_TouchPan::NavTarget, this);
		return stages;
	}

	std::string Task_TouchPan::Start() {
		// Setup Lighting
		renderer_->sunlight_.direction = glm::vec3(0.3, 1, 1);
        renderer_->lighting_.exposure = 1.0f;
        renderer_->lighting_.indirect_strength = 1.5f;
        renderer_->lighting_.traceshadow_distance = 0.3f;
        renderer_->lighting_.propagation_distance = 0.3f;
        renderer_->lighting_.force_disable_shadow = true;

        // Load SUNCG Scene
        iterations_ = 0;
        scene_->ResetMap();
       	scene_->SetRemoveAll( kRemoveStairs );
        scene_->LoadCategoryCSV(metadata_models.c_str());
        scene_->AddPhysicalProperties("chair", {100, false});
	    scene_->AddPhysicalProperties("fruit_bowl", {100, false});
	    scene_->AddPhysicalProperties("trash_can", {100, false});
	    scene_->AddPhysicalProperties("coffee_machine", {100, false});
	    scene_->AddPhysicalProperties("knife_rack", {100, false});
	    scene_->AddPhysicalProperties("knife", {10, false});
	    scene_->AddPhysicalProperties("teapot", {100, false});
	    scene_->AddPhysicalProperties("bottle", {10, false});
	    scene_->LoadJSON(house0.c_str(), data_dir.c_str(), true);
	    scene_->SetMapSize(-8, -8, 6, 6);

	   	// Load Agent
	   	agent_ = scene_->world_->LoadRobot(
	        "husky/robot_kuka.urdf",
	        btVector3(-6,0.21,-1),
	        btQuaternion(btVector3(-1,0,0),1.57),
	        btVector3(0.6f, 0.6f, 0.6f),
	        "agent",
	        true
	    );
	    agent_->move(true);
	    agent_->DisableSleeping();

	    // Reset Joint Pose
	    pos_0_ = 0.0f;
	    pos_1_ = 0.0f;
	    pos_2_ = 0.0f;
	    pos_3_ = 0.0f;
	    pos_4_ = 0.0f;
	    pos_5_ = 0.0f;
	    pos_6_ = 0.0f;
	    pos_7_ = 0.0f;

	   	// Create a Camera and Attach to the Agent
	    main_camera_ = scene_->world_->add_camera(vec3(0, 0, 0),
	    		vec3(0.3,1.3,0.0), (float) 4 / 3);
	    scene_->world_->attach_camera(main_camera_, agent_);
	    renderer_->Init(main_camera_);
	    scene_->world_->BulletStep();

	    return "NavTarget";
	}

	std::string Task_TouchPan::NavTarget() {
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

        if(ctx_->GetKeyPressKP1())
            pos_5_ += 0.0025f;

        if(ctx_->GetKeyPressKP2())
            pos_5_ -= 0.0025f;

        if(ctx_->GetKeyPressKP4())
            pos_6_ += 0.0025f;

        if(ctx_->GetKeyPressKP5())
            pos_6_ -= 0.0025f;

        if(ctx_->GetKeyPressKP7())
            pos_7_ = 0.05f;

        if(ctx_->GetKeyPressKP8())
            pos_7_ = 0.005f;

        cam_pitch_ = glm::clamp(cam_pitch_, -45.0f, 45.0f);
        pos_0_ = glm::clamp(pos_0_, -1.0f, 1.0f);
        pos_1_ = glm::clamp(pos_1_, -1.0f, 1.0f);
        pos_2_ = glm::clamp(pos_2_, -1.0f, 1.0f);
        pos_3_ = glm::clamp(pos_3_, -1.0f, 1.0f);
        pos_4_ = glm::clamp(pos_4_, -1.0f, 1.0f);
        pos_5_ = glm::clamp(pos_5_, -1.0f, 1.0f);
        pos_6_ = glm::clamp(pos_6_, -1.0f, 1.0f);

        // Control Desired Joints
        Joint * j;
        j = agent_->robot_data_.joints_list_[2];
        j->SetJointMotorControlPosition(pos_0_, 0.1f, 1.0f,1000.0f);
        j = agent_->robot_data_.joints_list_[3];
        j->SetJointMotorControlPosition(pos_1_, 0.1f, 1.0f,1000.0f);
        j = agent_->robot_data_.joints_list_[4];
        j->SetJointMotorControlPosition(pos_2_, 0.1f, 1.0f,1000.0f);
        j = agent_->robot_data_.joints_list_[5];
        j->SetJointMotorControlPosition(pos_3_, 0.1f, 1.0f,1000.0f);
        j = agent_->robot_data_.joints_list_[6];
        j->SetJointMotorControlPosition(pos_4_, 0.1f, 1.0f,1000.0f);
        j = agent_->robot_data_.joints_list_[7];
        j->SetJointMotorControlPosition(pos_5_, 0.1f, 1.0f,1000.0f);
        j = agent_->robot_data_.joints_list_[8];
        j->SetJointMotorControlPosition(pos_6_, 0.1f, 1.0f,1000.0f);
        j = agent_->robot_data_.joints_list_[10];
        j->SetJointMotorControlPosition(pos_7_, 0.1f, 1.0f,50.0f);
        j = agent_->robot_data_.joints_list_[12];
        j->SetJointMotorControlPosition(pos_7_, 0.1f, 1.0f,50.0f);
        scene_->world_->rotate_camera(main_camera_, cam_pitch_);

        // Check In-Range
        std::vector<ObjectAttributes> temp;
        scene_->world_->QueryObjectByLabel("pan", temp);

        for(int i = 0; i < temp.size(); ++i)
        {
        	glm::vec3 aabb_min = temp[i].aabb_min - glm::vec3(0.02);
        	glm::vec3 aabb_max = temp[i].aabb_max + glm::vec3(0.02);

        	// Check Gripper Left Finger
        	glm::vec3 gripper_aabb_min, gripper_aabb_max;
        	agent_->robot_data_.other_parts_[10]->GetAABB(gripper_aabb_min, gripper_aabb_max);
        	if(aabb_min.x < gripper_aabb_max.x && aabb_max.x > gripper_aabb_min.x &&
        	   aabb_min.y < gripper_aabb_max.y && aabb_max.y > gripper_aabb_min.y &&
        	   aabb_min.z < gripper_aabb_max.z && aabb_max.z > gripper_aabb_min.z) {
        		return "idle";
        	}

        	// Check Gripper Right Finger
        	agent_->robot_data_.other_parts_[11]->GetAABB(gripper_aabb_min, gripper_aabb_max);
        	if(aabb_min.x < gripper_aabb_max.x && aabb_max.x > gripper_aabb_min.x &&
        	   aabb_min.y < gripper_aabb_max.y && aabb_max.y > gripper_aabb_min.y &&
        	   aabb_min.z < gripper_aabb_max.z && aabb_max.z > gripper_aabb_min.z) {
        		return "idle";
        	}
        }

        if(ctx_->GetKeyPressSpace()) {
        	ctx_->PollEvent();
        	printf("count: %d\n", scene_->world_->reset_count_);
        	return "idle";
        }

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


	//=======================================Task 5=============================
	// This is a dummy task for only demo the usage of new features, such as
	// inventory, pickup/putdown and animation

	Task_NewFeatures::Task_NewFeatures(render_engine::Render * renderer,
		Map * map) : iterations_(0),
				     scene_(map),
				     agent_(nullptr),
				     target_(nullptr),
				     renderer_(renderer),
				     ctx_(renderer->ctx_),
				     main_camera_(nullptr),
				     cam_pitch_(0),
				     inventory_(new Inventory(10)) {}

	Task_NewFeatures::~Task_NewFeatures() {}	     

	TaskStages Task_NewFeatures::GetStages() {
		TaskStages stages;
		stages["idle"] = std::bind(&Task_NewFeatures::Start, this);
		stages["NavTarget"] = std::bind(&Task_NewFeatures::NavTarget, this);
		return stages;
	}


	std::string Task_NewFeatures::Start() {
		renderer_->sunlight_.ambient = glm::vec3(0.02,0.02,0.02);
        renderer_->lighting_.exposure = 0.7f;
        renderer_->lighting_.indirect_strength = 0.25f;
        renderer_->lighting_.traceshadow_distance = 0.3f;
        renderer_->lighting_.propagation_distance = 0.3f;
        renderer_->lighting_.sample_factor = 0.7f;
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


	    door_anim_ = scene_->world_->LoadRobot(
	        anim_action,
	        btVector3(1, 0.1, 6),
	        btQuaternion(btVector3(0,0,1),1.57),
	        btVector3(1, 1, 1),
	        "Door_Anim",
	        true
	    );

	    obj_conv_ = scene_->world_->LoadRobot(
	    	conv_action,
	    	btVector3(6, 0, 1),
	        btQuaternion(btVector3(1,0,0),0),
	        btVector3(1, 1, 1),
	        "OBJ_Conv",
	        true
	    );

		// Load Obj
		door_ = scene_->world_->LoadRobot(
	        door_ani,
	        btVector3(4, 0.1, 6),
	        btQuaternion(btVector3(0,0,1),1.57),
	        btVector3(1, 1, 1),
	        "door",
	        false
	    );
	   	door_->move(true);
	   	door_->DisableSleeping();

	  	
		RobotBase * obj = scene_->world_->LoadRobot(
	        crate03,
	        btVector3(2, 0, 2),
	        btQuaternion(btVector3(1,0,0),0),
	        btVector3(1, 1, 1),
	        "crate",
	        false
	    );
	   	obj->move(false);
	   	
		// Load Target Robot
		target_ = scene_->world_->LoadRobot(
	        "/home/ziyuli/model/pica_robot.urdf",
	        btVector3(2, 0.01, 0),
	        btQuaternion(btVector3(1,0,0),0),
	        btVector3(0.1f, 0.1f, 0.1f),
	        "pica_robot",
	        true
	    );
	   	target_->move(false);

	   	// Load Agent
	   	agent_ = scene_->world_->LoadRobot(
	        "husky/husky.urdf",
	        btVector3(0,0.001,0),
	        btQuaternion(btVector3(-1,0,0),1.57),
	        btVector3(1, 1, 1),
	        "husky",
	        true
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


		std::string Task_NewFeatures::NavTarget() {
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


        if(ctx_->GetKeyPress7() && door_anim_) {
            door_anim_->TakeAction(0);
        }

        if(ctx_->GetKeyPress8() && door_anim_)
            door_anim_->TakeAction(1);

        if(ctx_->GetKeyPress5() && obj_conv_) {
            obj_conv_->TakeAction(0);
        }

        if(ctx_->GetKeyPress6() && obj_conv_)
            obj_conv_->TakeAction(1);


        if(ctx_->GetKeyPress0() && door_) {
        	door_angle_ = glm::clamp(door_angle_ + 0.01f, 0.0f, 1.5f);
        	door_->SetJointPosition(1, door_angle_, 1.0f, 1.0f, 20000.0f);
        }
        if(ctx_->GetKeyPress9() && door_) {
        	door_angle_ = glm::clamp(door_angle_ - 0.01f, 0.0f, 1.5f);
        	door_->SetJointPosition(1, door_angle_, 1.0f, 1.0f, 20000.0f);
        }

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

        iterations_++;


        // This is just for testing
        if(ctx_->GetKeyPressSpace()) {
            ctx_->PollEvent();
            printf("Reset the Scene : %d\n", scene_->world_->reset_count_);
            return "idle";
        }

        // Step Simulation and Renderer
        scene_->world_->BulletStep();   
        renderer_->StepRender(scene_->world_);
        ctx_->SwapBuffer();
        ctx_->PollEvent();

        return "NavTarget";
	}
}