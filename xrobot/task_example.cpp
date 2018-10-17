#include "task_example.h"

static std::string door       = "./door/door.urdf";
static std::string wall       = "./wall0/floor.urdf";
static std::string floor_0    = "./floor0/floor.urdf";
static std::string floor_1    = "./floor1/floor.urdf";
static std::string crate1     = "./crate_1/crate.urdf";
static std::string crate03    = "./crate_0.3/crate.urdf";
static std::string apple      = "./apple/apple.urdf";
static std::string r2d2       = "./r2d2/r2d2.urdf";
static std::string husky      = "./husky/husky.urdf";
static std::string husky_kuka = "./husky/robot_kuka.urdf";

static std::string test_wall   = "./wall/floor.urdf";
static std::string test_floor  = "./floor/floor.urdf";

static std::string suncg_dir   = "/home/ziyuli/XWorld/xrobot/data/suncg";
static std::string suncg_meta  = suncg_dir + "/ModelCategoryMapping.csv";
static std::string suncg_house = suncg_dir + "/house/7c16efebdfe46f3f14fa81abe500589c/house.json";

static std::string object_with_action_0 = "/home/ziyuli/XWorld/xrobot/data/open_box.json";
static std::string object_with_action_1 = "/home/ziyuli/XWorld/xrobot/data/door.json";

namespace xrobot
{
	
	//=======================================Task 0=============================

	Task_FollowRobot::Task_FollowRobot(
		std::shared_ptr<render_engine::Render> renderer,
		std::shared_ptr<Map> map) : iterations_(0),
								    scene_(map),
								    agent_(),
								    target_(),
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
		scene_->CreateLabel(r2d2, "r2d2");
		scene_->CreateSectionType(test_floor, test_wall, door);
		scene_->GenerateTestFloorPlan(5, 5);

		// Load Target Robot
		target_ = scene_->world_->LoadRobot(
	        r2d2,
	        btVector3(2, 0.01, 0),
	        btQuaternion(btVector3(1,0,0),0),
	        btVector3(0.01f,0.01f,0.01f),
	        "r2d2",
	        true
	    );

	    if(auto target_sptr = target_.lock()) {
	    	target_sptr->move(true);
	   		target_sptr->DisableSleeping();
	    }

	   	// Load Agent
	   	agent_ = scene_->world_->LoadRobot(
	        husky,
	        btVector3(0,0.01,0),
	        btQuaternion(btVector3(-1,0,0),1.57),
	        btVector3(1,1,1),
	        "husky",
	        true
	    );

	    if(auto agent_sptr = agent_.lock()) {
		    agent_sptr->move(true);
		    agent_sptr->DisableSleeping();

		    // Create a Camera and Attach to the Agent
		    main_camera_ = scene_->world_->add_camera(vec3(0, 0, 0),
		    		vec3(0.3,1.3,0.0), (float) 4 / 3);
		    scene_->world_->attach_camera(main_camera_, agent_sptr.get());
		}

	    renderer_->Init(main_camera_);
	    scene_->world_->BulletStep();

	    return "NavTarget";
	}

	std::string Task_FollowRobot::NavTarget() {
		// TODO
		// Remove Freeze Parameter
		if(auto agent_sptr = agent_.lock()) {
			agent_sptr->Freeze(!iterations_);

			if(ctx_->GetKeyPressUp())
	            agent_sptr->MoveForward(1);

	        if(ctx_->GetKeyPressDown())
	            agent_sptr->MoveBackward(1);

	        if(ctx_->GetKeyPressLeft())
	            agent_sptr->TurnLeft(1);

	        if(ctx_->GetKeyPressRight())
	            agent_sptr->TurnRight(1);
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


	
	//=======================================Task 1=============================


	Task_NavToLargeCrate::Task_NavToLargeCrate(
		std::shared_ptr<render_engine::Render> renderer,
		std::shared_ptr<Map> map) : iterations_(0),
								    scene_(map),
								    agent_(),
								    lidar_(nullptr),
								    renderer_(renderer),
								    ctx_(renderer->ctx_),
								    main_camera_(nullptr),
								    cam_pitch_(0) 
	{

		lidar_ = std::make_shared<Lidar>(map->world_.get(), 180, 4.0f);

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
		
		renderer_->sunlight_.ambient = glm::vec3(0.15,0.15,0.15);
        renderer_->lighting_.exposure = 1.5f;
        renderer_->lighting_.indirect_strength = 0.4f;
        renderer_->lighting_.traceshadow_distance = 0.5f;
        renderer_->lighting_.propagation_distance = 0.5f;
        renderer_->lighting_.sample_factor = 0.4f;
        renderer_->lighting_.boost_ambient = 0.02f;
        renderer_->lighting_.shadow_bias_scale = 0.0003f;
        renderer_->lighting_.linear_voxelize = false;

		iterations_ = 0;
		scene_->ResetMap();
		scene_->ClearRules();

		scene_->CreateLabel(crate1, "crate1");
	    scene_->CreateSectionType(floor_0, wall, door);
	    scene_->CreateSectionType(floor_1, wall, door);
	    scene_->CreateSpawnOnFloor(crate1);
	    scene_->CreateSpawnOnFloor(crate03);
	    scene_->CreateSpawnOnObject(crate03);

	    // Generata Scene
	    glm::vec3 startPosition = scene_->GenerateFloorPlan(10, 10);
	    scene_->Spawn(3, 0, 0);

	   	// Load Agent
		agent_ = scene_->world_->LoadRobot(
	        husky,
	        btVector3(startPosition.x,0.01,startPosition.z),
	        btQuaternion(btVector3(-1,0,0),1.57),
	        btVector3(1,1,1),
	        "Husky",
	        true
	    );

		    if(auto agent_sptr = agent_.lock()) {
			    agent_sptr->move(true);
			    agent_sptr->DisableSleeping();

		    // Create a Camera and Attach to the Agent
		    main_camera_ = scene_->world_->add_camera(vec3(0, 0, 0),
		    		vec3(0.3,1.3,0.0), (float) 4 / 3);
		    scene_->world_->attach_camera(main_camera_, agent_sptr.get());
		}

	    renderer_->Init(main_camera_);
	    scene_->world_->BulletStep();

	    return "NavTarget";
	}

	std::string Task_NavToLargeCrate::NavTarget() {
		// TODO
		// Remove Freeze Parameter
		if(auto agent_sptr = agent_.lock()) {
			agent_sptr->Freeze(!iterations_);

			if(ctx_->GetKeyPressUp())
	            agent_sptr->MoveForward(1);

	        if(ctx_->GetKeyPressDown())
	            agent_sptr->MoveBackward(1);

	        if(ctx_->GetKeyPressLeft())
	            agent_sptr->TurnLeft(1);

	        if(ctx_->GetKeyPressRight())
	            agent_sptr->TurnRight(1);
    	}

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
        	return "idle";
        }

        // Step Simulation and Renderer
        scene_->world_->BulletStep();   
        renderer_->StepRender(scene_->world_.get());
        ctx_->SwapBuffer();
        ctx_->PollEvent();

        return "NavTarget";
	}


	//=======================================Task 2=============================

	Task_NavToSmallCrate::Task_NavToSmallCrate(
		std::shared_ptr<render_engine::Render> renderer,
		std::shared_ptr<Map> map) : iterations_(0),
								    scene_(map),
								    agent_(),
								    lidar_(nullptr),
								    renderer_(renderer),
								    ctx_(renderer->ctx_),
								    main_camera_(nullptr),
								    cam_pitch_(0) 
	{
		lidar_ = std::make_shared<Lidar>(map->world_.get(), 180, 4.0f);

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

		renderer_->sunlight_.ambient = glm::vec3(0.15,0.15,0.15);
        renderer_->lighting_.exposure = 1.5f;
        renderer_->lighting_.indirect_strength = 0.4f;
        renderer_->lighting_.traceshadow_distance = 0.5f;
        renderer_->lighting_.propagation_distance = 0.5f;
        renderer_->lighting_.sample_factor = 0.4f;
        renderer_->lighting_.boost_ambient = 0.02f;
        renderer_->lighting_.shadow_bias_scale = 0.0003f;
        renderer_->lighting_.linear_voxelize = false;

		iterations_ = 0;
		scene_->ResetMap();
		scene_->ClearRules();
		scene_->CreateLabel(crate03, "crate03");
	    scene_->CreateSectionType(floor_0, wall, door);
	    scene_->CreateSpawnOnFloor(crate03);
	    scene_->CreateSpawnOnObject(crate03);

	    // Generata Scene
	    glm::vec3 startPosition = scene_->GenerateFloorPlan(10, 10);
	    scene_->Spawn(8, 2, 0);

	   	// Load Agent
	   	agent_ = scene_->world_->LoadRobot(
	        husky,
	        btVector3(startPosition.x,0.01,startPosition.z),
	        btQuaternion(btVector3(-1,0,0),1.57),
	        btVector3(1,1,1),
	        "Husky",
	        true
	    );

	    if(auto agent_sptr = agent_.lock()) {
		    agent_sptr->move(true);
		    agent_sptr->DisableSleeping();

		    // Create a Camera and Attach to the Agent
		    main_camera_ = scene_->world_->add_camera(vec3(0, 0, 0),
		    		vec3(0.3,1.3,0.0), (float) 4 / 3);
		    scene_->world_->attach_camera(main_camera_, agent_sptr.get());
		}

	    renderer_->Init(main_camera_);
	    scene_->world_->BulletStep();

	    return "NavTarget";
	}

	std::string Task_NavToSmallCrate::NavTarget() {
		// TODO
		// Remove Freeze Parameter
		if(auto agent_sptr = agent_.lock()) {
			agent_sptr->Freeze(!iterations_);

			if(ctx_->GetKeyPressUp())
	            agent_sptr->MoveForward(1);

	        if(ctx_->GetKeyPressDown())
	            agent_sptr->MoveBackward(1);

	        if(ctx_->GetKeyPressLeft())
	            agent_sptr->TurnLeft(1);

	        if(ctx_->GetKeyPressRight())
	            agent_sptr->TurnRight(1);
	    }

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
        	return "idle";
        }

        // Step Simulation and Renderer
        scene_->world_->BulletStep();   
        renderer_->StepRender(scene_->world_.get());
        ctx_->SwapBuffer();
        ctx_->PollEvent();

        return "NavTarget";
	}


	//=======================================Task 3=============================


	Task_NavToFruitBowl::Task_NavToFruitBowl(
		std::shared_ptr<render_engine::Render> renderer,
		std::shared_ptr<MapSuncg> map)   : iterations_(0),
									       scene_(map),
									       agent_(),
									       lidar_(nullptr),
									       inventory_(nullptr),
									       renderer_(renderer),
									       ctx_(renderer->ctx_),
									       main_camera_(nullptr),
									       cam_pitch_(0) 
	{
		inventory_= std::make_shared<Inventory>(1);
		lidar_ = std::make_shared<Lidar>(map->world_.get(), 180, 4.0f);

		// Init Visualization for Lidar
		renderer->InitDrawBatchRay(180);
	}

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
        scene_->LoadCategoryCSV(suncg_meta.c_str());
        scene_->AddPhysicalProperties("chair", {100, false});
	    scene_->AddPhysicalProperties("fruit_bowl", {100, false});
	    scene_->AddPhysicalProperties("trash_can", {100, false});
	    scene_->AddPhysicalProperties("coffee_machine", {100, false});
	    scene_->AddPhysicalProperties("knife_rack", {100, false});
	    scene_->AddPhysicalProperties("knife", {10, false});
	    scene_->AddPhysicalProperties("teapot", {100, false});
	    scene_->LoadJSON(suncg_house.c_str(), suncg_dir.c_str(), true);
	    scene_->SetMapSize(-8, -8, 6, 6);

	    inventory_->ResetNonPickableObjectTag();
	    inventory_->AddNonPickableObjectTag("Wall");
	    inventory_->AddNonPickableObjectTag("Floor");
	    inventory_->AddNonPickableObjectTag("Ceiling");

	    // Reset Joint Pose
	    pos_0_ = 0.0f;
	    pos_1_ = 0.0f;
	    pos_2_ = 0.0f;
	    pos_3_ = 0.0f;
	    pos_4_ = 0.0f;
	    pos_5_ = 0.0f;
	    pos_6_ = 0.0f;
	    pos_7_ = 0.0f;

	   	// Load Agent
	   	agent_ = scene_->world_->LoadRobot(
	        husky_kuka,
	        btVector3(-6,0.21,-1),
	        btQuaternion(btVector3(-1,0,0),1.57),
	        btVector3(0.6f, 0.6f, 0.6f),
	        "agent",
	        true
	    );

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

	    return "NavTarget";
	}

	std::string Task_NavToFruitBowl::NavTarget() {
		// TODO
		// Remove Freeze Parameter

		if(auto agent_sptr = agent_.lock()) 
	   	{
			agent_sptr->Freeze(!iterations_);

			if(ctx_->GetKeyPressUp())
	            agent_sptr->MoveForward(1);

	        if(ctx_->GetKeyPressDown())
	            agent_sptr->MoveBackward(1);

	        if(ctx_->GetKeyPressLeft())
	            agent_sptr->TurnLeft(1);

	        if(ctx_->GetKeyPressRight())
	            agent_sptr->TurnRight(1);

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

	        if(ctx_->GetKeyPressKP4()) {
	            obj_conv_sptr->TakeAction(0);
	        }

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

        // if(ctx_->GetKeyPressKP1())
        //     pos_5_ += 0.0025f;

        // if(ctx_->GetKeyPressKP2())
        //     pos_5_ -= 0.0025f;

        // if(ctx_->GetKeyPressKP4())
        //     pos_6_ += 0.0025f;

        // if(ctx_->GetKeyPressKP5())
            pos_6_ -= 0.0025f;

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
	        j->SetJointMotorControlPosition(pos_0_, 0.1f, 1.0f,1000.0f);
	        j = agent_sptr->robot_data_.joints_list_[3];
	        j->SetJointMotorControlPosition(pos_1_, 0.1f, 1.0f,1000.0f);
	        j = agent_sptr->robot_data_.joints_list_[4];
	        j->SetJointMotorControlPosition(pos_2_, 0.1f, 1.0f,1000.0f);
	        j = agent_sptr->robot_data_.joints_list_[5];
	        j->SetJointMotorControlPosition(pos_3_, 0.1f, 1.0f,1000.0f);
	        j = agent_sptr->robot_data_.joints_list_[6];
	        j->SetJointMotorControlPosition(pos_4_, 0.1f, 1.0f,1000.0f);
	        j = agent_sptr->robot_data_.joints_list_[7];
	        j->SetJointMotorControlPosition(pos_5_, 0.1f, 1.0f,1000.0f);
	        j = agent_sptr->robot_data_.joints_list_[8];
	        j->SetJointMotorControlPosition(pos_6_, 0.1f, 1.0f,1000.0f);
	        j = agent_sptr->robot_data_.joints_list_[10];
	        j->SetJointMotorControlPosition(pos_7_, 0.1f, 1.0f,50.0f);
	        j = agent_sptr->robot_data_.joints_list_[12];
	        j->SetJointMotorControlPosition(pos_7_, 0.1f, 1.0f,50.0f);
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

	//=======================================Task 4=============================


	Task_TouchPan::Task_TouchPan(
		std::shared_ptr<render_engine::Render> renderer,
		std::shared_ptr<MapSuncg> map) : iterations_(0),
									     scene_(map),
									     agent_(),
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
        scene_->LoadCategoryCSV(suncg_meta.c_str());
        scene_->AddPhysicalProperties("chair", {100, false});
	    scene_->AddPhysicalProperties("fruit_bowl", {100, false});
	    scene_->AddPhysicalProperties("trash_can", {100, false});
	    scene_->AddPhysicalProperties("coffee_machine", {100, false});
	    scene_->AddPhysicalProperties("knife_rack", {100, false});
	    scene_->AddPhysicalProperties("knife", {10, false});
	    scene_->AddPhysicalProperties("teapot", {100, false});
	    scene_->AddPhysicalProperties("bottle", {10, false});
	    scene_->LoadJSON(suncg_house.c_str(), suncg_dir.c_str(), true);
	    scene_->SetMapSize(-8, -8, 6, 6);

	    // Reset Joint Pose
	    pos_0_ = 0.0f;
	    pos_1_ = 0.0f;
	    pos_2_ = 0.0f;
	    pos_3_ = 0.0f;
	    pos_4_ = 0.0f;
	    pos_5_ = 0.0f;
	    pos_6_ = 0.0f;
	    pos_7_ = 0.0f;

	    // Load Agent
	   	agent_ = scene_->world_->LoadRobot(
	        husky_kuka,
	        btVector3(-6,0.21,-1),
	        btQuaternion(btVector3(-1,0,0),1.57),
	        btVector3(0.6f, 0.6f, 0.6f),
	        "agent",
	        true
	    );

	   	// Load Agent
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

	    return "NavTarget";
	}

	std::string Task_TouchPan::NavTarget() {
		// TODO
		// Remove Freeze Parameter
		if(auto agent_sptr = agent_.lock()) 
	   	{

			agent_sptr->Freeze(!iterations_);

			if(ctx_->GetKeyPressUp())
	            agent_sptr->MoveForward(1);

	        if(ctx_->GetKeyPressDown())
	            agent_sptr->MoveBackward(1);

	        if(ctx_->GetKeyPressLeft())
	            agent_sptr->TurnLeft(1);

	        if(ctx_->GetKeyPressRight())
	            agent_sptr->TurnRight(1);

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
	        j->SetJointMotorControlPosition(pos_0_, 0.1f, 1.0f,1000.0f);
	        j = agent_sptr->robot_data_.joints_list_[3];
	        j->SetJointMotorControlPosition(pos_1_, 0.1f, 1.0f,1000.0f);
	        j = agent_sptr->robot_data_.joints_list_[4];
	        j->SetJointMotorControlPosition(pos_2_, 0.1f, 1.0f,1000.0f);
	        j = agent_sptr->robot_data_.joints_list_[5];
	        j->SetJointMotorControlPosition(pos_3_, 0.1f, 1.0f,1000.0f);
	        j = agent_sptr->robot_data_.joints_list_[6];
	        j->SetJointMotorControlPosition(pos_4_, 0.1f, 1.0f,1000.0f);
	        j = agent_sptr->robot_data_.joints_list_[7];
	        j->SetJointMotorControlPosition(pos_5_, 0.1f, 1.0f,1000.0f);
	        j = agent_sptr->robot_data_.joints_list_[8];
	        j->SetJointMotorControlPosition(pos_6_, 0.1f, 1.0f,1000.0f);
	        j = agent_sptr->robot_data_.joints_list_[10];
	        j->SetJointMotorControlPosition(pos_7_, 0.1f, 1.0f,50.0f);
	        j = agent_sptr->robot_data_.joints_list_[12];
	        j->SetJointMotorControlPosition(pos_7_, 0.1f, 1.0f,50.0f);
	    }

        // Check In-Range
        std::vector<ObjectAttributes> temp;
        scene_->world_->QueryObjectByLabel("pan", temp);

        for(int i = 0; i < temp.size(); ++i)
        {
        	glm::vec3 aabb_min = temp[i].aabb_min - glm::vec3(0.02);
        	glm::vec3 aabb_max = temp[i].aabb_max + glm::vec3(0.02);

        	if(auto agent_sptr = agent_.lock()) 
	   		{
	        	// Check Gripper Left Finger
	        	glm::vec3 gripper_aabb_min, gripper_aabb_max;
	        	agent_sptr->robot_data_.other_parts_[10]->GetAABB(gripper_aabb_min, gripper_aabb_max);
	        	if(aabb_min.x < gripper_aabb_max.x && aabb_max.x > gripper_aabb_min.x &&
	        	   aabb_min.y < gripper_aabb_max.y && aabb_max.y > gripper_aabb_min.y &&
	        	   aabb_min.z < gripper_aabb_max.z && aabb_max.z > gripper_aabb_min.z) {
	        		return "idle";
	        	}

	        	// Check Gripper Right Finger
	        	agent_sptr->robot_data_.other_parts_[11]->GetAABB(gripper_aabb_min, gripper_aabb_max);
	        	if(aabb_min.x < gripper_aabb_max.x && aabb_max.x > gripper_aabb_min.x &&
	        	   aabb_min.y < gripper_aabb_max.y && aabb_max.y > gripper_aabb_min.y &&
	        	   aabb_min.z < gripper_aabb_max.z && aabb_max.z > gripper_aabb_min.z) {
	        		return "idle";
	        	}

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

	
	//=======================================Task 5=============================
	// This is a dummy task for only demo the usage of new features, such as
	// inventory, pickup/putdown and animation

	Task_NewFeatures::Task_NewFeatures(
		std::shared_ptr<render_engine::Render> renderer,
		std::shared_ptr<Map> map) : iterations_(0),
								    scene_(map),
								    agent_(),
								    renderer_(renderer),
								    ctx_(renderer->ctx_),
								    main_camera_(nullptr),
								    cam_pitch_(0),
								    inventory_() 
	{
		inventory_= std::make_shared<Inventory>(10);
	}

	Task_NewFeatures::~Task_NewFeatures() {}	     

	TaskStages Task_NewFeatures::GetStages() {
		TaskStages stages;
		stages["idle"] = std::bind(&Task_NewFeatures::Start, this);
		stages["NavTarget"] = std::bind(&Task_NewFeatures::NavTarget, this);
		return stages;
	}


	std::string Task_NewFeatures::Start() {
		renderer_->sunlight_.ambient = glm::vec3(0.02,0.02,0.02);
        renderer_->lighting_.exposure = 0.4f;
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
		scene_->CreateLabel(object_with_action_0, "obj");
		scene_->CreateLabel(object_with_action_1, "door");
		scene_->CreateLabel(crate03, "crate");
		scene_->CreateSectionType(test_floor, test_wall, door);
		scene_->GenerateTestFloorPlan(5, 5);

		inventory_->ResetNonPickableObjectTag();
	    inventory_->AddNonPickableObjectTag("Wall");
	    inventory_->AddNonPickableObjectTag("Floor");
	    inventory_->AddNonPickableObjectTag("Ceiling");


	    door_anim_ = scene_->world_->LoadRobot(
	        object_with_action_1,
	        btVector3(1, 0.1, 6),
	        btQuaternion(btVector3(0,0,1),1.57),
	        btVector3(1, 1, 1),
	        "Door_Anim",
	        true
	    );

	    obj_conv_ = scene_->world_->LoadRobot(
	    	object_with_action_0,
	    	btVector3(6, 0, 1),
	        btQuaternion(btVector3(1,0,0),0),
	        btVector3(1, 1, 1),
	        "OBJ_Conv",
	        true
	    );
	  	
		scene_->world_->LoadRobot(
	        crate03,
	        btVector3(2, 0, 2),
	        btQuaternion(btVector3(1,0,0),0),
	        btVector3(1, 1, 1),
	        "crate",
	        false
	    );

	   	// Load Agent
	   	agent_ = scene_->world_->LoadRobot(
	        "husky/husky.urdf",
	        btVector3(0,0.001,0),
	        btQuaternion(btVector3(-1,0,0),1.57),
	        btVector3(1, 1, 1),
	        "husky",
	        true
	    );

	   	if(auto agent_sptr = agent_.lock()) {
		    agent_sptr->move(true);
		    agent_sptr->DisableSleeping();

		    // Create a Camera and Attach to the Agent
		    main_camera_ = scene_->world_->add_camera(vec3(0, 0, 0),
		    		vec3(0.3,1.3,0.0), (float) 4 / 3);
		    scene_->world_->attach_camera(main_camera_, agent_sptr.get());
		}

	    renderer_->Init(main_camera_);
	    scene_->world_->BulletStep();

	    return "NavTarget";
	}


		std::string Task_NewFeatures::NavTarget() {
		// TODO
		// Remove Freeze Parameter

		if(auto agent_sptr = agent_.lock()) {
			agent_sptr->Freeze(!iterations_);
			
			if(ctx_->GetKeyPressUp())
	            agent_sptr->MoveForward(1);

	        if(ctx_->GetKeyPressDown())
	            agent_sptr->MoveBackward(1);

	        if(ctx_->GetKeyPressLeft())
	            agent_sptr->TurnLeft(1);

	        if(ctx_->GetKeyPressRight())
	            agent_sptr->TurnRight(1);
	    }

        if(ctx_->GetKeyPressKP9())
            cam_pitch_ += 0.1f;

        if(ctx_->GetKeyPressKP6())
            cam_pitch_ -= 0.1f;


        if(auto door_anim_sptr = door_anim_.lock()) {

	        if(ctx_->GetKeyPress7()) {
	            door_anim_sptr->TakeAction(0);
	        }

	        if(ctx_->GetKeyPress8())
	            door_anim_sptr->TakeAction(1);
	    }

	    if(auto obj_conv_sptr = obj_conv_.lock()) {

	        if(ctx_->GetKeyPress5()) {
	            obj_conv_sptr->TakeAction(0);
	        }

	        if(ctx_->GetKeyPress6())
	            obj_conv_sptr->TakeAction(1);

	    }

	    if(auto agent_sptr = agent_.lock()) {

	        // Pick
		    if(ctx_->GetKeyPress1()) {
		    	glm::vec3 fromPosition = main_camera_->Position;
		    	glm::vec3 toPosition = main_camera_->Front * 3.0f + fromPosition;
		    	agent_sptr->PickUp(inventory_, fromPosition, toPosition);
		    	renderer_->BakeScene(scene_->world_.get());
		    }

		    // Put
		    if(ctx_->GetKeyPress2()) {
		    	glm::vec3 fromPosition = main_camera_->Position;
		    	glm::vec3 toPosition = main_camera_->Front * 3.0f + fromPosition;
		    	agent_sptr->PutDown(inventory_, fromPosition, toPosition);
		    	renderer_->BakeScene(scene_->world_.get());
		    }

		    // Rotate
		    if(ctx_->GetKeyPress3()) {
		    	glm::vec3 fromPosition = main_camera_->Position;
		    	glm::vec3 toPosition = main_camera_->Front * 3.0f + fromPosition;
		    	agent_sptr->RotateObject(glm::vec3(1.57,0,0), fromPosition, toPosition);
		    	renderer_->BakeScene(scene_->world_.get());
		    }
		    
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
        renderer_->StepRender(scene_->world_.get());
        ctx_->SwapBuffer();
        ctx_->PollEvent();

        return "NavTarget";
	}

	//=======================================Task 6=============================
	// This is a dummy task for only testing crowd and navigation features

	Task_Crowd::Task_Crowd(
		std::shared_ptr<render_engine::Render> renderer,
		std::shared_ptr<Map> map) : iterations_(0),
								      scene_(map),
								      renderer_(renderer),
								      ctx_(renderer->ctx_),
								      main_camera_(),
								      cam_pitch_(0)
	{
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
		renderer_->sunlight_.ambient = glm::vec3(0.02,0.02,0.02);
        renderer_->lighting_.exposure = 0.4f;
        renderer_->lighting_.indirect_strength = 0.25f;
        renderer_->lighting_.traceshadow_distance = 0.3f;
        renderer_->lighting_.propagation_distance = 0.3f;
        renderer_->lighting_.sample_factor = 0.7f;
        renderer_->lighting_.boost_ambient = 0.01f;
        renderer_->lighting_.shadow_bias_scale = 0.0003f;
        renderer_->lighting_.linear_voxelize = true;

		iterations_ = 0;
		crowd_->Reset();
		scene_->ResetMap();
		scene_->ClearRules();
		scene_->CreateLabel(crate1, "crate");
		scene_->CreateSectionType(test_floor, test_wall, door);
		scene_->GenerateTestFloorPlan(5, 5);
	  	
		std::weak_ptr<RobotBase> obj;

		obj = scene_->world_->LoadRobot(
	        crate1,
	        btVector3(2, 0, 2),
	        btQuaternion(btVector3(1,0,0),0),
	        btVector3(1, 1, 1),
	        "crate",
	        true
	    );
	    if(auto obj_sptr = obj.lock())
	   		obj_sptr->move(false);

	   	obj = scene_->world_->LoadRobot(
	        crate1,
	        btVector3(4, 0, 2),
	        btQuaternion(btVector3(1,0,0),0),
	        btVector3(1, 1, 1),
	        "crate",
	        true
	    );
	   	if(auto obj_sptr = obj.lock())
	   		obj_sptr->move(false);

	   	obj = scene_->world_->LoadRobot(
	        crate1,
	        btVector3(4, 0, 4),
	        btQuaternion(btVector3(1,0,0),0),
	        btVector3(1, 1, 1),
	        "crate",
	        true
	    );
	   	if(auto obj_sptr = obj.lock())
	   		obj_sptr->move(false);

	   	obj = scene_->world_->LoadRobot(
	        crate1,
	        btVector3(6, 0, 6),
	        btQuaternion(btVector3(1,0,0),0),
	        btVector3(1, 1, 1),
	        "crate",
	        true
	    );
	   	if(auto obj_sptr = obj.lock())
	   		obj_sptr->move(false);

	   	// Load Agent
	   	agent_ = scene_->world_->LoadRobot(
	        husky,
	        btVector3(0,0.001,2),
	        btQuaternion(btVector3(-1,0,0),1.57),
	        btVector3(1, 1, 1),
	        "husky",
	        true
	    );

	    obj = scene_->world_->LoadRobot(
	        crate03,
	        btVector3(1, 0, 0),
	        btQuaternion(btVector3(1,0,0),0),
	        btVector3(1, 1, 1),
	        "crate_03",
	        true
	    );
	   	if(auto obj_sptr = obj.lock())
	   		obj_sptr->move(false);

	   	if(auto agent_sptr = agent_.lock()) {
	   		agent_sptr->move(true);
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
	    return "NavTarget";
	}


		std::string Task_Crowd::NavTarget() {
		
		CarveShape cs;
		cs.position = glm::vec2(main_camera_->Position.x, main_camera_->Position.z);
		cs.radius = 1.3f;
		cs.moving = false;

		if(ctx_->GetKeyPressUp()) {
			if(auto agent_sptr = agent_.lock()) {
	            agent_sptr->MoveForward(0.5f);
	            cs.moving = true;
	        }
		}

        if(ctx_->GetKeyPressDown()) {
        	if(auto agent_sptr = agent_.lock()) {
	            agent_sptr->MoveBackward(0.5f);
	            cs.moving = true;
	        }
        }

        if(ctx_->GetKeyPressLeft()) {
        	if(auto agent_sptr = agent_.lock()) {
	            agent_sptr->TurnLeft(0.5f);
	        	cs.moving = true;
	        }
        }

        if(ctx_->GetKeyPressRight()) {
        	if(auto agent_sptr = agent_.lock()) {
	            agent_sptr->TurnRight(0.5f);
	        	cs.moving = true;
	        }
        }

        if(ctx_->GetKeyPressKP9()) 
            cam_pitch_ += 0.1f;

        if(ctx_->GetKeyPressKP6())
            cam_pitch_ -= 0.1f;

        cam_pitch_ = glm::clamp(cam_pitch_, -45.0f, 45.0f);
        scene_->world_->rotate_camera(main_camera_, cam_pitch_);
		iterations_++;

		// Generate New Agent
		if(iterations_ % 400 == 0) {
			crowd_->SpawnAgent(glm::vec3(0,0,0), crate03, "moving_agent");
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

        return "NavTarget";
	}
}