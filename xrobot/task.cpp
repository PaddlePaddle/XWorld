#include "task.h"

namespace xrobot {

	NavTask0::NavTask0() : reset_count_(0),
						 iterations_(0),
						 scene_(nullptr),
						 agent_(nullptr),
						 cam_pitch_(0.0f),
						 ctx_(nullptr),
						 renderer_(nullptr),
						 c0_(nullptr) {}

	NavTask0::NavTask0(render_engine::GLContext * ctx, 
			           render_engine::Render * renderer,
			           Map * map) : reset_count_(0),
						            iterations_(0),
            						scene_(map),
            						agent_(nullptr),
            						cam_pitch_(0.0f),
            						ctx_(ctx),
            						renderer_(renderer),
            						c0_(nullptr) {}

	NavTask0::~NavTask0() {
		delete scene_;
		delete renderer_;
	}

	TaskStages NavTask0::GetStages() {
		TaskStages stages;
		stages["idle"] = std::bind(&NavTask0::Start, this);
		stages["NavTarget"] = std::bind(&NavTask0::NavTarget, this);
		return stages;
	}

	std::string NavTask0::Start() {

		iterations_ = 0;

        if(reset_count_++ % 1000 == 0)
            scene_->ResetMap();
        else
            scene_->ClearMap();

        scene_->ClearRules();
		scene_->CreateLabel("r2d2/r2d2.urdf", "r2d2");
		scene_->CreateSectionType(floor0, wall, door0);
		glm::vec3 startPosition = scene_->GenerateTestFloorPlan(5, 5);
		startPosition = glm::vec3(0,0,0);

		renderer_->sunlight_.direction = glm::vec3(1,2,1);
	    target_ = scene_->world_->LoadURDF(
	        "r2d2/r2d2.urdf",
	        btVector3(startPosition.x + 2,0.01,startPosition.z),
	        btQuaternion(btVector3(1,0,0),0),
	        0.01f,
	        "r2d2",
	        true
	    );
	    target_->move(true);
	    target_->DisableSleeping();

	    agent_ = scene_->world_->LoadURDF(
	        "husky/husky.urdf",
	        btVector3(startPosition.x,0.01,startPosition.z),
	        btQuaternion(btVector3(-1,0,0),1.57)
	    );
	    agent_->DisableSleeping();

	    c0_ = scene_->world_->add_camera(vec3(0,0,0),
	    		vec3(0.3,1.3,0.0), (float) 16 / 9);

	    scene_->world_->attach_camera(c0_, agent_);

	    renderer_->Init(c0_);
	    
	    scene_->world_->BulletStep();

	    return "NavTarget";
	}

	std::string NavTask0::NavTarget() {
		agent_->Freeze(!iterations_);
        if(ctx_->GetKeyPressUp())
        {
            agent_->MoveForward(0.01f);
        }

        if(ctx_->GetKeyPressDown())
        {
            agent_->MoveBackward(0.01f);
        }

        if(ctx_->GetKeyPressLeft())
        {
            agent_->TurnLeft(0.01f);
        }

        if(ctx_->GetKeyPressRight())
        {
            agent_->TurnRight(0.01f);
        }

        if(ctx_->GetKeyPressKP9())
        {
            cam_pitch_ += 0.1f;
        }
        if(ctx_->GetKeyPressKP6())
        {
            cam_pitch_ -= 0.1f;
        }

        cam_pitch_ = glm::clamp(cam_pitch_, -45.0f, 45.0f);
        scene_->world_->rotate_camera(c0_, cam_pitch_); 


        std::vector<ObjectAttributes> temp;
        scene_->world_->QueryObjectByLabel("r2d2", temp);

        for(int i = 0; i < temp.size(); ++i)
        {
        	glm::vec3 aabb_min = temp[i].aabb_min - glm::vec3(1);
        	glm::vec3 aabb_max = temp[i].aabb_max + glm::vec3(1);

        	if(aabb_min.x < c0_->Position.x &&
        	   aabb_max.x > c0_->Position.x && 
        	   aabb_min.z < c0_->Position.z &&
        	   aabb_max.z > c0_->Position.z) 
        	{

        		// glm::vec3 fromPosition = c0_->Position;
        		// glm::vec3 toPosition = c0_->Front * 2.0f + fromPosition;
        		// int res = scene_->world_->RayTest(fromPosition, toPosition);

        		// if(res == temp[i].bullet_id)
        			return "idle";
        	}
        }

        // Rotate
        float s = 3 * sin(iterations_ * 0.002f) + 4;
        float c = 3 * cos(iterations_ * 0.002f) + 4;

        btTransform transform;
        transform.setIdentity();
        transform.setOrigin(btVector3(c, 0, s));
        transform.setRotation(btQuaternion(btVector3(0,1,0), -iterations_ * 0.002f));
        scene_->world_->SetTransformation(target_, transform);

        printf("count: %d\n", iterations_);
        if(iterations_++ > 12000) {
        	printf("--------Fail!---------\n");
        	return "idle";
        }

        scene_->world_->BulletStep();   
        renderer_->StepRender(scene_->world_);
        ctx_->SwapBuffer();
        ctx_->PollEvent();

        return "NavTarget";
	}




	NavTask1::NavTask1() : reset_count_(0),
						   iterations_(0),
						 scene_(nullptr),
						 agent_(nullptr),
						 lidar_(nullptr),
						 cam_pitch_(0.0f),
						 ctx_(nullptr),
						 renderer_(nullptr),
						 c0_(nullptr) {}

	NavTask1::NavTask1(render_engine::GLContext * ctx, 
			render_engine::Render * renderer,
			Map * map) : reset_count_(0),
						 iterations_(0),
						scene_(map),
						agent_(nullptr),
						lidar_(new Lidar(map->world_, 180, 3.0f)),
						cam_pitch_(0.0f),
						ctx_(ctx),
						renderer_(renderer),
						c0_(nullptr) {}

	NavTask1::~NavTask1() {
		delete scene_;
		delete renderer_;
		delete lidar_;
	}

	TaskStages NavTask1::GetStages() {
		TaskStages stages;
		stages["idle"] = std::bind(&NavTask1::Start, this);
        stages["Start"] = std::bind(&NavTask1::Start, this);
		stages["NavTarget"] = std::bind(&NavTask1::NavTarget, this);
		return stages;
	}

	std::string NavTask1::Start() {

		iterations_ = 0;

		scene_->ClearRules();
		scene_->CreateLabel(crate1, "crate1");
	    scene_->CreateSectionType(floor0, wall, door0);
	    scene_->CreateSpawnOnFloor(crate1);
	    scene_->CreateSpawnOnFloor(crate03);
	    scene_->CreateSpawnOnObject(crate03);

        if(reset_count_++ % 1000 == 0)
            scene_->ResetMap();
        else
            scene_->ClearMap();

		glm::vec3 startPosition = scene_->GenerateFloorPlan(10, 10);
	    scene_->Spawn(3, 2, 0);

	    agent_ = scene_->world_->LoadURDF(
	        "husky/husky.urdf",
	        btVector3(startPosition.x,0.01,startPosition.z),
	        btQuaternion(btVector3(-1,0,0),1.57)
	    );
	    agent_->DisableSleeping();

	    c0_ = scene_->world_->add_camera(vec3(0,0,0),
	    		vec3(0.3,1.3,0.0), (float) 4 / 3);

	    scene_->world_->attach_camera(c0_, agent_);

	    renderer_->Init(c0_);
	    
	    scene_->world_->BulletStep();

	    return "NavTarget";
	}

	std::string NavTask1::NavTarget() {
		agent_->Freeze(!iterations_);

        if(ctx_->GetKeyPressUp())
        {
            agent_->MoveForward(0.01f);
        }

        if(ctx_->GetKeyPressDown())
        {
            agent_->MoveBackward(0.01f);
        }

        if(ctx_->GetKeyPressLeft())
        {
            agent_->TurnLeft(0.01f);
        }

        if(ctx_->GetKeyPressRight())
        {
            agent_->TurnRight(0.01f);
        }

        if(ctx_->GetKeyPressKP9())
        {
            cam_pitch_ += 0.1f;
        }
        if(ctx_->GetKeyPressKP6())
        {
            cam_pitch_ -= 0.1f;
        }

        cam_pitch_ = glm::clamp(cam_pitch_, -45.0f, 45.0f);
        scene_->world_->rotate_camera(c0_, cam_pitch_); 

		std::vector<ObjectAttributes> temp;
        scene_->world_->QueryObjectByLabel("crate1", temp);

        for(int i = 0; i < temp.size(); ++i)
        {
        	glm::vec3 aabb_min = temp[i].aabb_min - glm::vec3(1);
        	glm::vec3 aabb_max = temp[i].aabb_max + glm::vec3(1);

        	if(aabb_min.x < c0_->Position.x &&
        	   aabb_max.x > c0_->Position.x && 
        	   aabb_min.z < c0_->Position.z &&
        	   aabb_max.z > c0_->Position.z) 
        	{

        		glm::vec3 fromPosition = c0_->Position;
        		glm::vec3 toPosition = c0_->Front * 2.0f + fromPosition;
        		int res = scene_->world_->RayTest(fromPosition, toPosition);

        		if(res == temp[i].bullet_id)
        			return "idle";
        	}
        }

        std::vector<RayTestInfo> batch_raycast_result;
        lidar_->Update(c0_->Front,
                      c0_->WorldUp,
                      c0_->Position - glm::vec3(0.0f,0.8f,0));
        batch_raycast_result = lidar_->GetResult();

        for (int i = 0; i < batch_raycast_result.size(); ++i)
        {
            if(batch_raycast_result[i].bullet_id < 0)
            {
                renderer_->UpdateRay(i, glm::vec3(0), glm::vec3(0));
            } else {
                renderer_->UpdateRay(i, c0_->Position - glm::vec3(0.0f,0.8f,0),
                        batch_raycast_result[i].pos);
            }
        }

        //printf("count: %d\n", iterations_);
        if(iterations_++ > 1200000) {
        	printf("--------Fail!---------\n");
        	return "idle";
        }

        scene_->world_->BulletStep();   
        renderer_->StepRender(scene_->world_);
        ctx_->SwapBuffer();
        ctx_->PollEvent();

        return "NavTarget";
	}






	NavTask2::NavTask2() : reset_count_(0),
						   iterations_(0),
						 scene_(nullptr),
						 agent_(nullptr),
						 lidar_(nullptr),
						 cam_pitch_(0.0f),
						 ctx_(nullptr),
						 renderer_(nullptr),
						 c0_(nullptr) {}

	NavTask2::NavTask2(render_engine::GLContext * ctx, 
			render_engine::Render * renderer,
			Map * map) : reset_count_(0),
						 iterations_(0),
						scene_(map),
						agent_(nullptr),
						lidar_(new Lidar(map->world_, 180, 3.0f)),
						cam_pitch_(0.0f),
						ctx_(ctx),
						renderer_(renderer),
						c0_(nullptr) {}

	NavTask2::~NavTask2() {
		delete scene_;
		delete renderer_;
		delete lidar_;
	}

	TaskStages NavTask2::GetStages() {
		TaskStages stages;
		stages["idle"] = std::bind(&NavTask2::Start, this);
		stages["NavTarget"] = std::bind(&NavTask2::NavTarget, this);
		return stages;
	}

	std::string NavTask2::Start() {

			iterations_ = 0;

			scene_->ClearRules();
		    scene_->CreateLabel(crate03, "crate03");
		    // scene_->CreateSectionType(floor0, wall, door0);
		    // scene_->CreateSectionType(floor1, wall, door0);
		    scene_->CreateSectionType(floor2, wall, door0);
		    scene_->CreateSpawnOnFloor(crate03);

        if(reset_count_++ % 1000 == 0)
            scene_->ResetMap();
        else
            scene_->ClearMap();

		glm::vec3 startPosition = scene_->GenerateFloorPlan(10, 10);
	    scene_->Spawn(10, 0, 0);

	    renderer_->sunlight_.direction = glm::vec3(1,2,1);
	    agent_ = scene_->world_->LoadURDF(
	        "husky/husky.urdf",
	        btVector3(startPosition.x,0.01,startPosition.z),
	        btQuaternion(btVector3(-1,0,0),1.57)
	    );
	    agent_->DisableSleeping();

	    c0_ = scene_->world_->add_camera(vec3(0,0,0),
	    		vec3(0.3,1.3,0.0), (float) 4 / 3);

	    scene_->world_->attach_camera(c0_, agent_);
	    renderer_->Init(c0_);
	    scene_->world_->BulletStep();

	    return "NavTarget";
	}

	std::string NavTask2::NavTarget() {
		agent_->Freeze(!iterations_);
        if(ctx_->GetKeyPressUp())
        {
            agent_->MoveForward(0.01f);
        }

        if(ctx_->GetKeyPressDown())
        {
            agent_->MoveBackward(0.01f);
        }

        if(ctx_->GetKeyPressLeft())
        {
            agent_->TurnLeft(0.01f);
        }

        if(ctx_->GetKeyPressRight())
        {
            agent_->TurnRight(0.01f);
        }

        if(ctx_->GetKeyPressKP9())
        {
            cam_pitch_ += 0.1f;
        }
        if(ctx_->GetKeyPressKP6())
        {
            cam_pitch_ -= 0.1f;
        }

        cam_pitch_ = glm::clamp(cam_pitch_, -45.0f, 45.0f);
        scene_->world_->rotate_camera(c0_, cam_pitch_); 

		std::vector<ObjectAttributes> temp;
        scene_->world_->QueryObjectByLabel("crate03", temp);

        printf("========================= %d\n", temp.size());

        for(int i = 0; i < temp.size(); ++i)
        {
        	glm::vec3 aabb_min = temp[i].aabb_min - glm::vec3(1);
        	glm::vec3 aabb_max = temp[i].aabb_max + glm::vec3(1);

        	if(aabb_min.x < c0_->Position.x &&
        	   aabb_max.x > c0_->Position.x && 
        	   aabb_min.z < c0_->Position.z &&
        	   aabb_max.z > c0_->Position.z) 
        	{

        		// glm::vec3 fromPosition = c0_->Position;
        		// glm::vec3 toPosition = c0_->Front * 2.0f + fromPosition;
        		// int res = scene_->world_->RayTest(fromPosition, toPosition);

        		std::vector<ObjectDirections> temp_dir;
        		scene_->world_->QueryObjectDirectionByLabel("crate03", c0_->Front, 
        				c0_->Position, temp_dir);
        		if(temp_dir[i].dirs[0] < 1.57 * 0.5f)
        			return "idle";
        	}
        }

        std::vector<RayTestInfo> batch_raycast_result;
        lidar_->Update(c0_->Front,
                      c0_->WorldUp,
                      c0_->Position - glm::vec3(0.0f,0.8f,0));
        batch_raycast_result = lidar_->GetResult();

        for (int i = 0; i < batch_raycast_result.size(); ++i)
        {
            if(batch_raycast_result[i].bullet_id < 0)
            {
                renderer_->UpdateRay(i, glm::vec3(0), glm::vec3(0));
            } else {
                renderer_->UpdateRay(i, c0_->Position - glm::vec3(0.0f,0.8f,0),
                        batch_raycast_result[i].pos);
            }
        }

        printf("count: %d\n", iterations_);
        if(iterations_++ > 12000) {
        	printf("--------Fail!---------\n");
        	return "idle";
        }

        scene_->world_->BulletStep();   
        renderer_->StepRender(scene_->world_);
        ctx_->SwapBuffer();
        ctx_->PollEvent();

        return "NavTarget";
	}















	NavTask3::NavTask3() : reset_count_(0),
						 scene_(nullptr),
						 agent_(nullptr),
						 cam_pitch_(0.0f),
						 ctx_(nullptr),
						 renderer_(nullptr),
						 c0_(nullptr) {}

	NavTask3::NavTask3(render_engine::GLContext * ctx, 
			render_engine::Render * renderer,
			Map * map) : reset_count_(0),
												scene_(map),
												agent_(nullptr),
												cam_pitch_(0.0f),
												ctx_(ctx),
												renderer_(renderer),
												c0_(nullptr) {}

	NavTask3::~NavTask3() {
		delete scene_;
		delete renderer_;
	}

	TaskStages NavTask3::GetStages() {
		TaskStages stages;
		stages["idle"] = std::bind(&NavTask3::Start, this);
		stages["NavTarget"] = std::bind(&NavTask3::NavTarget, this);
		return stages;
	}

	std::string NavTask3::Start() {

        if(reset_count_++ % 1000 == 0)
            scene_->ResetMap();
        else
            scene_->ClearMap();


        scene_->world_->LoadOBJ(
	        "/home/ziyuli/Downloads/db7814ade2a34ceea8c33979df3cf85f/db7814ade2a34ceea8c33979df3cf85f.obj",
	        btVector3(0,0,0),
	        btQuaternion(btVector3(-1,0,0),1.57),
	        btVector3(0.3,0.3,0.3),
	        "room",
	        0,
	        false,
	        true
	    );


		glm::vec3 startPosition = glm::vec3(0,0,0);
	    agent_ = scene_->world_->LoadURDF(
	        "husky/husky.urdf",
	        btVector3(startPosition.x,0.21,startPosition.z),
	        btQuaternion(btVector3(-1,0,0),1.57)
	    );
	    agent_->DisableSleeping();

	    c0_ = scene_->world_->add_camera(vec3(0,0,0),
	    		vec3(0.3,1.3,0.0), (float) 16 / 9);

	    scene_->world_->attach_camera(c0_, agent_);
	    renderer_->Init(c0_);
	    scene_->world_->BulletStep();

	    return "NavTarget";
	}

	std::string NavTask3::NavTarget() {
		agent_->Freeze();
        if(ctx_->GetKeyPressUp())
        {
            agent_->MoveForward(0.01f);
        }

        if(ctx_->GetKeyPressDown())
        {
            agent_->MoveBackward(0.01f);
        }

        if(ctx_->GetKeyPressLeft())
        {
            agent_->TurnLeft(0.01f);
        }

        if(ctx_->GetKeyPressRight())
        {
            agent_->TurnRight(0.01f);
        }

        if(ctx_->GetKeyPressKP9())
        {
            cam_pitch_ += 0.1f;
        }
        if(ctx_->GetKeyPressKP6())
        {
            cam_pitch_ -= 0.1f;
        }

        cam_pitch_ = glm::clamp(cam_pitch_, -45.0f, 45.0f);
        scene_->world_->rotate_camera(c0_, cam_pitch_); 

		std::vector<ObjectAttributes> temp;
        scene_->world_->QueryObjectByLabel("crate03", temp);

        for(int i = 0; i < temp.size(); ++i)
        {
        	glm::vec3 aabb_min = temp[i].aabb_min - glm::vec3(1);
        	glm::vec3 aabb_max = temp[i].aabb_max + glm::vec3(1);

        	if(aabb_min.x < c0_->Position.x &&
        	   aabb_max.x > c0_->Position.x && 
        	   aabb_min.z < c0_->Position.z &&
        	   aabb_max.z > c0_->Position.z) 
        	{

        		glm::vec3 fromPosition = c0_->Position;
        		glm::vec3 toPosition = c0_->Front * 2.0f + fromPosition;
        		int res = scene_->world_->RayTest(fromPosition, toPosition);

        		if(res == temp[i].bullet_id)
        			return "idle";
        	}
        }

        scene_->world_->BulletStep();   
        renderer_->StepRender(scene_->world_);
        ctx_->SwapBuffer();
        ctx_->PollEvent();

        return "NavTarget";
	}













	NavTask4::NavTask4() : reset_count_(0),
						 iterations_(0),
						 scene_(nullptr),
						 agent_(nullptr),
						 cam_pitch_(0.0f),
						 ctx_(nullptr),
						 renderer_(nullptr),
						 c0_(nullptr) {}

	NavTask4::NavTask4(render_engine::GLContext * ctx, 
			render_engine::Render * renderer,
			MapSuncg * map) : reset_count_(0),
							  iterations_(0),
												scene_(map),
												agent_(nullptr),
												cam_pitch_(0.0f),
												ctx_(ctx),
												renderer_(renderer),
												c0_(nullptr) {}

	NavTask4::~NavTask4() {
		delete scene_;
		delete renderer_;
	}

	TaskStages NavTask4::GetStages() {
		TaskStages stages;
		stages["idle"] = std::bind(&NavTask4::Start, this);
		stages["NavTarget"] = std::bind(&NavTask4::NavTarget, this);
		return stages;
	}

	std::string NavTask4::Start() {

		printf("-----Find Fruit Bowl------\n");

		iterations_ = 0;
        scene_->ResetMap();
        scene_->LoadCategoryCSV(metadata_models.c_str());
	    scene_->SetRemoveAll( kRemoveStairs );
	    scene_->LoadJSON(house0.c_str(), data_dir.c_str(), true);
	    scene_->SetMapSize(-8, -8, 6, 6);

    	agent_ = scene_->world_->LoadURDF(
	        "husky/robot_kuka.urdf",
	        btVector3(-6,0.21,-1),
	        btQuaternion(btVector3(-1,0,0),1.57),
	        0.6f
	    );
    	agent_->DisableSleeping();

    	pos_0_ = 0.0f;
	    pos_1_ = 0.0f;
	    pos_2_ = 0.0f;
	    pos_3_ = 0.0f;
	    pos_4_ = 0.0f;
	    pos_5_ = 0.0f;
	    pos_6_ = 0.0f;
	    pos_7_ = 0.0f;

    	c0_ = scene_->world_->add_camera(vec3(0,0,0), vec3(0.3,1.2,0.0), (float) 16 / 9);
    	scene_->world_->attach_camera(c0_, agent_);
    	renderer_->Init(c0_);
    	scene_->world_->BulletStep();

	    return "NavTarget";
	}

	std::string NavTask4::NavTarget() {


		agent_->Freeze(!iterations_);
        if(ctx_->GetKeyPressUp())
        {
            agent_->MoveForward(0.01f);
        }

        if(ctx_->GetKeyPressDown())
        {
            agent_->MoveBackward(0.01f);
        }

        if(ctx_->GetKeyPressLeft())
        {
            agent_->TurnLeft(0.01f);
        }

        if(ctx_->GetKeyPressRight())
        {
            agent_->TurnRight(0.01f);
        }

		if(ctx_->GetKeyPress1())
        {
            pos_0_ += 0.0025f;
        }
        if(ctx_->GetKeyPress2())
        {
            pos_0_ -= 0.0025f;
        }

        if(ctx_->GetKeyPress3())
        {
            pos_1_ += 0.0025f;
        }
        if(ctx_->GetKeyPress4())
        {
            pos_1_ -= 0.0025f;
        }

        if(ctx_->GetKeyPress5())
        {
            pos_2_ += 0.0025f;
        }
        if(ctx_->GetKeyPress6())
        {
            pos_2_ -= 0.0025f;
        }

        if(ctx_->GetKeyPress7())
        {
            pos_3_ += 0.0025f;
        }
        if(ctx_->GetKeyPress8())
        {
            pos_3_ -= 0.0025f;
        }

        if(ctx_->GetKeyPress9())
        {
            pos_4_ += 0.0025f;
        }
        if(ctx_->GetKeyPress0())
        {
            pos_4_ -= 0.0025f;
        }

        if(ctx_->GetKeyPressKP1())
        {
            pos_5_ += 0.0025f;
        }
        if(ctx_->GetKeyPressKP2())
        {
            pos_5_ -= 0.0025f;
        }

        if(ctx_->GetKeyPressKP4())
        {
            pos_6_ += 0.0025f;
        }
        if(ctx_->GetKeyPressKP5())
        {
            pos_6_ -= 0.0025f;
        }

        if(ctx_->GetKeyPressKP7())
        {
            pos_7_ = 0.05f;
        }
        if(ctx_->GetKeyPressKP8())
        {
            pos_7_ = 0.005f;
        }

        if(ctx_->GetKeyPressKP9())
        {
            cam_pitch_ += 0.2f;
        }
        if(ctx_->GetKeyPressKP6())
        {
            cam_pitch_ -= 0.2f;
        }

        cam_pitch_ = glm::clamp(cam_pitch_, -45.0f, 45.0f);
        pos_0_ = glm::clamp(pos_0_, -1.0f, 1.0f);
        pos_1_ = glm::clamp(pos_1_, -1.0f, 1.0f);
        pos_2_ = glm::clamp(pos_2_, -1.0f, 1.0f);
        pos_3_ = glm::clamp(pos_3_, -1.0f, 1.0f);
        pos_4_ = glm::clamp(pos_4_, -1.0f, 1.0f);
        pos_5_ = glm::clamp(pos_5_, -1.0f, 1.0f);
        pos_6_ = glm::clamp(pos_6_, -1.0f, 1.0f);

        Joint *j;
        j = agent_->joints_list_[2];
        j->SetJointMotorControlPosition(pos_0_, 0.1f, 1.0f,100.0f);
        j = agent_->joints_list_[3];
        j->SetJointMotorControlPosition(pos_1_, 0.1f, 1.0f,100.0f);
        j = agent_->joints_list_[4];
        j->SetJointMotorControlPosition(pos_2_, 0.1f, 1.0f,100.0f);
        j = agent_->joints_list_[5];
        j->SetJointMotorControlPosition(pos_3_, 0.1f, 1.0f,100.0f);
        j = agent_->joints_list_[6];
        j->SetJointMotorControlPosition(pos_4_, 0.1f, 1.0f,100.0f);
        j = agent_->joints_list_[7];
        j->SetJointMotorControlPosition(pos_5_, 0.1f, 1.0f,100.0f);
        j = agent_->joints_list_[8];
        j->SetJointMotorControlPosition(pos_6_, 0.1f, 1.0f,100.0f);
        j = agent_->joints_list_[10];
        j->SetJointMotorControlPosition(pos_7_, 0.1f, 1.0f,10.0f);
        j = agent_->joints_list_[12];
        j->SetJointMotorControlPosition(pos_7_, 0.1f, 1.0f,10.0f);

        scene_->world_->rotate_camera(c0_, cam_pitch_); 

		std::vector<ObjectAttributes> temp;
        scene_->world_->QueryObjectByLabel("fruit_bowl", temp);

        for(int i = 0; i < temp.size(); ++i)
        {
        	glm::vec3 aabb_min = temp[i].aabb_min - glm::vec3(0.65);
        	glm::vec3 aabb_max = temp[i].aabb_max + glm::vec3(0.65);

        	if(aabb_min.x < c0_->Position.x &&
        	   aabb_max.x > c0_->Position.x && 
        	   aabb_min.z < c0_->Position.z &&
        	   aabb_max.z > c0_->Position.z) 
        	{

        		// glm::vec3 fromPosition = c0_->Position;
        		// glm::vec3 toPosition = c0_->Front * 2.0f + fromPosition;
        		// int res = scene_->world_->RayTest(fromPosition, toPosition);

        		std::vector<ObjectDirections> temp_dir;
        		scene_->world_->QueryObjectDirectionByLabel("fruit_bowl", c0_->Front, 
        				c0_->Position, temp_dir);
        		if(temp_dir[i].dirs[0] < 1.57 * 0.3f)
        			return "idle";
        	}
        }

        printf("count: %d\n", iterations_);
        if(iterations_++ > 12000) {
        	printf("--------Fail!---------\n");
        	return "idle";
        }

        scene_->world_->BulletStep();   
        renderer_->StepRender(scene_->world_);
        ctx_->SwapBuffer();
        ctx_->PollEvent();

        return "NavTarget";
	}










	NavTask5::NavTask5() : reset_count_(0),
						 iterations_(0),
						 scene_(nullptr),
						 agent_(nullptr),
						 cam_pitch_(0.0f),
						 ctx_(nullptr),
						 renderer_(nullptr),
						 c0_(nullptr) {}

	NavTask5::NavTask5(render_engine::GLContext * ctx, 
			render_engine::Render * renderer,
			MapSuncg * map) : reset_count_(0),
							  iterations_(0),
												scene_(map),
												agent_(nullptr),
												cam_pitch_(0.0f),
												ctx_(ctx),
												renderer_(renderer),
												c0_(nullptr) {}

	NavTask5::~NavTask5() {
		delete scene_;
		delete renderer_;
	}

	TaskStages NavTask5::GetStages() {
		TaskStages stages;
		stages["idle"] = std::bind(&NavTask5::Start, this);
		stages["NavTarget"] = std::bind(&NavTask5::NavTarget, this);
		return stages;
	}

	std::string NavTask5::Start() {

		printf("-----Touch Pan------\n");

		iterations_ = 0;
        scene_->ResetMap();
        scene_->LoadCategoryCSV(metadata_models.c_str());
	    scene_->SetRemoveAll( kRemoveStairs );
	    scene_->LoadJSON(house0.c_str(), data_dir.c_str(), true);
	    scene_->SetMapSize(-8, -8, 6, 6);

    	agent_ = scene_->world_->LoadURDF(
	        "husky/robot_kuka.urdf",
	        btVector3(-6,0.21,-1),
	        btQuaternion(btVector3(-1,0,0),1.57),
	        0.6f,
	        "agent"
	    );
    	agent_->DisableSleeping();

    	pos_0_ = 0.0f;
	    pos_1_ = 0.0f;
	    pos_2_ = 0.0f;
	    pos_3_ = 0.0f;
	    pos_4_ = 0.0f;
	    pos_5_ = 0.0f;
	    pos_6_ = 0.0f;
	    pos_7_ = 0.0f;

    	c0_ = scene_->world_->add_camera(vec3(0,0,0), vec3(0.3,1.2,0.0), (float) 16 / 9);
    	scene_->world_->attach_camera(c0_, agent_);
    	renderer_->Init(c0_);
    	scene_->world_->BulletStep();

	    return "NavTarget";
	}

	std::string NavTask5::NavTarget() {

		agent_->Freeze(!iterations_);
        if(ctx_->GetKeyPressUp())
        {
            agent_->MoveForward(0.01f);
        }

        if(ctx_->GetKeyPressDown())
        {
            agent_->MoveBackward(0.01f);
        }

        if(ctx_->GetKeyPressLeft())
        {
            agent_->TurnLeft(0.01f);
        }

        if(ctx_->GetKeyPressRight())
        {
            agent_->TurnRight(0.01f);
        }

		if(ctx_->GetKeyPress1())
        {
            pos_0_ += 0.0025f;
        }
        if(ctx_->GetKeyPress2())
        {
            pos_0_ -= 0.0025f;
        }

        if(ctx_->GetKeyPress3())
        {
            pos_1_ += 0.0025f;
        }
        if(ctx_->GetKeyPress4())
        {
            pos_1_ -= 0.0025f;
        }

        if(ctx_->GetKeyPress5())
        {
            pos_2_ += 0.0025f;
        }
        if(ctx_->GetKeyPress6())
        {
            pos_2_ -= 0.0025f;
        }

        if(ctx_->GetKeyPress7())
        {
            pos_3_ += 0.0025f;
        }
        if(ctx_->GetKeyPress8())
        {
            pos_3_ -= 0.0025f;
        }

        if(ctx_->GetKeyPress9())
        {
            pos_4_ += 0.0025f;
        }
        if(ctx_->GetKeyPress0())
        {
            pos_4_ -= 0.0025f;
        }

        if(ctx_->GetKeyPressKP1())
        {
            pos_5_ += 0.0025f;
        }
        if(ctx_->GetKeyPressKP2())
        {
            pos_5_ -= 0.0025f;
        }

        if(ctx_->GetKeyPressKP4())
        {
            pos_6_ += 0.0025f;
        }
        if(ctx_->GetKeyPressKP5())
        {
            pos_6_ -= 0.0025f;
        }

        if(ctx_->GetKeyPressKP7())
        {
            pos_7_ = 0.05f;
        }
        if(ctx_->GetKeyPressKP8())
        {
            pos_7_ = 0.005f;
        }

        if(ctx_->GetKeyPressKP9())
        {
            cam_pitch_ += 0.2f;
        }
        if(ctx_->GetKeyPressKP6())
        {
            cam_pitch_ -= 0.2f;
        }

        cam_pitch_ = glm::clamp(cam_pitch_, -45.0f, 45.0f);
        pos_0_ = glm::clamp(pos_0_, -1.0f, 1.0f);
        pos_1_ = glm::clamp(pos_1_, -1.0f, 1.0f);
        pos_2_ = glm::clamp(pos_2_, -1.0f, 1.0f);
        pos_3_ = glm::clamp(pos_3_, -1.0f, 1.0f);
        pos_4_ = glm::clamp(pos_4_, -1.0f, 1.0f);
        pos_5_ = glm::clamp(pos_5_, -1.0f, 1.0f);
        pos_6_ = glm::clamp(pos_6_, -1.0f, 1.0f);

        Joint *j;
        j = agent_->joints_list_[2];
        j->SetJointMotorControlPosition(pos_0_, 0.1f, 1.0f,1000.0f);
        j = agent_->joints_list_[3];
        j->SetJointMotorControlPosition(pos_1_, 0.1f, 1.0f,1000.0f);
        j = agent_->joints_list_[4];
        j->SetJointMotorControlPosition(pos_2_, 0.1f, 1.0f,1000.0f);
        j = agent_->joints_list_[5];
        j->SetJointMotorControlPosition(pos_3_, 0.1f, 1.0f,1000.0f);
        j = agent_->joints_list_[6];
        j->SetJointMotorControlPosition(pos_4_, 0.1f, 1.0f,1000.0f);
        j = agent_->joints_list_[7];
        j->SetJointMotorControlPosition(pos_5_, 0.1f, 1.0f,1000.0f);
        j = agent_->joints_list_[8];
        j->SetJointMotorControlPosition(pos_6_, 0.1f, 1.0f,1000.0f);
        j = agent_->joints_list_[10];
        j->SetJointMotorControlPosition(pos_7_, 0.1f, 1.0f,100.0f);
        j = agent_->joints_list_[12];
        j->SetJointMotorControlPosition(pos_7_, 0.1f, 1.0f,100.0f);

        scene_->world_->rotate_camera(c0_, cam_pitch_); 

		std::vector<ObjectAttributes> temp;
        scene_->world_->QueryObjectByLabel("pan", temp);

        for(int i = 0; i < temp.size(); ++i)
        {
        	glm::vec3 aabb_min = temp[i].aabb_min - glm::vec3(0.02);
        	glm::vec3 aabb_max = temp[i].aabb_max + glm::vec3(0.02);

        	glm::vec3 gripper_aabb_min, gripper_aabb_max;
        	agent_->other_parts_[10]->GetAABB(gripper_aabb_min, gripper_aabb_max);
        	if(aabb_min.x < gripper_aabb_max.x && aabb_max.x > gripper_aabb_min.x &&
        	   aabb_min.y < gripper_aabb_max.y && aabb_max.y > gripper_aabb_min.y &&
        	   aabb_min.z < gripper_aabb_max.z && aabb_max.z > gripper_aabb_min.z) {
        		return "idle";
        	}

        	agent_->other_parts_[11]->GetAABB(gripper_aabb_min, gripper_aabb_max);
        	if(aabb_min.x < gripper_aabb_max.x && aabb_max.x > gripper_aabb_min.x &&
        	   aabb_min.y < gripper_aabb_max.y && aabb_max.y > gripper_aabb_min.y &&
        	   aabb_min.z < gripper_aabb_max.z && aabb_max.z > gripper_aabb_min.z) {
        		return "idle";
        	}
        }

        printf("count: %d\n", iterations_);
        if(iterations_++ > 12000) {
        	printf("--------Fail!---------\n");
        	return "idle";
        }

        scene_->world_->BulletStep();   
        renderer_->StepRender(scene_->world_);
        ctx_->SwapBuffer();
        ctx_->PollEvent();

        return "NavTarget";
	}
}