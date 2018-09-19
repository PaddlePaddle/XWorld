#include "task_example_testing.h"

static std::string data_dir = "/home/ziyuli/Desktop/suncg";
static std::string metadata_models = "/home/ziyuli/Desktop/suncg/metadata/ModelCategoryMapping.csv";
static std::string house1 = "/home/ziyuli/Desktop/suncg/house/0911a85dffc1286db3a7aa1c9e4e2476/house.json";
static std::string crate03 = "./crate_0.3/crate.urdf";

namespace xrobot
{
	Task_GetAndPut::Task_GetAndPut(render_engine::Render * renderer,
			MapSuncg * map) : iterations_(0),
						     scene_(map),
						     agent_(nullptr),
						     renderer_(renderer),
						     ctx_(renderer->ctx_),
						     main_camera_(nullptr),
						     cam_pitch_(0),
						     inventory_(new Inventory(map->world_, 10)) {}

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
		renderer_->sunlight_.direction = glm::normalize(glm::vec3(0.3, 2, 1));
        renderer_->lighting_.exposure = 1.5f;
        renderer_->lighting_.indirect_strength = 1.0f;

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
	    scene_->AddPhysicalProperties("sofa", {1000, false});
	    scene_->AddPhysicalProperties("cup", {50, false});
	    scene_->LoadJSON(house1.c_str(), data_dir.c_str(), true);
	    scene_->SetMapSize(-8, -8, 6, 6);

	   	// Load Agent
	    agent_ = scene_->world_->LoadURDF(
	        "husky/husky.urdf",
	        btVector3(-1,0.06,-1),
	        btQuaternion(btVector3(-1,0,0),1.57),
	        0.6f,
	        "agent",
	        true
	    );

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

	    if(ctx_->GetKeyPress1() && !put_box_) {
	    	glm::vec3 fromPosition = main_camera_->Position;
	    	glm::vec3 toPosition = main_camera_->Front * 3.0f + fromPosition;

    		std::vector<RayTestInfo> temp_res;
    		std::vector<Ray> temp_ray;
    		temp_ray.push_back({fromPosition, toPosition});

    		scene_->world_->BatchRayTest(temp_ray, temp_res);

    		if(temp_res[0].bullet_id > 0) {

    			std::string label = scene_->map_bullet_label_[temp_res[0].bullet_id];
    			if(label != "Wall" && label != "Ceiling" && label != "Floor") {
    				Robot * r = scene_->world_->bullet_handle_to_robot_map_[temp_res[0].bullet_id];
    				inventory_->PutObject(r);
    				scene_->world_->RemoveRobot2(r);
    				scene_->map_bullet_label_.erase(scene_->map_bullet_label_.find(temp_res[0].bullet_id));
    				put_box_ = true;
    			}
    		}
	    }

	    static float angle = 0;

	    // Rotate Object
	    if(ctx_->GetKeyPress2() && !put_box_) {

	    	glm::vec3 fromPosition = main_camera_->Position;
    		glm::vec3 toPosition = main_camera_->Front * 3.0f + fromPosition;

    		std::vector<RayTestInfo> temp_res;
    		std::vector<Ray> temp_ray;
    		temp_ray.push_back({fromPosition, toPosition});

    		scene_->world_->BatchRayTest(temp_ray, temp_res);

    		if(temp_res[0].bullet_id > 0) {
    			Robot * temp_obj = scene_->world_->bullet_handle_to_robot_map_[temp_res[0].bullet_id];

    			if(temp_obj->label_!= "Wall"&& temp_obj->label_!= "Floor") {

	    			btTransform tr = temp_obj->root_part_->object_position_;
	    			tr.setRotation(btQuaternion(btVector3(0,1,0), angle+=1.57 / 2));
	    			scene_->world_->SetTransformation(temp_obj, tr);
	    			scene_->world_->BulletStep();

					glm::vec3 aabb_min0, aabb_max0;
					temp_obj->root_part_->GetAABB(aabb_min0, aabb_max0);

					aabb_min0 += glm::vec3(0.1f);
				    aabb_max0 -= glm::vec3(0.1f);

					bool intersect = false;

	    			for (size_t i = 0; i < scene_->world_->size(); i++) {
						Robot* body = scene_->world_->robot_list_[i];
						Object * part = body->root_part_;
						if (part && !body->recycle() && part->id()!=temp_obj->bullet_handle_
							&& body->label_!= "Wall"
							&& body->label_!= "Floor")
				        {
				            glm::vec3 aabb_min1, aabb_max1;
				            part->GetAABB(aabb_min1, aabb_max1);

				            if(aabb_min1.x < aabb_max0.x &&
				               aabb_max1.x > aabb_min0.x &&
				               aabb_min1.z < aabb_max0.z &&
				               aabb_max1.z > aabb_min0.z) {
				            	intersect = true;
				            	printf("label: %s\n", body->label_.c_str());
				            	break;
				            }
				        }
					}

					if(intersect) {
						tr.setRotation(btQuaternion(btVector3(0,1,0), angle-=1.57 / 2));
		    			scene_->world_->SetTransformation(temp_obj, tr);
		    			scene_->world_->BulletStep();
						printf("intersect with object\n");
					} else {
	    				put_box_ = true;
	    			}
	    		}
    		}
	    }

	    // Put a Box
	    if(ctx_->GetKeyPressSpace() && !put_box_) {

    	    glm::vec3 fromPosition = main_camera_->Position;
    		glm::vec3 toPosition = main_camera_->Front * 3.0f + fromPosition;

    		std::vector<RayTestInfo> temp_res;
    		std::vector<Ray> temp_ray;
    		temp_ray.push_back({fromPosition, toPosition});

    		scene_->world_->BatchRayTest(temp_ray, temp_res);

    		if(temp_res[0].bullet_id > 0) {
    			Robot * obj = scene_->world_->bullet_handle_to_robot_map_[temp_res[0].bullet_id];
    			glm::vec3 normal = temp_res[0].norm;
    			glm::vec3 position = temp_res[0].pos;

    			if(obj && glm::dot(normal, glm::vec3(0,1,0)) > 0.9f) {
    				glm::vec3 aabb_min, aabb_max;
    				obj->root_part_->GetAABB(aabb_min, aabb_max);

    				if(aabb_max.y - 0.05f < position.y || true) {

    					std::string temp_obj_label; 
    					std::string temp_obj_path = inventory_->GetObjectRandomly(temp_obj_label);

    					if(!temp_obj_path.empty()) {

		    				Robot * temp_obj = scene_->world_->LoadOBJ(
						        temp_obj_path,
						        btVector3(position.x,20,position.z),
						        btQuaternion(btVector3(-1,0,0),1.57),
						        btVector3(1,1,1),
						        temp_obj_label,
						        0,
						        false,
						        true
						    );
		    				temp_obj->Sleep();

		    				glm::vec3 aabb_min0, aabb_max0;
		    				temp_obj->root_part_->GetAABB(aabb_min0, aabb_max0);

		    				float width = (aabb_max0.x - aabb_min0.x) / 2;
		    				float height = (aabb_max0.z - aabb_min0.z) / 2;

		    				if(aabb_min.x + width < position.x && 
		    				   aabb_max.x - width > position.x &&
		    				   aabb_min.z + height < position.z && 
		    				   aabb_max.z - height > position.z) {

		    					bool intersect = false;

		    					for (size_t i = 0; i < scene_->world_->size(); i++) {
		    						Robot* body = scene_->world_->robot_list_[i];
        							Object * part = body->root_part_;
        							if (part && !body->recycle() && part->id()!=temp_obj->bullet_handle_
        								&& body->label_!= "Wall"
        								&& body->label_!= "Floor")
							        {
							            glm::vec3 aabb_min1, aabb_max1;
							            part->GetAABB(aabb_min1, aabb_max1);

							            if(aabb_min1.x < aabb_max0.x &&
							               aabb_max1.x > aabb_min0.x &&
							               aabb_min1.z < aabb_max0.z &&
							               aabb_max1.z > aabb_min0.z) {
							            	intersect = true;
							            	printf("label: %s\n", body->label_.c_str());
							            	break;
							            }
							        }
		    					}

		    					if(intersect) {
		    						inventory_->PutObject(temp_obj);
		    						scene_->world_->RemoveRobot2(temp_obj);
		    						printf("intersect with object\n");
		    					} else {
				    				float height = (aabb_max0.y - aabb_min0.y) * 0.5f;
				    				glm::vec3 intersection = temp_res[0].pos;

				    				btTransform tr;
				    				tr.setIdentity();
				    				tr.setOrigin(btVector3(intersection.x, intersection.y, intersection.z));

				    				scene_->world_->SetTransformation(temp_obj, tr);
				    				put_box_ = true;
				    				printf("success!\n");
				    			}

			    			} else {
			    				inventory_->PutObject(temp_obj);
			    				scene_->world_->RemoveRobot2(temp_obj);
			    				printf("intersect with border\n");
			    			}
			    		}
    				}
    			}
    		}
	    }

		// if(ctx_->GetKeyPressUp()) {
		// 	vel_front_left_wheel_  += speed_;
  //           vel_front_right_wheel_ += speed_;
  //           vel_rear_left_wheel_   += speed_;
  //           vel_rear_right_wheel_  += speed_;
  //           agent_->Wake();
		// }

  //       if(ctx_->GetKeyPressDown()) {
		// 	vel_front_left_wheel_  -= speed_;
  //           vel_front_right_wheel_ -= speed_;
  //           vel_rear_left_wheel_   -= speed_;
  //           vel_rear_right_wheel_  -= speed_;
  //           agent_->Wake();
  //       }

  //       if(ctx_->GetKeyPressLeft()) {
		// 	vel_front_left_wheel_  -= speed_;
  //           vel_front_right_wheel_ += speed_;
  //           vel_rear_left_wheel_   -= speed_;
  //           vel_rear_right_wheel_  += speed_;
  //           agent_->Wake();
  //       }

  //       if(ctx_->GetKeyPressRight()) {
		// 	vel_front_left_wheel_  += speed_;
  //           vel_front_right_wheel_ -= speed_;
  //           vel_rear_left_wheel_   += speed_;
  //           vel_rear_right_wheel_  -= speed_;
  //           agent_->Wake();
  //       }

        if(ctx_->GetKeyPressKP9())
            cam_pitch_ += 0.1f;

        if(ctx_->GetKeyPressKP6())
            cam_pitch_ -= 0.1f;

        cam_pitch_ = glm::clamp(cam_pitch_, -45.0f, 45.0f);
        scene_->world_->rotate_camera(main_camera_, cam_pitch_);

        // Joint *j;
        // j = agent_->joints_list_[2];
        // j->SetJointMotorControlVelocity(vel_front_left_wheel_, 1, 100.0);
        // j = agent_->joints_list_[3];
        // j->SetJointMotorControlVelocity(vel_front_right_wheel_, 1, 100.0);
        // j = agent_->joints_list_[4];
        // j->SetJointMotorControlVelocity(vel_rear_left_wheel_, 1, 100.0);
        // j = agent_->joints_list_[5];
        // j->SetJointMotorControlVelocity(vel_rear_right_wheel_, 1, 100.0);

        // THIS IS ONLY FOR TESTING! 
        // Force to Switch a New Scene
        // if(ctx_->GetKeyPressSpace()) {
        //     ctx_->PollEvent();
        //     return "idle";
        // }

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