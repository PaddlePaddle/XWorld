#include "python_binding.h"

Thing::Thing() : label_("Nothing"),
				 position_(vec2tuple(glm::vec3(0,0,0))) {}

boost::python::tuple Thing::GetPosition()
{
	Sync();
	return position_;
}

boost::python::tuple Thing::GetOrientation()
{
    Sync();
    return orientation_;
}

std::string Thing::GetLabel()
{
	Sync();
	return label_;
}

void Thing::Sync()
{
	if(auto robot_sptr = robot_.lock()) {
		btTransform tr_bt = robot_sptr->robot_data_.root_part_->object_position_;
		btVector3 pos_bt = tr_bt.getOrigin();
        btQuaternion orn_bt = tr_bt.getRotation();
		position_ = vec2tuple(glm::vec3(pos_bt[0], pos_bt[1], pos_bt[2]));
        orientation_ = vec2tuple(glm::vec4(orn_bt[0], orn_bt[1], orn_bt[2], orn_bt[3]));
		label_ = robot_sptr->robot_data_.label_;
	}
}

Playground::Playground(const int w, const int h, const int headless, const int quality)
{
	assert(quality > -1 && quality < 4);
	
	render_engine::RenderSettings render_profile(quality);

	renderer_ = std::make_shared<render_engine::Render>(w, h, 1, render_profile, headless);
	scene_ = std::make_shared<Map>();
	ctx_ = renderer_->ctx_;
	main_camera_ = nullptr;
	camera_aspect_ = (float) w / h;
	camera_pitch_ = 0.0f;
    w_ = w;
    h_ = h;
    interact_ = false;
    gameover_ = false;
}

Playground::~Playground() {}

void Playground::SetLighting()
{
	renderer_->sunlight_.ambient = glm::vec3(0.02,0.02,0.02);
    renderer_->lighting_.exposure = 0.3f;
    renderer_->lighting_.indirect_strength = 0.25f;
    renderer_->lighting_.traceshadow_distance = 0.3f;
    renderer_->lighting_.propagation_distance = 0.3f;
    renderer_->lighting_.sample_factor = 0.7f;
    renderer_->lighting_.boost_ambient = 0.01f;
    renderer_->lighting_.shadow_bias_scale = 0.0003f;
    renderer_->lighting_.linear_voxelize = true;
}

void Playground::EnableLidar(const int num_rays, const float max_distance)
{
	if(!lidar_) {
		lidar_ = std::make_shared<Lidar>(scene_->world_.get(), 
										 num_rays,
										 max_distance);
		renderer_->InitDrawBatchRay(180);
	}
}

boost::python::list Playground::UpdateLidar(const boost::python::list front_py, 
							    		    const boost::python::list up_py,
							    		    const boost::python::list position_py)
{

	boost::python::list distance_result;

	glm::vec3 front    = list2vec3(front_py);
	glm::vec3 up       = list2vec3(up_py);
	glm::vec3 position = list2vec3(position_py);

	if(lidar_)
	{
		std::vector<RayTestInfo> batch_raycast_result;

        lidar_->Update(front, up, position);
        batch_raycast_result = lidar_->GetResult();

        for (int i = 0; i < batch_raycast_result.size(); ++i)
        {
            if(batch_raycast_result[i].bullet_id < 0)
            {
                renderer_->UpdateRay(i, glm::vec3(0), glm::vec3(0));
                distance_result.append(-1.0f);
            } else {
                renderer_->UpdateRay(i, position,
                    batch_raycast_result[i].pos);
                distance_result.append(glm::distance(position,
                	batch_raycast_result[i].pos));
            }
        }
	}

	return distance_result;
}


void Playground::EnableInventory(const int max_capacity)
{
	if(!inventory_)
		inventory_ = std::make_shared<Inventory>(max_capacity);
}

void Playground::EnableCrowds()
{
	if(!crowd_)
		crowd_ = std::make_shared<Navigation>(ctx_, scene_->world_.get());
}

void Playground::Clear()
{
	scene_->ResetMap();
	scene_->ClearRules();

	if(inventory_)
		inventory_->ResetNonPickableObjectTag();
	
	iterations_ = 0;
}

void Playground::CreateAnTestScene()
{
	static std::string door0 = "./door0/door.urdf";
	static std::string wall1 = "/home/ziyuli/model/wall1/floor.urdf";
	static std::string floor1 = "/home/ziyuli/model/floor/floor.urdf";

	scene_->CreateSectionType(floor1, wall1, door0);
	scene_->GenerateTestFloorPlan(5, 5);

	if(inventory_) {
	    inventory_->AddNonPickableObjectTag("Wall");
	    inventory_->AddNonPickableObjectTag("Floor");
	    inventory_->AddNonPickableObjectTag("Ceiling");
	}
}

Thing Playground::SpawnAnObject(const std::string& file, 
							    const boost::python::list position_py,
				    			const boost::python::list orentation_py,
							    const float scale,
							    const std::string& label,
							    const bool fixed)
{

	std::weak_ptr<RobotBase> obj_wptr;

	glm::vec3 position   = list2vec3(position_py);
	glm::vec4 orentation = list2vec4(orentation_py);

	obj_wptr = scene_->world_->LoadRobot(
        file,
        btVector3(position.x, position.y, position.z),
        btQuaternion(btVector3(orentation.x,orentation.y,orentation.z),orentation.w),
        btVector3(scale, scale, scale),
        label,
        fixed
    );

	if(auto obj_sptr = obj_wptr.lock()) {
		if(!fixed) {
   			obj_sptr->move(true);
   		}
		obj_sptr->DisableSleeping();
	}

	scene_->CreateLabel(file, label);

	Thing object_temp;
	object_temp.SetPtr(obj_wptr);

	return object_temp;
}

void Playground::AttachCameraTo(Thing object, const boost::python::list offset_py)
{

	glm::vec3 offset = list2vec3(offset_py);

	main_camera_ = scene_->world_->add_camera(vec3(0, 0, 0),
    	offset, camera_aspect_);

	if(auto obj_sptr = object.GetPtr().lock()) {
		scene_->world_->attach_camera(main_camera_, obj_sptr.get());
	}

    agent_ = object;
}

void Playground::Initialize()
{
	renderer_->Init(main_camera_);
    scene_->world_->BulletStep();

	if(auto obj_sptr = agent_.GetPtr().lock()) {
		obj_sptr->UnFreeze();
	}
}

boost::python::dict Playground::GetObservationSpace()
{
    boost::python::dict dictionary;

    int img_size = w_ * h_;
    int lidar_rays = lidar_->GetNumRays();
    float lidar_dist = lidar_->GetMaxDistance();

    dictionary["rgb"]    = boost::python::make_tuple(img_size * 3, 0, 255);
    dictionary["d"]      = boost::python::make_tuple(img_size, 0, 255);
    dictionary["lidar"]  = boost::python::make_tuple(lidar_rays, 0, lidar_dist);
    dictionary["pos"]    = boost::python::make_tuple(3, -100, 100);
    dictionary["orn"]    = boost::python::make_tuple(3, -3.14, 3.14);
    dictionary["vel"]    = boost::python::make_tuple(3, -100, 100);
    dictionary["avel"]   = boost::python::make_tuple(3, -100, 100);
    dictionary["acc"]    = boost::python::make_tuple(3, -100, 100);
    dictionary["aacc"]   = boost::python::make_tuple(3, -100, 100);

    return dictionary;
}

boost::python::dict Playground::GetActionSpace()
{
    boost::python::dict dictionary;

    boost::python::list all_actions;

    all_actions.append("move_forward");
    all_actions.append("move_backward");
    all_actions.append("turn_left");
    all_actions.append("turn_right");
    all_actions.append("look_up");
    all_actions.append("look_down");
    all_actions.append("attach");
    all_actions.append("detach");
    all_actions.append("attach");
    all_actions.append("pickup");
    all_actions.append("putdown");
    all_actions.append("rotate");
    all_actions.append("interact");

    dictionary["discrete"] = all_actions;

    return dictionary;
}

boost::python::dict Playground::UpdateSimulationWithAction(const int action)
{
    assert(action < 14 && action > -1);

    boost::python::list actions;

    if(!interact_) {
        if(action == 0) {
            MoveForward(25);
        } 
        else if(action == 1) {
            MoveBackward(25);
        }
        else if(action == 2) {
            TurnLeft(25);
        }
        else if(action == 3) {
            TurnRight(25);
        }
        else if(action == 4) {
            LookUp();
        }
        else if(action == 5) {
            LookDown();
        }
        else if(action == 6) {
            Attach();
        }
        else if(action == 7) {
            Detach();
        }
        else if(action == 8) {
            Grasp();
        }
        else if(action == 9) {
            PutDown();
        }
        else if(action == 10) {
            boost::python::list rotate_angle;
            rotate_angle.append(0);
            rotate_angle.append(1.57);
            rotate_angle.append(0);
            Rotate(rotate_angle);
        }
    } else {
        if(action == 11) {
            actions = EnableInteraction();
        }
        else if(action == 12) {
            DisableInteraction();
        }
        else if(action == 13) {
            //Do Nothing...
        }
    }


    scene_->world_->BulletStep();

    boost::python::dict ret;
    ret["reward"] = -0.1f;
    ret["actions"] = actions;

    // reward, possible interactions
    return ret;
}

boost::python::dict Playground::UpdateSimulation()
{
	scene_->world_->BulletStep();

    boost::python::dict ret;
    ret["reward"] = -0.1f;
    ret["actions"] = boost::python::list();

    // reward
    return ret;
}

void Playground::UpdateRenderer()
{
	renderer_->StepRender(scene_->world_.get());
	ctx_->SwapBuffer();
    ctx_->PollEvent();
}

void Playground::Update()
{
	UpdateSimulation();
	UpdateRenderer();
}

float Playground::GetNearClippingDistance()
{
    return main_camera_->Near;
}

float Playground::GetFarClippingDistance()
{
    return main_camera_->Far;
}

boost::python::object Playground::GetCameraRGBDRaw()
{
    unsigned char * raw_image = renderer_->GetRenderedImages()[0].data.data();

    size_t destination_size = sizeof(unsigned char) * w_ * h_ * 4;

    PyObject* py_buf = PyBuffer_FromReadWriteMemory(raw_image, destination_size);
    boost::python::object ret_val= boost::python::object(boost::python::handle<>(py_buf));
    return ret_val;
}

boost::python::object Playground::GetCameraRGBRaw()
{
    unsigned char * raw_image = renderer_->GetRenderedImages()[0].data.data();
    unsigned char raw_rgb[w_ * h_ * 3];

    size_t destination_size = sizeof(unsigned char) * w_ * h_ * 3;

    memcpy(raw_rgb, raw_image, destination_size);

    PyObject* py_buf = PyBuffer_FromReadWriteMemory(raw_image, destination_size);
    boost::python::object ret_val= boost::python::object(boost::python::handle<>(py_buf));
    return ret_val;
}

boost::python::object Playground::GetCameraDepthRaw()
{
    unsigned char * raw_image = renderer_->GetRenderedImages()[0].data.data();
    unsigned char raw_depth[w_ * h_];

    size_t offset_size      = sizeof(unsigned char) * w_ * h_ * 3;
    size_t destination_size = sizeof(unsigned char) * w_ * h_;

    memcpy(raw_depth, raw_image + offset_size, destination_size);

    PyObject* py_buf = PyBuffer_FromReadWriteMemory(raw_depth, destination_size);
    boost::python::object ret_val= boost::python::object(boost::python::handle<>(py_buf));
    return ret_val;
}

void Playground::MoveForward(const float speed)
{
	if(auto object_sptr = agent_.GetPtr().lock()) 
	{
        object_sptr->MoveForward(speed);
    }
}

void Playground::MoveBackward(const float speed)
{
	if(auto object_sptr = agent_.GetPtr().lock()) 
	{
        object_sptr->MoveBackward(speed);
    }
}

void Playground::TurnLeft(const float speed)
{
	if(auto object_sptr = agent_.GetPtr().lock()) 
	{
        object_sptr->TurnLeft(speed);
    }
}

void Playground::TurnRight(const float speed)
{
	if(auto object_sptr = agent_.GetPtr().lock()) 
	{
        object_sptr->TurnRight(speed);
    }
}

void Playground::LookUp()
{
	camera_pitch_ += 0.5f;
	camera_pitch_ = glm::clamp(camera_pitch_, -45.0f, 45.0f);
    scene_->world_->rotate_camera(main_camera_, camera_pitch_);
}

void Playground::LookDown()
{
	camera_pitch_ -= 0.5f;
	camera_pitch_ = glm::clamp(camera_pitch_, -45.0f, 45.0f);
    scene_->world_->rotate_camera(main_camera_, camera_pitch_);
}

boost::python::list Playground::EnableInteraction()
{
    glm::vec3 from = main_camera_->Position;
    glm::vec3 to = main_camera_->Front * 3.0f + from;
    
    auto bullet_world = scene_->world_;
    std::vector<RayTestInfo> temp_res;
    std::vector<Ray> temp_ray;
    temp_ray.push_back({from, to});

    bullet_world->BatchRayTest(temp_ray, temp_res);

    boost::python::list ret;

    if(temp_res[0].bullet_id > 0) {

        auto object = bullet_world->bullet_handle_to_robot_map_[temp_res[0].bullet_id];

        std::vector<std::string> actions = object->GetActions();

        for (int i = 0; i < actions.size(); ++i)
            ret.append(actions[i]);

        interact_ = true;
    }

    return ret;
}

void Playground::DisableInteraction()
{
    interact_ = false;
}

void Playground::Attach()
{
    glm::vec3 from = main_camera_->Position;
    glm::vec3 to = main_camera_->Front * 3.0f + from;
    
    auto bullet_world = scene_->world_;
    std::vector<RayTestInfo> temp_res;
    std::vector<Ray> temp_ray;
    temp_ray.push_back({from, to});

    bullet_world->BatchRayTest(temp_ray, temp_res);

    if(temp_res[0].bullet_id > 0) {

        auto object = bullet_world->bullet_handle_to_robot_map_[temp_res[0].bullet_id];

        if(auto agent = agent_.GetPtr().lock()) {
            agent->AttachObject(object);
        }
    }
}

void Playground::Detach()
{
    if(auto agent = agent_.GetPtr().lock()) {
        agent->DetachObject();
    }
}

void Playground::Grasp()
{
	if(!inventory_)
		return;

	glm::vec3 from = main_camera_->Position;
	glm::vec3 to = main_camera_->Front * 3.0f + from;

    if(auto agent_sptr = agent_.GetPtr().lock())
    {
        agent_sptr->PickUp(inventory_, from, to);
    }
}

void Playground::PutDown()
{
	if(!inventory_)
		return;

	glm::vec3 from = main_camera_->Position;
	glm::vec3 to = main_camera_->Front * 3.0f + from;

    if(auto agent_sptr = agent_.GetPtr().lock())
    {
        agent_sptr->PutDown(inventory_, from, to);
    }
}

void Playground::Rotate(const boost::python::list angle_py)
{

	glm::vec3 angle = list2vec3(angle_py);

	glm::vec3 from = main_camera_->Position;
	glm::vec3 to = main_camera_->Front * 3.0f + from;
	
	if(auto agent_sptr = agent_.GetPtr().lock())
    {
        agent_sptr->RotateObject(angle, from, to);
    }
}

void Playground::TakeAction(const int action_id)
{
	glm::vec3 from = main_camera_->Position;
	glm::vec3 to = main_camera_->Front * 3.0f + from;
	
	auto bullet_world = scene_->world_;
    std::vector<RayTestInfo> temp_res;
    std::vector<Ray> temp_ray;
    temp_ray.push_back({from, to});

    bullet_world->BatchRayTest(temp_ray, temp_res);

    if(temp_res[0].bullet_id > 0) {
    	auto object = bullet_world->bullet_handle_to_robot_map_[temp_res[0].bullet_id];

    	object->TakeAction(action_id);
    }
}

bool Playground::QueryObjectWithLabelAtCameraCenter(const std::string& label)
{
	std::vector<ObjectAttributes> temp;
    scene_->world_->QueryObjectByLabel(label, temp);

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

    		if(res == temp[i].bullet_id) {
    			return true;
    			break;
    		}
    	}
    }

    return false;
}


boost::python::list Playground::QueryObjectByLabel(const std::string& label)
{
	boost::python::list result;

	std::vector<ObjectAttributes> temp;
    scene_->world_->QueryObjectByLabel(label, temp);

    for(int i = 0; i < temp.size(); ++i)
    {
    	Range r;
    	r.min = temp[i].aabb_min;
    	r.max = temp[i].aabb_max;
    	result.append(r);
    }

    return result;
}

Thing Playground::QueryObjectAtCameraCenter()
{
	glm::vec3 from = main_camera_->Position;
	glm::vec3 to = main_camera_->Front * 3.0f + from;
	
	auto bullet_world = scene_->world_;
    std::vector<RayTestInfo> temp_res;
    std::vector<Ray> temp_ray;
    temp_ray.push_back({from, to});

    bullet_world->BatchRayTest(temp_ray, temp_res);

    Thing object_temp;
    if(temp_res[0].bullet_id > 0) {
    	auto object = bullet_world->bullet_handle_to_robot_map_[temp_res[0].bullet_id];
    	object_temp.SetPtr(object);
    }

	return object_temp;
}

bool Playground::QueryObjectAABBIntersect(Thing& object_a, Thing& object_b)
{
	if(auto object_a_sptr = object_a.GetPtr().lock())
	{
		if(auto object_b_sptr = object_b.GetPtr().lock())
		{
			glm::vec3 a_min, a_max, b_min, b_max;

            object_a_sptr->robot_data_.root_part_->GetAABB(a_min, a_max);
            object_b_sptr->robot_data_.root_part_->GetAABB(b_min, b_max);

            if(a_min.x < b_min.x && a_max.x > b_max.x &&
               a_min.z < b_min.z && a_max.z > b_max.z) {
            	return true;
            }
		}
	}
	return false;
}

bool Playground::QueryObjectWithLabelAtForward(const std::string& label)
{
	std::vector<ObjectAttributes> temp;
    scene_->world_->QueryObjectByLabel(label, temp);

    for(int i = 0; i < temp.size(); ++i)
    {
    	glm::vec3 aabb_min = temp[i].aabb_min;
    	glm::vec3 aabb_max = temp[i].aabb_max;
    	glm::vec3 aabb_center = (aabb_min + aabb_max) * 0.5f;
    	glm::vec3 camera_center = main_camera_->Position;
    	glm::vec3 camera_front  = main_camera_->Front;

    	if(glm::distance(camera_center, aabb_center) < 3.0f) 
    	{
            float ang = glm::dot(glm::normalize(aabb_center - camera_center), camera_front);
    		if(ang > 0.9f)
    			return true;
    	}
    }
    return false;
}


bool Playground::QueryObjectWithLabelNearMe(const std::string& label)
{
	std::vector<ObjectAttributes> temp;
    scene_->world_->QueryObjectByLabel(label, temp);

    for(int i = 0; i < temp.size(); ++i)
    {
    	glm::vec3 aabb_min = temp[i].aabb_min;
    	glm::vec3 aabb_max = temp[i].aabb_max;
    	glm::vec3 aabb_center = (aabb_min + aabb_max) * 0.5f;
    	glm::vec3 camera_center = main_camera_->Position;
    	glm::vec3 camera_front  = main_camera_->Front;

    	if(glm::distance(camera_center, aabb_center) < 3.0f) 
    		return true;
    }
    return false;
}

void Playground::Teleport(Thing object, const boost::python::list position_py)
{
	glm::vec3 position = list2vec3(position_py);

	if(auto object_sptr = object.GetPtr().lock())
	{
		auto root_part_ = object_sptr->robot_data_.root_part_;

		btTransform root_transform = root_part_->object_position_;
		root_transform.setOrigin(btVector3(position.x, position.y, position.z));
		scene_->world_->SetTransformation(object_sptr, root_transform);
	}
}