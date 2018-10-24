#include "python_binding.h"

NavAgent::NavAgent(const int uid, const std::string& label) 
    : label_(label), uid_(uid) {}

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

Playground::Playground(const int w, const int h,
                       const int headless, const int quality, const int device)
{
	assert(quality > -1 && quality < 2);
	
	render_engine::RenderSettings render_profile(quality);

	renderer_ = std::make_shared<render_engine::Render>(w, h, 1, 
                                                        render_profile, 
                                                        headless,
                                                        device);

	ctx_ = renderer_->ctx_;
	main_camera_ = nullptr;
	camera_aspect_ = (float) w / h;
	camera_pitch_ = 0.0f;
    w_ = w;
    h_ = h;
    interact_ = false;
    gameover_ = false;
    kill_after_arrived_ = true;
    scene_ = nullptr;
    crowd_ = nullptr;
    inventory_ = nullptr;
    lidar_ = nullptr;
}

Playground::~Playground() {}

boost::python::dict Playground::GetStatus() const
{
    boost::python::dict dictionary;

    int rendered_frames = (int) renderer_->num_frames_;
    int framerate = (int) renderer_->current_framerate_;
    int model_cache_size = scene_->world_->model_cache_.size();

    dictionary["frames"] = rendered_frames;
    dictionary["framerate"] = framerate;
    dictionary["cachesize"] = model_cache_size;

    return dictionary;
}

void Playground::SetLighting(const boost::python::dict lighting)
{
    if(lighting.has_key("direction_x"))
    {
        boost::python::extract<float> val(lighting["direction_x"]);
        renderer_->sunlight_.direction.x = val;
    }

    if(lighting.has_key("direction_y"))
    {
        boost::python::extract<float> val(lighting["direction_y"]);
        renderer_->sunlight_.direction.y = val;
    }

    if(lighting.has_key("direction_z"))
    {
        boost::python::extract<float> val(lighting["direction_z"]);
        renderer_->sunlight_.direction.z = val;
    }

    if(lighting.has_key("ambient"))
    {
        boost::python::extract<float> val(lighting["ambient"]);
        renderer_->sunlight_.ambient = glm::vec3(val);
    }

    if(lighting.has_key("ssr"))
    {
        boost::python::extract<float> val(lighting["ssr"]);
        renderer_->lighting_.use_ssr = val > 0;
    }

    if(lighting.has_key("exposure"))
    {
        boost::python::extract<float> val(lighting["exposure"]);
        renderer_->lighting_.exposure = val;
    }

    if(lighting.has_key("indirect_strength"))
    {
        boost::python::extract<float> val(lighting["indirect_strength"]);
        renderer_->lighting_.indirect_strength = val;
    }

    if(lighting.has_key("traceshadow_distance"))
    {
        boost::python::extract<float> val(lighting["traceshadow_distance"]);
        renderer_->lighting_.traceshadow_distance = val;
    }

    if(lighting.has_key("propagation_distance"))
    {
        boost::python::extract<float> val(lighting["propagation_distance"]);
        renderer_->lighting_.propagation_distance = val;
    }

    if(lighting.has_key("conetracing_distance"))
    {
        boost::python::extract<float> val(lighting["conetracing_distance"]);
        renderer_->lighting_.conetracing_distance = val;
    }

    if(lighting.has_key("sample_factor"))
    {
        boost::python::extract<float> val(lighting["sample_factor"]);
        renderer_->lighting_.sample_factor = val;
    }

    if(lighting.has_key("ao_falloff"))
    {
        boost::python::extract<float> val(lighting["ao_falloff"]);
        renderer_->lighting_.ao_falloff = val;
    }

    if(lighting.has_key("force_disable_propagation"))
    {
        boost::python::extract<float> val(lighting["force_disable_propagation"]);
        renderer_->lighting_.force_disable_propagation = val > 0;
    }

    if(lighting.has_key("shadow_bias_scale"))
    {
        boost::python::extract<float> val(lighting["shadow_bias_scale"]);
        renderer_->lighting_.shadow_bias_scale = val;
    }

    if(lighting.has_key("shadow_bias_clamp"))
    {
        boost::python::extract<float> val(lighting["shadow_bias_clamp"]);
        renderer_->lighting_.shadow_bias_clamp = val;
    }

    if(lighting.has_key("force_disable_shadow"))
    {
        boost::python::extract<float> val(lighting["force_disable_shadow"]);
        renderer_->lighting_.force_disable_shadow = val > 0;
    }

    if(lighting.has_key("linear_voxelize"))
    {
        boost::python::extract<float> val(lighting["linear_voxelize"]);
        renderer_->lighting_.linear_voxelize = val > 0;
    }
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

void Playground::EnableNavigation(const boost::python::list min_corner, 
                                  const boost::python::list max_corner,
                                  const bool kill_after_arrived)
{
	glm::vec3 min_c = list2vec3(min_corner);
    glm::vec3 max_c = list2vec3(max_corner);

    if(!crowd_) 
		crowd_ = std::make_shared<Navigation>(ctx_, scene_->world_.get());

    crowd_->SetBakeArea(min_c, max_c);
    // scene_->world_->BulletStep();
    // crowd_->BakeNavMesh();

    kill_after_arrived_ = kill_after_arrived;
}

void Playground::AssignAgentRadius(const float radius)
{
    if(crowd_)
        crowd_->SetAgentRadius(radius);
}

void Playground::AssignSurfaceLevel(const float level)
{
    if(crowd_)
        crowd_->SetSurfaceLevel(level);
}

void Playground::BakeNavigationMesh()
{
    if(crowd_) {
        scene_->world_->BulletStep();
        crowd_->BakeNavMesh();
    }
}

NavAgent Playground::SpawnNavigationAgent(const std::string& path,
                                          const std::string& label,
                                          const boost::python::list position,
                                          const boost::python::list orientation)
{
    if(crowd_) {
        crowd_->SpawnAgent(list2vec3(position), list2quat(orientation), path, label);
        
        Agent agent_temp = crowd_->crowd_.back();
        NavAgent nav_agent(agent_temp.uid_, label);
        return nav_agent;
    }

    return NavAgent(-1, "Invalid");
}

void Playground::AssignNavigationAgentTarget(const NavAgent& agent,
                                             const boost::python::list position)
{
    if(crowd_) {
        if(agent.GetUid() < 0)
            return;

        for (int i = 0; i < crowd_->crowd_.size(); ++i)
        {
            if(crowd_->crowd_[i].uid_ == agent.GetUid())
            {
                crowd_->crowd_[i].AssignTarget(list2vec3(position));
            }
        }
    }
}

void Playground::Clear()
{
    if(scene_)  
	   scene_->ResetMap();

    if(crowd_)
        crowd_->Reset();

	if(inventory_)
		inventory_->ResetNonPickableObjectTag();
	
	iterations_ = 0;
    camera_pitch_ = 0;
}

void Playground::CreateAnTestScene()
{
	static std::string door = "./door/door.urdf";
	static std::string test_wall = "./wall/floor.urdf";
	static std::string test_floor = "./floor/floor.urdf";

    if(!scene_)
        scene_ = std::make_shared<MapGrid>();

    std::shared_ptr<MapGrid> scene_grid 
        = std::dynamic_pointer_cast<MapGrid>(scene_);

	scene_grid->CreateSectionType(test_floor, test_wall, door);
	scene_grid->GenerateTestFloorPlan(5, 5);

	if(inventory_) {
	    inventory_->AddNonPickableObjectTag("Wall");
	    inventory_->AddNonPickableObjectTag("Floor");
	    inventory_->AddNonPickableObjectTag("Ceiling");
	}

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

void Playground::CreateRandomGenerateScene()
{
    if(!scene_)
        scene_ = std::make_shared<MapGrid>();

    std::shared_ptr<MapGrid> scene_grid 
        = std::dynamic_pointer_cast<MapGrid>(scene_);

    renderer_->sunlight_.ambient = glm::vec3(0.15,0.15,0.15);
    renderer_->lighting_.exposure = 1.5f;
    renderer_->lighting_.indirect_strength = 0.4f;
    renderer_->lighting_.traceshadow_distance = 0.5f;
    renderer_->lighting_.propagation_distance = 0.5f;
    renderer_->lighting_.sample_factor = 0.4f;
    renderer_->lighting_.boost_ambient = 0.02f;
    renderer_->lighting_.shadow_bias_scale = 0.0003f;
    renderer_->lighting_.linear_voxelize = false;
}

void Playground::LoadRandomSceneConfigure(const boost::python::dict conf)
{
    std::shared_ptr<MapGrid> scene_grid 
        = std::dynamic_pointer_cast<MapGrid>(scene_);

    if(conf.has_key("room"))
    {
        boost::python::list val =
            boost::python::extract<boost::python::list>(conf["room"]);

        assert(boost::python::len(val) % 3 == 0);
        int num_conf_room = boost::python::len(val) / 3;

        for (int i = 0; i < num_conf_room; ++i)
        {
            boost::python::extract<std::string> str_0(val[0 + i * 3]);
            boost::python::extract<std::string> str_1(val[1 + i * 3]);
            boost::python::extract<std::string> str_2(val[2 + i * 3]);
            scene_grid->CreateSectionType(str_0, str_1, str_2);
        }
    }

    if(conf.has_key("on_floor"))
    {
        boost::python::list val =
            boost::python::extract<boost::python::list>(conf["on_floor"]);

        assert(boost::python::len(val) % 2 == 0);
        int num_conf_room = boost::python::len(val) / 2;

        for (int i = 0; i < num_conf_room; ++i)
        {
            boost::python::extract<std::string> str_path (val[0 + i * 2]);
            boost::python::extract<std::string> str_label(val[1 + i * 2]);
            scene_grid->CreateSpawnOnFloor(str_path);
            scene_grid->CreateLabel(str_path, str_label);
        }
    }

    if(conf.has_key("on_object"))
    {
        boost::python::list val =
            boost::python::extract<boost::python::list>(conf["on_object"]);

        assert(boost::python::len(val) % 2 == 0);
        int num_conf_room = boost::python::len(val) / 2;

        for (int i = 0; i < num_conf_room; ++i)
        {
            boost::python::extract<std::string> str_path (val[0 + i * 2]);
            boost::python::extract<std::string> str_label(val[1 + i * 2]);
            scene_grid->CreateSpawnOnObject(str_path);
            scene_grid->CreateLabel(str_path, str_label);
        }
    }

    if(conf.has_key("on_either"))
    {
        boost::python::list val =
            boost::python::extract<boost::python::list>(conf["on_either"]);

        assert(boost::python::len(val) % 2 == 0);
        int num_conf_room = boost::python::len(val) / 2;

        for (int i = 0; i < num_conf_room; ++i)
        {
            boost::python::extract<std::string> str_path (val[0 + i * 2]);
            boost::python::extract<std::string> str_label(val[1 + i * 2]);
            scene_grid->CreateSpawnEither(str_path);
            scene_grid->CreateLabel(str_path, str_label);
        }
    }
}

boost::python::list Playground::LoadRandomScene(const int size, 
                                                const int on_floor,
                                                const int on_object,
                                                const int on_either)
{
    std::shared_ptr<MapGrid> scene_grid 
        = std::dynamic_pointer_cast<MapGrid>(scene_);

    glm::vec3 start = scene_grid->GenerateFloorPlan(size, size);
    scene_grid->Spawn(on_floor, on_object, on_either);

    boost::python::list ret;
    ret.append(start.x);
    ret.append(start.y);
    ret.append(start.z);
    return ret;
}

void Playground::CreateSceneFromSUNCG()
{
    if(!scene_)
        scene_ = std::make_shared<MapSuncg>();

    std::shared_ptr<MapSuncg> scene_suncg 
        = std::dynamic_pointer_cast<MapSuncg>(scene_);

    scene_suncg->SetMapSize(-8, -8, 8, 8);

    renderer_->sunlight_.direction = glm::vec3(0.3, 1, 1);
    renderer_->lighting_.exposure = 1.0f;
    renderer_->lighting_.indirect_strength = 1.5f;
    renderer_->lighting_.traceshadow_distance = 0.3f;
    renderer_->lighting_.propagation_distance = 0.3f;
    renderer_->lighting_.force_disable_shadow = true;
}

void Playground::CreateEmptyScene(const float min_x, const float max_x,
                                  const float min_z, const float max_z)
{
    if(!scene_)
        scene_ = std::make_shared<Map>();

    std::shared_ptr<Map> scene_generic 
        = std::dynamic_pointer_cast<Map>(scene_);

    scene_generic->GenerateGenetricMap();

    scene_generic->world_->set_world_size(min_x, min_z, max_x, max_z);

    renderer_->sunlight_.direction = glm::vec3(0.3, 1, 1);
    renderer_->lighting_.exposure = 1.0f;
    renderer_->lighting_.indirect_strength = 1.5f;
    renderer_->lighting_.traceshadow_distance = 0.3f;
    renderer_->lighting_.propagation_distance = 0.3f;
    renderer_->lighting_.force_disable_shadow = true;
}

void Playground::LoadSUNCG(const std::string& house,
                           const std::string& metadata,
                           const std::string& suncg_data_dir,
                           const int filter)
{
    if(!scene_)
    {
        printf("Use Playground::CreateSceneFromSUNCG first for initializing!\n");
        return;
    }

    std::shared_ptr<MapSuncg> scene_suncg 
        = std::dynamic_pointer_cast<MapSuncg>(scene_);

    if(filter > -1)
        scene_suncg->SetRemoveAll(filter);

    // TODO
    // Assign Props

    scene_suncg->LoadCategoryCSV(metadata.c_str());

    scene_suncg->LoadJSON(house.c_str(), suncg_data_dir.c_str(), true);

    // TODO
    // Assign Map Size

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

	//scene_->CreateLabel(file, label);

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

void Playground::FreeCamera(const boost::python::list position, 
                            const float yaw, const float pitch)
{
    glm::vec3 pos = list2vec3(position);

    main_camera_ = scene_->world_->add_camera(pos,
        vec3(0, 0, 0), camera_aspect_);

    main_camera_->Yaw = yaw;
    main_camera_->Pitch = pitch;
    main_camera_->updateCameraVectors();
}

void Playground::UpdateFreeCamera(const boost::python::list position, 
                                  const float yaw, const float pitch)
{
    glm::vec3 pos = list2vec3(position);

    main_camera_->Position = pos;
    main_camera_->Yaw = yaw;
    main_camera_->Pitch = pitch;
    main_camera_->updateCameraVectors();
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
        else if(action == 11) {
            current_actions_ = EnableInteraction();
        }
    } else {
        if(action == 12) {
            DisableInteraction();
            current_actions_ = boost::python::list();
        }
        else if(action < 10) {
            TakeAction(action);
        }
    }

    if(crowd_) {
        crowd_->Update();

        if(kill_after_arrived_)
            crowd_->KillAgentOnceArrived();
    }

    scene_->world_->BulletStep();

    boost::python::dict ret;
    ret["reward"] = -0.1f;
    ret["actions"] = current_actions_;

    // reward, possible interactions
    return ret;
}

boost::python::dict Playground::UpdateSimulation()
{

    if(crowd_) {
        crowd_->Update();

        if(kill_after_arrived_)
            crowd_->KillAgentOnceArrived();
    }

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

void Playground::ControlJoints(const Thing& object, 
                               const boost::python::dict joint_positions,
                               const float max_force)
{
    if(auto object_sptr = object.GetPtr().lock())
    {
        boost::python::list joints_position_keys = joint_positions.keys();

        for (int i = 0; i < boost::python::len(joints_position_keys); ++i)
        {
            int joint_id   = boost::python::extract<int>(joints_position_keys[i]);
            float position = boost::python::extract<float>(joint_positions[joint_id]);

            auto joint_ptr = object_sptr->robot_data_.joints_list_[joint_id];
            joint_ptr->SetJointMotorControlPosition(position, 0.1f, 1.0f, max_force);
        }
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