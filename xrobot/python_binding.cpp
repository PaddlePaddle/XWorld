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
		btTransform tr_bt = robot_sptr->root_part_->object_position_;
		btVector3 pos_bt = tr_bt.getOrigin();
        btQuaternion orn_bt = tr_bt.getRotation();
		position_ = vec2tuple(glm::vec3(pos_bt[0], pos_bt[1], pos_bt[2]));
        orientation_ = vec2tuple(glm::vec4(orn_bt[0], orn_bt[1], orn_bt[2], orn_bt[3]));
		label_ = robot_sptr->body_data_.label;
	}
}

Playground::Playground(const int w, const int h,
                       const int headless, const int quality, const int device)
{
	assert(quality > -1 && quality < 5);
	
	render_engine::Profile render_profile = render_engine::profiles[quality];

	renderer_ = std::make_shared<render_engine::Render>(w, h, 
                                                        render_profile, 
                                                        headless,
                                                        device);

	ctx_ = renderer_->GetContext();
	main_camera_ = nullptr;
	camera_aspect_ = (float) w / h;
	camera_pitch_ = 0.0f;
    w_ = w;
    h_ = h;
    hold_actions_ = false;
    highlight_objects_ = false;
    inventory_opened_ = false;
    interact_ = false;
    gameover_ = false;
    kill_after_arrived_ = true;
    scene_ = nullptr;
    crowd_ = nullptr;
    inventory_ = nullptr;
    lidar_ = nullptr;
    objects_ = std::vector<Thing>();
}

Playground::~Playground() {}

boost::python::list Playground::GetGoals() const
{
    boost::python::list goal_list;
    return goal_list;
}

void Playground::AssignTag(const std::string& path, const std::string& tag)
{
    scene_->world_->AssignTag(path, tag);
}

void Playground::LoadTag(const std::string& path)
{
    scene_->world_->LoadMetadata(path.c_str());
}

void Playground::MakeObjectPickable(const std::string& tag)
{
    scene_->world_->UpdatePickableList(tag, true);
}

boost::python::dict Playground::GetStatus() const
{
    boost::python::dict dictionary;

    int rendered_frames = 0;
    int framerate = 0;
    int model_cache_size = scene_->world_->model_cache_.size();

    dictionary["frames"] = rendered_frames;
    dictionary["framerate"] = framerate;
    dictionary["cachesize"] = model_cache_size;

    return dictionary;
}

void Playground::HighlightCenter(const bool highlight)
{
    scene_->world_->highlight_center_ = highlight - 1;
    
}

void Playground::DisplayInventory(const bool display)
{
    if(!inventory_) return;

    if(display) {
        scene_->world_->inventory_size_ = 
            inventory_->GetNumUsedSpace() + inventory_->GetNumFreeSpace();
    } else {
        scene_->world_->inventory_size_ = 0;
    }
}

void Playground::SetLighting(const boost::python::dict lighting)
{
    if(lighting.has_key("direction_x") && 
       lighting.has_key("direction_y") &&
       lighting.has_key("direction_z")) {
        float val_x = boost::python::extract<float>(lighting["direction_x"]);
        float val_y = boost::python::extract<float>(lighting["direction_y"]);
        float val_z = boost::python::extract<float>(lighting["direction_z"]);
        renderer_->UpdateLightDirection(glm::vec3(val_x, val_y, val_z));
    }

    if(lighting.has_key("ambient"))
    {
        boost::python::extract<float> val(lighting["ambient"]);
        renderer_->UpdateAmbientColor(glm::vec3(val));
    }

    if(lighting.has_key("exposure"))
    {
        boost::python::extract<float> val(lighting["exposure"]);
        renderer_->UpdateExposure(val);
    }

    if(lighting.has_key("background_r") && 
       lighting.has_key("background_g") &&
       lighting.has_key("background_b")) {
        float val_x = boost::python::extract<float>(lighting["background_r"]);
        float val_y = boost::python::extract<float>(lighting["background_g"]);
        float val_z = boost::python::extract<float>(lighting["background_b"]);
        renderer_->UpdateBackgroundColor(glm::vec3(val_x, val_y, val_z));
    }

    if(lighting.has_key("directional_light_r") && 
       lighting.has_key("directional_light_g") &&
       lighting.has_key("directional_light_b")) {
        float val_x = boost::python::extract<float>(lighting["directional_light_r"]);
        float val_y = boost::python::extract<float>(lighting["directional_light_g"]);
        float val_z = boost::python::extract<float>(lighting["directional_light_b"]);
        renderer_->UpdateLightColor(glm::vec3(val_x, val_y, val_z));
    }
}

void Playground::EnableLidar(const int num_rays, const float max_distance)
{
	if(!lidar_) {
		lidar_ = std::make_shared<Lidar>(scene_->world_.get(), 
										 num_rays,
										 max_distance);
		renderer_->InitDrawBatchRays(num_rays);
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

    front = glm::normalize(front);
    up = glm::normalize(up);

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

void Playground::ClearInventory()
{
    if(inventory_)
        inventory_->ClearInventory();
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

void Playground::ClearSpawnObjectsExceptAgent()
{
    for(auto& object : objects_) {
        if(!object.__eq__(agent_)) {
            RemoveAnObject(object);
        }
    }

    if(crowd_)
        crowd_->Reset();

    if(inventory_)
        inventory_->ClearInventory();
}

void Playground::Clear()
{
    if(scene_)  
	   scene_->ResetMap();

    if(crowd_)
        crowd_->Reset();

	if(inventory_)
		inventory_->ClearInventory();
        //inventory_->ResetPickableObjectTag();
	
    objects_.clear();
	iterations_ = 0;
    camera_pitch_ = 0;
}

void Playground::CreateArena(const int width, const int length)
{
	static std::string door = "./door/door.urdf";
	static std::string test_wall = "./wall/floor.urdf";
	static std::string test_floor = "./floor/floor.urdf";

    if(!scene_)
        scene_ = std::make_shared<MapGrid>();

    scene_->world_->inventory_size_ = 0;

    std::shared_ptr<MapGrid> scene_grid 
        = std::dynamic_pointer_cast<MapGrid>(scene_);

    scene_grid->LoadWallURDF(test_wall);
    scene_grid->CreateAndLoadTileURDF(test_floor);
	scene_grid->GenerateArena(width, length);

	// if(inventory_) {
	    
	// }

    // renderer_->sunlight_.ambient = glm::vec3(0.02,0.02,0.02);
    // renderer_->lighting_.exposure = 0.3f;
    // renderer_->lighting_.indirect_strength = 0.25f;
    // renderer_->lighting_.traceshadow_distance = 0.3f;
    // renderer_->lighting_.propagation_distance = 0.3f;
    // renderer_->lighting_.sample_factor = 0.7f;
    // renderer_->lighting_.boost_ambient = 0.01f;
    // renderer_->lighting_.shadow_bias_scale = 0.0003f;
    // renderer_->lighting_.linear_voxelize = true;
}

void Playground::CreateRandomGenerateScene()
{
    if(!scene_)
        scene_ = std::make_shared<MapGrid>();

    scene_->world_->inventory_size_ = 0;

    std::shared_ptr<MapGrid> scene_grid 
        = std::dynamic_pointer_cast<MapGrid>(scene_);

    // renderer_->sunlight_.ambient = glm::vec3(0.15,0.15,0.15);
    // renderer_->lighting_.exposure = 1.5f;
    // renderer_->lighting_.indirect_strength = 0.4f;
    // renderer_->lighting_.traceshadow_distance = 0.5f;
    // renderer_->lighting_.propagation_distance = 0.5f;
    // renderer_->lighting_.sample_factor = 0.4f;
    // renderer_->lighting_.boost_ambient = 0.02f;
    // renderer_->lighting_.shadow_bias_scale = 0.0003f;
    // renderer_->lighting_.linear_voxelize = false;
}

void Playground::LoadXWorldScene(const std::string& filename)
{
    scene_->world_->inventory_size_ = 0;

    boost::python::dict scene_dict;

    Json::Value json_root;
    if (!json_parse_text(filename, json_root)) {
        fprintf(stderr, "Unable to parse %s\n", filename.c_str());
        return;
    }

    if(json_get_string(&json_root, "Version", "") != "xworld3d@2.0")
        return;

    // Scene Size
    Json::Value *json_size;
    if (json_get_object(json_size, &json_root, "Size", Json::objectValue)) {

        float min_x = json_get_float(json_size, "min_x");
        float min_z = json_get_float(json_size, "min_z");
        float max_x = json_get_float(json_size, "max_x");
        float max_z = json_get_float(json_size, "max_z");

        scene_->world_->set_world_size(min_x, min_z, max_x, max_z);
    }

    // Parse levels
    Json::Value *json_levels, *json_level;
    if (!json_get_object(json_levels, &json_root, "Scene", Json::arrayValue)) 
        return;

    for (Json::ArrayIndex index = 0; index < json_levels->size(); index++) {
        if (!json_get_array(json_level, json_levels, index)) 
            return;

        if (json_level->type() != Json::objectValue) continue;

        int obj_id = index;

        std::string label = json_get_string(json_level, "Label");
        std::string path = json_get_string(json_level, "Path");
        int uid = json_get_int(json_level, "UID");
        bool fixed = json_get_bool(json_level, "Fixed");


        Json::Value *json_positions;
        Json::Value *json_position;
        if (!json_get_object(json_positions, json_level, "Position", Json::objectValue)) 
            return;

        boost::python::list position;
        float px = json_get_float(json_positions, "x");
        float py = json_get_float(json_positions, "y");
        float pz = json_get_float(json_positions, "z");
        position.append(px);
        position.append(py);
        position.append(pz);

        Json::Value *json_scales;
        Json::Value *json_scale;
        if (!json_get_object(json_scales, json_level, "Scale", Json::objectValue)) 
            return;

        float uniform_scale = json_get_float(json_scales, "x");

        Json::Value *json_orientations;
        Json::Value *json_orientation;
        if (!json_get_object(json_orientations, json_level, "Orientation", Json::objectValue)) 
            return;

        boost::python::list orientation;
        float qx = json_get_float(json_orientations, "x");
        float qy = json_get_float(json_orientations, "y");
        float qz = json_get_float(json_orientations, "z");
        float qw = json_get_float(json_orientations, "w");
        orientation.append(qx);
        orientation.append(qy);
        orientation.append(qz);
        orientation.append(qw);

        Thing obj = SpawnAnObject(path, position, orientation, uniform_scale, label, fixed);
        scene_dict[uid] = obj;
        objects_.push_back(obj);
    }

    json_scene_ = scene_dict;
}

void Playground::LoadBasicObjects(const boost::python::list doors,
                                  const boost::python::list keys,
                                  const boost::python::list key_tags,
                                  const std::string unlocked_door,
                                  const std::string& wall,
                                  const boost::python::list tiles)
{
    std::shared_ptr<MapGrid> scene_grid 
        = std::dynamic_pointer_cast<MapGrid>(scene_);

    int num_pairs = boost::python::len(keys);
    int num_tiles = boost::python::len(tiles);

    for (int i = 0; i < num_pairs; ++i)
    {
        boost::python::extract<std::string> door(doors[i]);
        boost::python::extract<std::string> key(keys[i]);
        boost::python::extract<std::string> tag(key_tags[i]);
        scene_grid->CreateAndLoadLockedDoorJSON(door);
        scene_grid->CreateAndLoadKeyURDF(key, tag);
    }

    for (int i = 0; i < num_tiles; ++i)
    {
        boost::python::extract<std::string> tile(tiles[i]);
        scene_grid->CreateAndLoadTileURDF(tile);
    }

    scene_grid->LoadUnlockedDoorJSON(unlocked_door);
    scene_grid->LoadWallURDF(wall);
}

void Playground::SpawnModelsConf(const boost::python::dict conf)
{

    std::shared_ptr<MapGrid> scene_grid 
        = std::dynamic_pointer_cast<MapGrid>(scene_);

    if(conf.has_key("single"))
    {
        boost::python::list val = 
            boost::python::extract<boost::python::list>(conf["single"]);
        for (int i = 0; i < boost::python::len(val) / 3; ++i)
        {
            std::string path = boost::python::extract<std::string>(val[3 * i + 0]);
            int room = boost::python::extract<int>(val[3 * i + 1]);
            int num = boost::python::extract<int>(val[3 * i + 2]);

            for (int j = 0; j < num; ++j)
                scene_grid->SpawnSingleObject(path, room);
        }
    }

    if(conf.has_key("pair"))
    {
        boost::python::list val = 
            boost::python::extract<boost::python::list>(conf["pair"]);
        for (int i = 0; i < boost::python::len(val) / 4; ++i)
        {
            std::string path_0 = boost::python::extract<std::string>(val[4 * i + 0]);
            std::string path_1 = boost::python::extract<std::string>(val[4 * i + 1]);
            int room = boost::python::extract<int>(val[4 * i + 2]);
            int num = boost::python::extract<int>(val[4 * i + 3]);

            for (int j = 0; j < num; ++j)
                scene_grid->SpawnPairOfObjects(path_0, path_1, room);
        }
    }
    
    if(conf.has_key("stack"))
    {
        boost::python::list val = 
            boost::python::extract<boost::python::list>(conf["stack"]);
        for (int i = 0; i < boost::python::len(val) / 5; ++i)
        {
            std::string path_0 = boost::python::extract<std::string>(val[5 * i + 0]);
            std::string path_1 = boost::python::extract<std::string>(val[5 * i + 1]);
            int room = boost::python::extract<int>(val[5 * i + 2]);
            int num_top = boost::python::extract<int>(val[5 * i + 3]);
            int num = boost::python::extract<int>(val[5 * i + 4]);

            for (int j = 0; j < num; ++j)
                scene_grid->SpawnStackOfObjects(path_0, path_1, num_top, room);
        }
    }
}

void Playground::SpawnModels()
{
    std::shared_ptr<MapGrid> scene_grid 
        = std::dynamic_pointer_cast<MapGrid>(scene_);
        
    scene_grid->GenerateObjects();
}

void Playground::LoadModels(const boost::python::list models,
                            const boost::python::list tags) 
{
    std::shared_ptr<MapGrid> scene_grid 
        = std::dynamic_pointer_cast<MapGrid>(scene_);

    for (int i = 0; i < boost::python::len(models); ++i)
    {
        boost::python::extract<std::string> model(models[i]);
        boost::python::extract<std::string> tag(tags[i]);
        scene_grid->CreateAndLoadObjectFILE(model, tag);
    }
}

boost::python::list Playground::LocateObjectInGrid(Thing& object)
{
    std::shared_ptr<MapGrid> scene_grid 
        = std::dynamic_pointer_cast<MapGrid>(scene_);

    boost::python::list locate;

    if(!scene_grid) return locate;

    boost::python::tuple position = object.GetPosition();
    float pos_x = boost::python::extract<float>(position[0]);
    float pos_z = boost::python::extract<float>(position[2]);
    auto sub = scene_grid->GetSubTileFromWorldPosition(glm::vec2(pos_x, pos_z));
    
    if(sub) {
        if(auto parent_sptr = sub->parent.lock()) {
            bool occupy = sub->occupied;
            int room_id = parent_sptr->room_id;
            int roomgroup_id = parent_sptr->roomgroup_id;

            locate.append(room_id - 1);
            locate.append(roomgroup_id);
            locate.append(occupy ? 1 : 0);
        }
    }

    return locate;
}

boost::python::list Playground::LocatePositionInGrid(const float x, const float z)
{
    std::shared_ptr<MapGrid> scene_grid 
        = std::dynamic_pointer_cast<MapGrid>(scene_);

    boost::python::list locate;

    if(!scene_grid) return locate;

    auto sub = scene_grid->GetSubTileFromWorldPosition(glm::vec2(x, z));
    
    if(sub) {
        if(auto parent_sptr = sub->parent.lock()) {
            bool occupy = sub->occupied;
            int room_id = parent_sptr->room_id;
            int roomgroup_id = parent_sptr->roomgroup_id;

            locate.append(room_id - 1);
            locate.append(roomgroup_id);
            locate.append(occupy ? 1 : 0);
        }
    }

    return locate;
}

boost::python::list Playground::GetRoomVisitSequence()
{
    std::shared_ptr<MapGrid> scene_grid 
        = std::dynamic_pointer_cast<MapGrid>(scene_);

    boost::python::list visit_sequence;

    if(!scene_grid) return visit_sequence;
    
    for (const auto& room : scene_grid->rooms_)
    {
        int visit = room.visit_sequence;
        visit_sequence.append(visit);
    }

    return visit_sequence;
}

boost::python::list Playground::GetRoomGroups()
{
    std::shared_ptr<MapGrid> scene_grid 
        = std::dynamic_pointer_cast<MapGrid>(scene_);

    boost::python::list room_to_group;

    if(!scene_grid) return room_to_group;
    
    for (const auto& room : scene_grid->rooms_)
    {
        int group_id = room.tiles[0]->roomgroup_id;
        room_to_group.append(group_id);
    }

    return room_to_group;
}

boost::python::list Playground::GetSpaceNearPosition(
    const boost::python::list position, const float radius)
{
    std::shared_ptr<MapGrid> scene_grid 
        = std::dynamic_pointer_cast<MapGrid>(scene_);

    boost::python::list ret;
    if(!scene_grid) return ret;

    float pos_x = boost::python::extract<float>(position[0]); 
    float pos_y = boost::python::extract<float>(position[1]); 
    float pos_z = boost::python::extract<float>(position[2]); 
    glm:;vec2 near = scene_grid->GetASpaceNearPosition(
        glm::vec2(pos_x, pos_z), radius);

    ret.append(near.x);
    ret.append(0);
    ret.append(near.y);
    return ret;
}

boost::python::list Playground::LoadSceneConfigure(const int w, const int l, 
    const int n, const int d)
{
    std::shared_ptr<MapGrid> scene_grid 
        = std::dynamic_pointer_cast<MapGrid>(scene_);

    glm::vec3 start = scene_grid->GenerateLayout(w, l, n, d);
    scene_grid->ResolvePath();

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

    // renderer_->sunlight_.direction = glm::vec3(0.3, 1, 1);
    // renderer_->lighting_.exposure = 1.0f;
    // renderer_->lighting_.indirect_strength = 1.5f;
    // renderer_->lighting_.traceshadow_distance = 0.3f;
    // renderer_->lighting_.propagation_distance = 0.3f;
    // renderer_->lighting_.force_disable_shadow = true;
}

void Playground::CreateEmptyScene(const float min_x, const float max_x,
                                  const float min_z, const float max_z)
{
    if(!scene_) {
        scene_ = std::make_shared<Map>();
        scene_->GenerateGenetricMap();
    }

    scene_->world_->inventory_size_ = 0;
    scene_->world_->set_world_size(min_x, min_z, max_x, max_z);

    // renderer_->sunlight_.direction = glm::vec3(0.3, 1, 1);
    // renderer_->lighting_.exposure = 1.0f;
    // renderer_->lighting_.indirect_strength = 1.5f;
    // renderer_->lighting_.traceshadow_distance = 0.3f;
    // renderer_->lighting_.propagation_distance = 0.3f;
    // renderer_->lighting_.force_disable_shadow = true;
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

    scene_->world_->inventory_size_ = 0;

    if(filter > -1)
        scene_suncg->SetRemoveAll(filter);

    // TODO
    // Assign Props

    scene_suncg->LoadCategoryCSV(metadata.c_str());

    scene_suncg->LoadJSON(house.c_str(), suncg_data_dir.c_str(), true);

    // TODO
    // Assign Map Size

    if(inventory_) {
        // inventory_->AddNonPickableObjectTag("Wall");
        // inventory_->AddNonPickableObjectTag("Floor");
        // inventory_->AddNonPickableObjectTag("Ceiling");
    }
}

void Playground::ResolvePath() 
{
    if(auto scene_grid = std::dynamic_pointer_cast<MapGrid>(scene_)) {
        scene_grid->ResolvePath();
    }
}

void Playground::RemoveAnObject(Thing& object)
{
    if(object.__eq__(agent_)) {
        printf("Cannot remove agent!\n");
        return;
    }

    if(auto obj_sptr = object.GetPtr().lock()) {
        obj_sptr->recycle();
        scene_->world_->BulletStep();
    }
}

Thing Playground::SpawnAnObject(const std::string& file, 
							    const boost::python::list position_py,
				    			const boost::python::list orentation_py,
							    const float scale,
							    const std::string& label,
							    const bool fixed,
                                const bool occupy)
{

	std::weak_ptr<RobotBase> obj_wptr;

	glm::vec3 position   = list2vec3(position_py);
	glm::vec4 orentation = list2vec4(orentation_py);

	obj_wptr = scene_->world_->LoadRobot(
        file,
        glm::vec3(position.x, position.y, position.z),
        glm::vec3(orentation.x,orentation.y,orentation.z),
        orentation.w,
        glm::vec3(scale, scale, scale),
        label,
        fixed // fixed ? 0 : 100
    );

	if(auto obj_sptr = obj_wptr.lock()) {
		if(!fixed) {
   			obj_sptr->ignore_baking(true);
   		}
		obj_sptr->DisableSleeping();
	}

    if(occupy) {

        if(auto scene_grid = std::dynamic_pointer_cast<MapGrid>(scene_)) {

            BBox box(
                glm::vec2(position.x - 0.5f, position.z - 0.5f),
                glm::vec2(position.x + 0.5f, position.z + 0.5f)
            );

            scene_grid->GenerateEmpty(box);

            if(!scene_grid->resolve_path_) {
                std::shared_ptr<SubTile> subtile = scene_grid->GetSubTileFromWorldPosition(
                    glm::vec2(position.x, position.z));

                if(auto tile = subtile->parent.lock()) {

                    int roomgroup_id = tile->roomgroup_id;

                    auto& waypoints = scene_grid->roomgroups_[roomgroup_id].waypoints;
                    waypoints.push_back(subtile);   
                }
            }
        }
    }

	//scene_->CreateLabel(file, label);

	Thing object_temp;
	object_temp.SetPtr(obj_wptr);
    objects_.push_back(object_temp);

    scene_->world_->BulletStep();

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

void Playground::UpdateAttachCamera(const float pitch)
{
    float camera_pitch_ = glm::clamp(pitch, -45.0f, 45.0f);
    scene_->world_->rotate_camera(main_camera_, camera_pitch_);
}

void Playground::FreeCamera(const boost::python::list position, 
                            const float yaw, const float pitch)
{
    glm::vec3 pos = list2vec3(position);

    main_camera_ = scene_->world_->add_camera(pos,
        vec3(0, 0, 0), camera_aspect_);

    main_camera_->yaw_ = yaw;
    main_camera_->pitch_ = pitch;
    main_camera_->Update();
}

void Playground::UpdateFreeCamera(const boost::python::list position, 
                                  const float yaw, const float pitch)
{
    glm::vec3 pos = list2vec3(position);

    main_camera_->position_ = pos;
    main_camera_->yaw_ = yaw;
    main_camera_->pitch_ = pitch;
    main_camera_->Update();
}

void Playground::Initialize()
{
	// renderer_->Init(main_camera_);
    scene_->world_->BulletStep();

	if(auto obj_sptr = agent_.GetPtr().lock()) {
		obj_sptr->UnFreeze();
	}

    // glm::vec3 world_front(1,0,0);
    // float angle = 

    camera_yaw_ = 0.0f;
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
    // dictionary["vel"]    = boost::python::make_tuple(3, -100, 100);
    // dictionary["avel"]   = boost::python::make_tuple(3, -100, 100);
    // dictionary["acc"]    = boost::python::make_tuple(3, -100, 100);
    // dictionary["aacc"]   = boost::python::make_tuple(3, -100, 100);

    return dictionary;
}

boost::python::tuple Playground::GetCameraPosition() const
{
    glm::vec3 position = main_camera_->position_;
    return boost::python::make_tuple(position.x, position.y, position.z);
}

boost::python::tuple Playground::GetCameraRight() const
{
    glm::vec3 right = main_camera_->right_;
    return boost::python::make_tuple(right.x, right.y, right.z);
}

boost::python::tuple Playground::GetCameraFront() const
{
    glm::vec3 front = main_camera_->front_;
    return boost::python::make_tuple(front.x, front.y, front.z);
}

boost::python::tuple Playground::GetCameraUp() const
{
    glm::vec3 up = main_camera_->up_;
    return boost::python::make_tuple(up.x, up.y, up.z);
}

float Playground::GetCameraYaw() const
{
    glm::vec3 world_front(1,0,0);
    glm::vec3 world_up(0,1,0);
    glm::vec3 front_norm = glm::normalize(main_camera_->front_);
    float yaw = glm::orientedAngle(front_norm, world_front, world_up);
    return yaw;
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

void Playground::HoldActions(const bool hold)
{
    hold_actions_ = hold;
}

boost::python::dict Playground::UpdateSimulationWithAction(const int action)
{
    assert(action < 16 && action > -1);

    if(hold_actions_)
        return boost::python::dict();

    current_event_ = boost::python::list();

    if(!interact_ && !inventory_opened_) {
        if(action == 0) {
            MoveForward(25);

            // Update Event
            current_event_.append("MoveForward");
        } 
        else if(action == 1) {
            MoveBackward(25);

            // Update Event
            current_event_.append("MoveBackward");
        }
        else if(action == 2) {
            TurnLeft(25);

            // Update Event
            current_event_.append("TurnLeft");
        }
        else if(action == 3) {
            TurnRight(25);

            // Update Event
            current_event_.append("TurnRight");
        }
        else if(action == 4) {
            LookUp();

            // Update Event
            current_event_.append("LookUp");
        }
        else if(action == 5) {
            LookDown();

            // Update Event
            current_event_.append("LookDown");
        }
        else if(action == 6) {
            Attach();

            // Update Event
            current_event_.append("Attach");
        }
        else if(action == 7) {
            Detach();

            // Update Event
            current_event_.append("Detach");
        }
        else if(action == 8) {
            std::string obj = Grasp();

            // Update Event
            current_event_.append("Grasp");
            current_event_.append(obj);
        }
        else if(action == 9) {
            std::string obj = PutDown();

            // Update Event
            current_event_.append("PutDown");
            current_event_.append(obj);
        }
        else if(action == 10) {
            boost::python::list rotate_angle;
            rotate_angle.append(0);
            rotate_angle.append(1.57);
            rotate_angle.append(0);
            std::string obj = Rotate(rotate_angle);

            // Update Event
            current_event_.append("Rotate");
            current_event_.append(obj);
        }
        else if(action == 11) {
            current_actions_ = EnableInteraction();

            // Update Event
            current_event_.append("EnableInteraction");
        }
        else if(action == 12) {
            current_objects_ = OpenInventory();

            // Update Event
            current_event_.append("OpenInventory");
        }
    } else if(interact_) {
        if(action == 13) {
            DisableInteraction();
            current_actions_ = boost::python::list();

            // Update Event
            current_event_.append("DisableInteraction");
        }
        else if(action < boost::python::len(current_actions_)) {
            bool success = TakeAction(action);

            // Update Event
            current_event_.append("Action");
            current_event_.append(action);
            current_event_.append(success ? "S" : "F");
        }
    } else if(inventory_opened_) {
        if(action == 13) {
            CloseInventory();
            current_objects_ = boost::python::list();

            // Update Event
            current_event_.append("CloseInventory");
        }
        else if(action < boost::python::len(current_objects_)) {
            Use(action);

            // Update Event
            current_event_.append("Use");
            current_event_.append(action);
        }
    }

    if(crowd_) {
        crowd_->Update();

        if(kill_after_arrived_)
            crowd_->KillAgentOnceArrived();
    }

    scene_->world_->BulletStep();

    boost::python::dict ret;
    //ret["reward"] = -0.1f;
    ret["actions"] = current_actions_;
    ret["inventory"] = current_objects_;

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
}

void Playground::Update()
{
	UpdateSimulation();
	UpdateRenderer();
}

float Playground::GetNearClippingDistance()
{
    return main_camera_->GetNear();
}

float Playground::GetFarClippingDistance()
{
    return main_camera_->GetFar();
}

boost::python::object Playground::GetCameraRGBDRaw()
{
    unsigned char * raw_image = renderer_->GetRenderedImage().data.data();

    size_t destination_size = sizeof(unsigned char) * (w_ * h_ * 4);

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

boost::python::list Playground::OpenInventory()
{
    boost::python::list ret;

    if(inventory_) {
        std::vector<std::string> objects = inventory_->GetObjectTagInInventory();

        inventory_opened_ = true;

        for (int i = 0; i < objects.size(); ++i) 
            ret.append(objects[i]);
    }

    return ret;
}

void Playground::CloseInventory()
{
    inventory_opened_ = false;
}

boost::python::list Playground::EnableInteraction()
{
    // glm::vec3 from = main_camera_->Position;
    // glm::vec3 to = main_camera_->Front * 3.0f + from;
    
    // auto bullet_world = scene_->world_;
    // std::vector<RayTestInfo> temp_res;
    // std::vector<Ray> temp_ray;
    // temp_ray.push_back({from, to});

    // bullet_world->BatchRayTest(temp_ray, temp_res);

    int bullet_id = -1;

    glm::vec3 from = main_camera_->position_;
    glm::vec3 frnt = main_camera_->front_;

    auto bullet_world = scene_->world_;

    for (int i = 0; i < bullet_world->robot_list_.size(); ++i)
    {
        auto body = bullet_world->robot_list_[i];

        if (!body->is_recycled()) {

            if(std::dynamic_pointer_cast<RobotWithAnimation>(body) || 
               std::dynamic_pointer_cast<RobotWithConvertion>(body)) {

                glm::vec3 aabb_min, aabb_max;
                body->root_part_->GetAABB(aabb_min, aabb_max);

                bool intersect = bullet_engine::RayAABBIntersect(
                    {from, from + frnt}, aabb_min, aabb_max);

                glm::vec3 object_pos = (aabb_max - aabb_min) * 0.5f + aabb_min;
                float dist = glm::distance(object_pos, from);

                if(intersect && dist < bullet_engine::kDistanceThreshold) {
                    bullet_id = body->body_data_.body_uid;
                }
            }
        }
    }

    // printf("bullet_id: %d\n", bullet_id);

    boost::python::list ret;

    if(bullet_id >= 0) {

        // TODO
        // Is id_to_robot_fully functional?
        auto object = bullet_world->id_to_robot_[bullet_id];

        std::vector<std::string> actions = object->GetActions();

        for (int i = 0; i < actions.size(); ++i) {
            ret.append(actions[i]);
        }

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
    glm::vec3 from = main_camera_->position_;
    glm::vec3 to = main_camera_->front_ * bullet_engine::kDistanceThreshold + from;
    
    auto bullet_world = scene_->world_;
    std::vector<RayTestInfo> temp_res;
    std::vector<Ray> temp_ray;
    temp_ray.push_back({from, to});

    bullet_world->BatchRayTest(temp_ray, temp_res);

    if(temp_res[0].bullet_id >= 0) {

        auto object = bullet_world->id_to_robot_[temp_res[0].bullet_id];

        if(auto agent = agent_.GetPtr().lock()) {
            agent->AttachTo(object);
        }
    }
}

void Playground::Detach()
{
    if(auto agent = agent_.GetPtr().lock()) {
        agent->Detach();
    }
}

std::string Playground::Grasp()
{
	if(!inventory_)
		return "Nothing";

	glm::vec3 from = main_camera_->position_;
	glm::vec3 to = main_camera_->front_ * bullet_engine::kDistanceThreshold + from;

    if(auto agent_sptr = agent_.GetPtr().lock())
    {
        return agent_sptr->PickUp(inventory_, from, to);
    }

    return "Nothing";
}

std::string Playground::PutDown()
{
	if(!inventory_)
		return "Nothing";

	glm::vec3 from = main_camera_->position_;
	glm::vec3 to = main_camera_->front_ * bullet_engine::kDistanceThreshold + from;

    if(auto agent_sptr = agent_.GetPtr().lock())
    {
        return agent_sptr->PutDown(inventory_, from, to);
    }

    return "Nothing";
}

std::string Playground::Rotate(const boost::python::list angle_py)
{
    if(!inventory_)
        return "Nothing";

	glm::vec3 angle = list2vec3(angle_py);

	glm::vec3 from = main_camera_->position_;
	glm::vec3 to = main_camera_->front_ * bullet_engine::kDistanceThreshold + from;
	
	if(auto agent_sptr = agent_.GetPtr().lock())
    {
        return agent_sptr->Rotate(angle, from, to);
    }

    return "Nothing";
}

bool Playground::TakeAction(const int action_id)
{
	// glm::vec3 from = main_camera_->Position;
	// glm::vec3 to = main_camera_->Front * 3.0f + from;
	
	// auto bullet_world = scene_->world_;
    // std::vector<RayTestInfo> temp_res;
    // std::vector<Ray> temp_ray;
    // temp_ray.push_back({from, to});

    // bullet_world->BatchRayTest(temp_ray, temp_res);

    int bullet_id = -1;

    glm::vec3 from = main_camera_->position_;
    glm::vec3 frnt = main_camera_->front_;

    auto bullet_world = scene_->world_;

    for (int i = 0; i < bullet_world->robot_list_.size(); ++i)
    {
        auto body = bullet_world->robot_list_[i];
        if (!body->is_recycled()) {

            if(std::dynamic_pointer_cast<RobotWithAnimation>(body) || 
               std::dynamic_pointer_cast<RobotWithConvertion>(body)) {

                glm::vec3 aabb_min, aabb_max;
                body->root_part_->GetAABB(aabb_min, aabb_max);

                bool intersect = bullet_engine::RayAABBIntersect(
                    {from, from + frnt}, aabb_min, aabb_max);

                glm::vec3 object_pos = (aabb_max - aabb_min) * 0.5f + aabb_min;
                float dist = glm::distance(object_pos, from);

                if(intersect && dist < bullet_engine::kDistanceThreshold) {
                    bullet_id = body->body_data_.body_uid;
                }
            }
        }
    }



    if(bullet_id >= 0 && action_id < boost::python::len(current_actions_)) {
    	auto object = bullet_world->id_to_robot_[bullet_id];

    	return object->TakeAction(action_id);
    }

    return false;
}

void Playground::Use(const int object_id)
{
    // glm::vec3 from = main_camera_->Position;
    // glm::vec3 to = main_camera_->Front * 3.0f + from;
    
    // auto bullet_world = scene_->world_;
    // std::vector<RayTestInfo> temp_res;
    // std::vector<Ray> temp_ray;
    // temp_ray.push_back({from, to});

    // bullet_world->BatchRayTest(temp_ray, temp_res);

    int bullet_id = -1;

    glm::vec3 from = main_camera_->position_;
    glm::vec3 frnt = main_camera_->front_;

    auto bullet_world = scene_->world_;

    for (int i = 0; i < bullet_world->robot_list_.size(); ++i)
    {
        auto body = bullet_world->robot_list_[i];

        if (!body->is_recycled()) {

            if(std::dynamic_pointer_cast<RobotWithAnimation>(body) || 
               std::dynamic_pointer_cast<RobotWithConvertion>(body)) {

                glm::vec3 aabb_min, aabb_max;
                body->root_part_->GetAABB(aabb_min, aabb_max);

                bool intersect = bullet_engine::RayAABBIntersect(
                    {from, from + frnt}, aabb_min, aabb_max);

                glm::vec3 object_pos = (aabb_max - aabb_min) * 0.5f + aabb_min;
                float dist = glm::distance(object_pos, from);

                if(intersect && dist < bullet_engine::kDistanceThreshold) {
                    bullet_id = body->body_data_.body_uid;
                }
            }
        }
    }

    if(bullet_id >= 0) {
        auto object = bullet_world->id_to_robot_[bullet_id];

        if(object_id < inventory_->GetNumUsedSpace()) {

            std::string use_tag = boost::python::extract<std::string>(current_objects_[object_id]);

            object->InteractWith(use_tag);
        }
    }
}

void Playground::ControlJointPositions(const Thing& object, 
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

            auto joint_ptr = object_sptr->joints_[joint_id];
            joint_ptr->SetJointMotorControlPosition(position, 0.1f, 1.0f, max_force);
        }
    }
}

void Playground::ControlJointVelocities(const Thing& object, 
                                        const boost::python::dict joint_velocities,
                                        const float max_force)
{
    if(auto object_sptr = object.GetPtr().lock())
    {
        boost::python::list joint_velocities_keys = joint_velocities.keys();

        for (int i = 0; i < boost::python::len(joint_velocities_keys); ++i)
        {
            int joint_id   = boost::python::extract<int>(joint_velocities_keys[i]);
            float velocity = boost::python::extract<float>(joint_velocities[joint_id]);

            auto joint_ptr = object_sptr->joints_[joint_id];
            joint_ptr->SetJointMotorControlVelocity(velocity, 1.0f, max_force);
        }
    }
}

boost::python::list Playground::QueryLastEvent()
{
    return current_event_;
    // current_event_ = boost::python::list();
}

bool Playground::QueryContact(Thing& object)
{
    if(auto object_sptr = object.GetPtr().lock())
    {
        auto object_root_ptr = object_sptr->root_part_;

        std::vector<ContactPoint> contact_points;
        scene_->world_->GetContactPoints(object_sptr, object_root_ptr, contact_points);

        if(contact_points.size())
            return true;
    }

    return false;
}

bool Playground::QueryObjectWithLabelAtCameraCenter(const std::string& label)
{
    glm::vec3 from = main_camera_->position_;
    glm::vec3 to = main_camera_->front_ * bullet_engine::kDistanceThreshold + from;
    
    auto bullet_world = scene_->world_;
    std::vector<RayTestInfo> temp_res;
    std::vector<Ray> temp_ray;
    temp_ray.push_back({from, to});

    bullet_world->BatchRayTest(temp_ray, temp_res);

    if(temp_res[0].bullet_id >= 0) {
        auto object = bullet_world->id_to_robot_[temp_res[0].bullet_id];
        
        return object->body_data_.label == label;
    }

    return false;
}

boost::python::list Playground::QueryObjectNearObject(Thing& object, const bool exlude,
    const float dist)
{
    boost::python::list result;

    auto bullet_world = scene_->world_;

    auto object_wptr = object.GetPtr();
    auto object_sptr = object_wptr.lock();
    int object_self_id = object_sptr->body_data_.body_uid;


    boost::python::tuple object_position = object.GetPosition();
    float pos_x = boost::python::extract<float>(object_position[0]); 
    float pos_y = boost::python::extract<float>(object_position[1]); 
    float pos_z = boost::python::extract<float>(object_position[2]); 
    glm::vec3 object_pos(pos_x, pos_y, pos_z);

    for (int i = 0; i < bullet_world->robot_list_.size(); ++i)
    {
        auto body = bullet_world->robot_list_[i];

        if (!body->is_recycled() && body->body_data_.body_uid >= 0) {

            if(exlude && object_self_id == body->body_data_.body_uid)
                continue;

            btTransform tr_bt = body->root_part_->object_position_;
            btVector3 pos_bt = tr_bt.getOrigin();
            glm::vec3 pos(pos_bt[0], pos_bt[1], pos_bt[2]);

            if(glm::distance(pos, object_pos) <= dist) {
                Thing object_temp;
                auto object = bullet_world->id_to_robot_[body->body_data_.body_uid];
                object_temp.SetPtr(object);

                result.append(object_temp);
            }
        }
    }

    return result;
}

boost::python::list Playground::QueryObjectByLabel(const std::string& label)
{
	boost::python::list result;

	std::vector<ObjectAttributes> temp;
    scene_->world_->QueryObjectByLabel(label, temp);

    auto bullet_world = scene_->world_;

    for(int i = 0; i < temp.size(); ++i)
    {
    	// Range r;
    	// r.min = temp[i].aabb_min;
    	// r.max = temp[i].aabb_max;
    	// result.append(r);

        Thing object_temp;
        if(temp[i].bullet_id >= 0) {
            auto object = bullet_world->id_to_robot_[temp[i].bullet_id];
            object_temp.SetPtr(object);
        }
        result.append(object_temp);
    }

    return result;
}

Thing Playground::QueryObjectAtCameraCenter()
{
	glm::vec3 from = main_camera_->position_;
	glm::vec3 to = main_camera_->front_ * bullet_engine::kDistanceThreshold + from;
	
	auto bullet_world = scene_->world_;
    std::vector<RayTestInfo> temp_res;
    std::vector<Ray> temp_ray;
    temp_ray.push_back({from, to});

    bullet_world->BatchRayTest(temp_ray, temp_res);

    Thing object_temp;
    if(temp_res[0].bullet_id >= 0) {
    	auto object = bullet_world->id_to_robot_[temp_res[0].bullet_id];
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

            object_a_sptr->root_part_->GetAABB(a_min, a_max);
            object_b_sptr->root_part_->GetAABB(b_min, b_max);

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
    	glm::vec3 camera_center = main_camera_->position_;
    	glm::vec3 camera_front  = main_camera_->front_;

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
    	glm::vec3 camera_center = main_camera_->position_;
    	glm::vec3 camera_front  = main_camera_->front_;

    	if(glm::distance(camera_center, aabb_center) < 3.0f) 
    		return true;
    }
    return false;
}

void Playground::Teleport(Thing object, const boost::python::list position_py,
                                        const boost::python::list orientation_py)
{
	glm::vec3 position = list2vec3(position_py);
    glm::vec4 orientation = list2vec4(orientation_py);

	if(auto object_sptr = object.GetPtr().lock())
	{
		auto root_part_ = object_sptr->root_part_;

		btTransform root_transform = root_part_->object_position_;
		root_transform.setOrigin(btVector3(position.x, position.y, position.z));
        root_transform.setRotation(btQuaternion(
            btVector3(orientation.x, orientation.y, orientation.z), orientation.w));
		scene_->world_->SetTransformation(object_sptr, root_transform);
	}
}