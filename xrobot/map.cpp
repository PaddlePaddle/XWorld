#include "map.h"

namespace xrobot
{

Map::~Map()
{
	for (auto aabb : sections_AABB_)
	{
		delete aabb;
	}

	for (auto aabb : first_layer_AABB_)
	{
		delete aabb;
	}

	for (auto aabb : second_layer_AABB_)
	{
		delete aabb;
	}

	delete world_;
}

Map::Map() : world_(nullptr),
on_floor_list_(0), on_object_list_(0), either_list_(0),
sections_AABB_(0), sections_(1),
first_layer_AABB_(0), first_layer_(1),
second_layer_AABB_(0), second_layer_(1),
first_layer_map_(0), empty_map_(0),
rand_device_(), mt_(rand_device_()),
map_(nullptr)
{
	world_ = new World();
	world_->BulletInit(-9.81f, 0.01f);
}

// This function is adapted from SUNCG!
int Map::GetJsonArrayEntry(Json::Value *&result, Json::Value *array, unsigned int k, int expected_type)
{
  // Check array type
  if (array->type() != Json::arrayValue) {
    fprintf(stderr, "JSON: not an array\n");
    return 0;
  }

  // Check array size
  if (array->size() <= k) {
    // fprintf(stderr, "JSON array has no member %d\n", k);
    return 0;
  }

  // Get entry
  result = &((*array)[k]);
  if (result->type() == Json::nullValue) {
    // fprintf(stderr, "JSON array has null member %d\n", k);
    return 0;
  }

  // Check entry type
  if (expected_type > 0) {
    if (result->type() != expected_type) {
      // fprintf(stderr, "JSON array entry %d has unexpected type %d (rather than %d)\n", k, result->type(), expected_type);
      return 0;
    }
  }
  
  // Return success
  return 1;
}

// This function is adapted from SUNCG!
int Map::GetJsonObjectMember(Json::Value *&result, Json::Value *object, const char *str, int expected_type)
{
  // Check object type
  if (object->type() != Json::objectValue) {
    // fprintf(stderr, "JSON: not an object\n");
    return 0;
  }

  // Check object member
  if (!object->isMember(str)) {
    // fprintf(stderr, "JSON object has no member named %s\n", str);
    return 0;
  }

  // Get object member
  result = &((*object)[str]);
  if (result->type() == Json::nullValue) {
    // fprintf(stderr, "JSON object has null member named %s\n", str);
    return 0;
  }

  // Check member type
  if (expected_type > 0) {
    if (result->type() != expected_type) {
      // fprintf(stderr, "JSON object member %s has unexpected type %d (rather than %d)\n", str, result->type(), expected_type);
      return 0;
    }
  }
  
  // Check for empty strings
  if (result->type() == Json::stringValue) {
    if (result->asString().length() == 0) {
      // fprintf(stderr, "JSON object has zero length string named %s\n", str);
      return 0;
    }
  }

  // Return success
  return 1;
}

// This function is adapted from SUNCG!
void Map::LoadJSON(const char * houseFile, const char * input_data_directory, const bool concave)
{
	FILE* fp = fopen(houseFile, "rb");
	if (!fp) {
		fprintf(stderr, "Unable to open SUNCG file %s\n", houseFile);
	return;
	}

	std::string text;
	fseek(fp, 0, SEEK_END);
	long const size = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	char* buffer = new char[size + 1];
	unsigned long const usize = static_cast<unsigned long const>(size);
	if (fread(buffer, 1, usize, fp) != usize) { fprintf(stderr, "Unable to read %s\n", houseFile); return; }
	else { buffer[size] = 0; text = buffer; }
	delete[] buffer;

	fclose(fp);

	// Digest file
	Json::Value json_root;
	Json::Reader json_reader;
	Json::Value *json_items, *json_item, *json_value;
	if (!json_reader.parse(text, json_root, false)) {
		fprintf(stderr, "Unable to parse %s\n", houseFile);
		return;
	}

	  // Get/check version
	char version[1024];
	strncpy(version, "suncg@1.0.0", 1024);
	if (GetJsonObjectMember(json_value, &json_root, "version", Json::stringValue)) {
		strncpy(version, json_value->asString().c_str(), 1024);
		if (strcmp(version, "suncg@1.0.0")) {
			fprintf(stderr, "Unrecognized version %s in SUNCG file %s\n", version, houseFile);
			return;
		}
	}

	// Get scene id
	char scene_id[1024];
	strncpy(scene_id, "NoName", 1024);
	if (GetJsonObjectMember(json_value, &json_root, "id", Json::stringValue)) {
		strncpy(scene_id, json_value->asString().c_str(), 1024);
	}

	// Get scene up direction
	vec3 scene_up(0, 1, 0);
	if (GetJsonObjectMember(json_items, &json_root, "up", Json::arrayValue)) {
		if (json_items->size() >= 3) {
			if (GetJsonArrayEntry(json_item, json_items, 0))
				scene_up[0] = json_item->asDouble();
			if (GetJsonArrayEntry(json_item, json_items, 1))
				scene_up[1] = json_item->asDouble();
			if (GetJsonArrayEntry(json_item, json_items, 2))
				scene_up[2] = json_item->asDouble();
			scene_up = glm::normalize(scene_up);
		}
	}

	// Get scene front direction
	vec3 scene_front(0, 0, 1);
	if (GetJsonObjectMember(json_items, &json_root, "front", Json::arrayValue)) {
		if (json_items->size() >= 3) {
			if (GetJsonArrayEntry(json_item, json_items, 0))
				scene_front[0] = json_item->asDouble();
			if (GetJsonArrayEntry(json_item, json_items, 1))
				scene_front[1] = json_item->asDouble();
			if (GetJsonArrayEntry(json_item, json_items, 2))
				scene_front[2] = json_item->asDouble();
			scene_front = glm::normalize(scene_front);
		}
	}

	// Get scene scale factor (to convert to meters)
	double scaleToMeters = 1.0;
	if (GetJsonObjectMember(json_value, &json_root, "scaleToMeters")) {
		scaleToMeters = json_value->asDouble();
	}
	printf("scene id: %s\n", scene_id);


	// Parse levels
	Json::Value *json_levels, *json_level;
	if (!GetJsonObjectMember(json_levels, &json_root, "levels", Json::arrayValue)) return;
	for (Json::ArrayIndex index = 0; index < json_levels->size(); index++) {
		if (!GetJsonArrayEntry(json_level, json_levels, index)) return;
		if (json_level->type() != Json::objectValue) continue;

		// Parse level attributes
		int level_id = index;
		if (GetJsonObjectMember(json_value, json_level, "valid"))
		if (!json_value->asString().compare(std::string("0")))  continue;
		if (GetJsonObjectMember(json_value, json_level, "id"))
		level_id = atoi(json_value->asString().c_str());


		Json::Value *json_nodes, *json_node, *json_materials;
		if (GetJsonObjectMember(json_nodes, json_level, "nodes", Json::arrayValue)) {
			if (json_nodes->size() == 0) continue;
			for (Json::ArrayIndex index = 0; index < json_nodes->size(); index++) {
				if (!GetJsonArrayEntry(json_node, json_nodes, index)) continue; 
				if (json_node->type() != Json::objectValue) continue;

				char node_id[1024] = { '\0' };;
				char modelId[1024] = { '\0' };;
				char node_type[1024] = { '\0' };
				int hideCeiling = 0, hideFloor = 0, hideWalls = 0;
				int isMirrored = 0, state = 0;
				if (GetJsonObjectMember(json_value, json_node, "valid"))
				if (!json_value->asString().compare(std::string("0")))  continue;
				if (GetJsonObjectMember(json_value, json_node, "id"))
					strncpy(node_id, json_value->asString().c_str(), 1024);
				if (GetJsonObjectMember(json_value, json_node, "type")) 
					strncpy(node_type, json_value->asString().c_str(), 1024);
				if (GetJsonObjectMember(json_value, json_node, "modelId"))
					strncpy(modelId, json_value->asString().c_str(), 1024);
				if (GetJsonObjectMember(json_value, json_node, "hideCeiling")) 
				if (!json_value->asString().compare(std::string("1"))) hideCeiling = 1;
				if (GetJsonObjectMember(json_value, json_node, "hideFloor")) 
				if (!json_value->asString().compare(std::string("1"))) hideFloor = 1;
				if (GetJsonObjectMember(json_value, json_node, "hideWalls")) 
				if (!json_value->asString().compare(std::string("1"))) hideWalls = 1;
				if (GetJsonObjectMember(json_value, json_node, "isMirrored")) 
				if (!json_value->asString().compare(std::string("1"))) isMirrored = 1;
				if (GetJsonObjectMember(json_value, json_node, "state")) 
				if (!json_value->asString().compare(std::string("1"))) state = 1;


				mat4 transformation = mat4(1);
				if (GetJsonObjectMember(json_items, json_node, "transform", Json::arrayValue)) {
					if (json_items->size() >= 16) {
						mat4 matrix = mat4(1);
						for (Json::ArrayIndex index = 0; index < json_items->size(); index++) {
							if (!GetJsonArrayEntry(json_item, json_items, index)) continue;
							matrix[index%4][index/4] = json_item->asDouble();
						}

						transformation = glm::transpose(matrix);
					}
				}


				vec3 scale = vec3(1);
				quat rotation = quat();
				vec3 translation = vec3(0);
				vec3 skew = vec3(0);
				vec4 persp = vec4(0);
				glm::decompose(transformation, scale, rotation, translation, skew, persp);

				//rotation = glm::normalize(rotation);

				// printf("Position: %f %f %f \n", translation.x, translation.y, translation.z);
				// printf("Scale: %f %f %f\n", scale.x, scale.y, scale.z);

				// Create scene node(s) based on type
				char obj_name[4096], node_name[4096];
				if (!strcmp(node_type, "Ground")) {
					sprintf(obj_name, "%s/room/%s/%sf.obj", input_data_directory, scene_id, modelId); 
					if (!hideFloor) {
						//printf("Load Floor: %s\n", obj_name);
						// createObjectAtTransform(obj_name,
						// 	translation.x, translation.y, translation.z,
						// 	rotation.x, rotation.y, rotation.z, rotation.w
						// 	//glm::max(glm::max(scale.x, scale.y), scale.z)
						// );
						// world->load_obj(
						// 	obj_name,
						// 	btVector3(translation.x,translation.y,translation.z),
						// 	btQuaternion(rotation.x,rotation.y,rotation.z,rotation.z),
						// 	btVector3(scale.x, scale.y, scale.z)
						// );
						world_->LoadOBJ2(
							obj_name,
							btVector3(translation.x,translation.y,translation.z),
							btQuaternion(rotation.x,rotation.y,rotation.z,rotation.w),
							btVector3(scale.x, scale.y, scale.z),
							isMirrored == 1 ? -1.0f : 1.0f
						);
					}
				}
				else if(!strcmp(node_type, "Room"))
				{
					// Create node for floor
					sprintf(obj_name, "%s/room/%s/%sf.obj", input_data_directory, scene_id, modelId); 
					if (!hideFloor) {
						//printf("Load Room: %s\n", obj_name);
						// createObjectAtTransform(obj_name,
						// 	translation.x, translation.y, translation.z,
						// 	rotation.x, rotation.y, rotation.z, rotation.w
						// 	//glm::max(glm::max(scale.x, scale.y), scale.z)
						// );
						world_->LoadOBJ2(
							obj_name,
							btVector3(translation.x,translation.y,translation.z),
							btQuaternion(rotation.x,rotation.y,rotation.z,rotation.w),
							btVector3(scale.x, scale.y, scale.z),
							isMirrored == 1 ? -1.0f : 1.0f
						);
					}


					// Create node for walls
					sprintf(obj_name, "%s/room/%s/%sw.obj", input_data_directory, scene_id, modelId); 
					if (!hideWalls) {
						//printf("Load Wall: %s\n", obj_name);
						// createObjectAtTransform(obj_name,
						// 	translation.x, translation.y, translation.z,
						// 	rotation.x, rotation.y, rotation.z, rotation.w
						// 	//glm::max(glm::max(scale.x, scale.y), scale.z)
						// );
						world_->LoadOBJ2(
							obj_name,
							btVector3(translation.x,translation.y,translation.z),
							btQuaternion(rotation.x,rotation.y,rotation.z,rotation.w),
							btVector3(scale.x, scale.y, scale.z),
							isMirrored == 1 ? -1.0f : 1.0f,
							concave
						);
					}
				}
				else if(!strcmp(node_type, "Object"))
				{
					if (state) sprintf(obj_name, "%s/object/%s/%s_0.obj", input_data_directory, modelId, modelId); 
					else sprintf(obj_name, "%s/object/%s/%s.obj", input_data_directory, modelId, modelId); 
					if (true) {
						//printf("Load Object: %s\n", obj_name);
						// createObjectAtTransform(obj_name,
						// 	translation.x, translation.y, translation.z,
						// 	rotation.x, rotation.y, rotation.z, rotation.w
						// 	//glm::max(glm::max(scale.x, scale.y), scale.z)
						// );

						 world_->LoadOBJ2(
							obj_name,
							btVector3(translation.x,translation.y,translation.z),
							btQuaternion(rotation.x,rotation.y,rotation.z,rotation.w),
							btVector3(scale.x, scale.y, scale.z),
							isMirrored == 1 ? -1.0f : 1.0f
						);
					}
				}

			}
		}
	}

}

void Map::CreateSectionType(const std::string& floor, const std::string& wall, const std::string& door)
{
	assert(!floor.empty());

	SectionType st(false, floor, wall, door);

	section_types_.push_back(st);
}


glm::vec3 Map::GenerateFloorPlan(const int w, const int l)
{
	assert(section_types_.size() > 0);

	glm::vec3 random_center;

	map_ = new MapGenerator(w, l, section_types_.size(), 12, 0.6);
	map_->generate();
	map_->print_map();

	int** map_ptr = map_->get_map();
	bool first = true;

	// Convert to AABBs
	for(int i = 0; i < w; ++i)
	for(int j = 0; j < l; ++j)
	{
		const int id = map_ptr[i][j];
		if(id > 0)
		{
			glm::vec3 tile_center = glm::vec3(2 * i, 0, 2 * j);
			glm::vec3 minAABB = tile_center - glm::vec3(1, 0, 1);
			glm::vec3 maxAABB = tile_center + glm::vec3(1, 0, 1);


			if(first)
			{
				random_center = tile_center;
				CreateEmptyVolume(minAABB.x, maxAABB.x, minAABB.z, maxAABB.z);
				first = false;
			}


			// Four Directions
			const int dirx[4] = {1, -1, 0, 0};
			const int diry[4] = {0, 0, 1, -1};
			for (int d = 0; d < 4; ++d)
			{
				int x = i + dirx[d];
				int y = j + diry[d];

				if(x >= 0 && x < w && y >= 0 && y < l)
				{
					// Inner
					if(map_ptr[x][y] < 1) // Outter Side
					{
						GenerateWall(2 * i + dirx[d], 1, 2 * j + diry[d], d, section_types_[id - 1].urdf_wall);
					}
					else if(map_ptr[x][y] != map_ptr[i][j]) // Door????
					{
						glm::vec3 door_center = glm::vec3(2 * i + dirx[d], 0, 2 * j + diry[d]);
						glm::vec3 minAABB = door_center - glm::vec3(0.5, 0, 0.5);
						glm::vec3 maxAABB = door_center + glm::vec3(0.5, 0, 0.5);

						CreateEmptyVolume(minAABB.x, maxAABB.x, minAABB.z, maxAABB.z);
						GenerateDoor(2 * i + dirx[d], 1, 2 * j + diry[d], d, section_types_[id - 1].urdf_door);
					}
				}
				else
				{
					// Outter
					GenerateWall(2 * i + dirx[d], 1, 2 * j + diry[d], d, section_types_[id - 1].urdf_wall);
				}
			}


			GenerateFloor(2 * i, 0, 2 * j, section_types_[id - 1].urdf_floor);
			GenerateCeiling(2 * i, 2, 2 * j, section_types_[id - 1].urdf_ceiling);
			CreateSectionWithSize(minAABB.x, maxAABB.x, minAABB.z, maxAABB.z);
		}
	}

	return random_center;
}

void Map::GenerateCeiling(const float x, const float y, const float z, const string& st)
{

}

void Map::GenerateDoor(const float x, const float y, const float z, const int face, const string st)
{
	const int dir[4] = {0, 0, 1, 1};

	world_->LoadURDF2(
		st,
		btVector3(x,y,z),
		btQuaternion(btVector3(0,1,0),-1.57 * dir[face]),
		1.0f,
		true
	);
}


void Map::GenerateWall(const float x, const float y, const float z, const int face, const string st)
{
	const int dir[4] = {0, 0, 1, 1};

	world_->LoadURDF2(
		st,
		btVector3(x,y,z),
		btQuaternion(btVector3(0,1,0),-1.57 * dir[face]),
		1.0f,
		true
	);
}

void Map::GenerateFloor(const float x, const float y, const float z, const string& st)
{
	Robot * floor = world_->LoadURDF2(
		st,
		btVector3(x,y,z),
		btQuaternion(0,0,0,1),
		1.0f,
		true
	);
}


void Map::CreateSpawnOnFloor(const string name)
{
	if(!name.empty() &&
	 std::find(on_floor_list_.begin(), on_floor_list_.end(), name) == on_floor_list_.end())
	{
		on_floor_list_.push_back(name);
	}
}

void Map::CreateSpawnOnObject(const string name)
{
	if(!name.empty() &&
	 std::find(on_object_list_.begin(), on_object_list_.end(), name) == on_object_list_.end())
	{
		on_object_list_.push_back(name);
	}
}

void Map::CreateSpawnEither(const string name)
{
	if(!name.empty() &&
	 std::find(either_list_.begin(), either_list_.end(), name) == either_list_.end())
	{
		either_list_.push_back(name);
	}
}

void Map::CreateSectionWithSize(const float min_x, const float max_x, const float min_z, const float max_z)
{
	assert(min_x < max_x && min_z < max_z);

	AABB * bbox = new AABB(min_x, 0, min_z, max_x, kCeiling, max_z);

	sections_AABB_.push_back(bbox);
	sections_map_.push_back(std::make_pair(glm::vec3(bbox->minX, 0, bbox->minZ), glm::vec3(bbox->maxX, 2, bbox->maxZ)));

}

void Map::CreateSpawnConstraint(const string cannot_be_topped)
{
	if(!cannot_be_topped.empty() &&
	 cannot_be_topped_list.find(cannot_be_topped) == cannot_be_topped_list.end())
	{
		cannot_be_topped_list.insert(cannot_be_topped);
	}
}

void Map::CreateEmptyVolume(const float min_x, const float max_x, const float min_z, const float max_z)
{
	assert(min_x < max_x && min_z < max_z);

	AABB * bbox = new AABB(min_x, 0, min_z, max_x, kCeiling, max_z);

	first_layer_AABB_.push_back(bbox);
	first_layer_map_.push_back(-2);
	empty_map_.push_back(std::make_pair(glm::vec3(bbox->minX, 0, bbox->minZ), glm::vec3(bbox->maxX, 1, bbox->maxZ)));
}

void Map::CreateObjectAtTransform(
	const string name, 
	const float tx, const float ty, const float tz,
	const float rx, const float ry, const float rz, const float rw,
	const float s
)
{
	assert(!name.empty());

	Robot * robot = world_->LoadURDF2(
		name,
		btVector3(tx,ty,tz),
		btQuaternion(rx,ry,rz,rw),
		s
	);

	vec3 aabbMin, aabbMax;
	robot->root_part_->GetAABB(aabbMin, aabbMax);

	AABB * bbox = new AABB(aabbMin.x, aabbMin.y, aabbMin.z, aabbMax.x, aabbMax.y, aabbMax.z);
	if(aabbMin.y < kOnFloorThreshold)
	{
		// firstLayer.insertObject(bbox);
		first_layer_AABB_.push_back(bbox);
		first_layer_map_.push_back(-1);
	} 
	else 
	{
		// secondLayer.insertObject(bbox);
		//secondLayerAABB.push_back(bbox);
	}
}

void Map::ResetMap()
{
	delete map_;

	world_->CleanEverything();

	for (auto aabb : sections_AABB_)
	{
		delete aabb;
	}

	for (auto aabb : first_layer_AABB_)
	{
		delete aabb;
	}

	for (auto aabb : second_layer_AABB_)
	{
		delete aabb;
	}

	empty_map_.clear();
	sections_map_.clear();
	sections_AABB_.clear();
	first_layer_AABB_.clear();
	second_layer_AABB_.clear();
	first_layer_map_.clear();
}


void Map::ClearMap()
{
	delete map_;

	world_->CleanEverything2();

	for (auto aabb : sections_AABB_)
	{
		delete aabb;
	}

	for (auto aabb : first_layer_AABB_)
	{
		delete aabb;
	}

	for (auto aabb : second_layer_AABB_)
	{
		delete aabb;
	}

	empty_map_.clear();
	sections_map_.clear();
	sections_AABB_.clear();
	first_layer_AABB_.clear();
	second_layer_AABB_.clear();
	first_layer_map_.clear();
}

bool Map::Overlap(const std::vector<AABB *> aabbs, const AABB * other)
{
	for (const AABB * bbox : aabbs)
	{
		if(bbox->overlaps(*other)) return true;
	}

	return false;
}

float Map::GetRandom(const float low, const float high)
{
	std::uniform_real_distribution<float> dist(low, high);
	return dist(mt_);
}

void Map::SpawnOnFloor(const int num)
{
	if(!on_floor_list_.size()) return;

	for (int i = 0, success = 0; i < num << 1 && success < num; ++i)
	{

		int object_index = rand() % on_floor_list_.size();
		int room_index = rand() % sections_AABB_.size();
		const string object = on_floor_list_[object_index];
		const AABB * room_AABB = sections_AABB_[room_index];

		glm::vec3 rand_position_on_tile = glm::vec3(
			GetRandom(room_AABB->minX + kFirstLayerOffset, room_AABB->maxX - kFirstLayerOffset),
			0,
			GetRandom(room_AABB->minZ + kFirstLayerOffset, room_AABB->maxZ - kFirstLayerOffset)
		);

		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(btVector3(rand_position_on_tile.x, 0, rand_position_on_tile.z));

		Robot * robot = world_->LoadURDF2(
			object,
			transform.getOrigin(),
			transform.getRotation()
		);

		AABB * bbox = new AABB();

		bool find_sol = false;
		for(int t = 0; t < 15; ++t){

			transform.setOrigin(btVector3(rand_position_on_tile.x, 0, rand_position_on_tile.z));
			world_->SetTransformation(robot, transform);

			vec3 aabb_min, aabb_max;
			robot->root_part_->GetAABB(aabb_min, aabb_max);

			bbox->update(
				aabb_min.x, aabb_min.y, aabb_min.z,
			 	aabb_max.x, aabb_max.y, aabb_max.z
			 );

			if(!Overlap(first_layer_AABB_, bbox))
			{
				// firstLayer.insertObject(bbox);
				first_layer_AABB_.push_back(bbox);
				first_layer_map_.push_back(object_index);

				transform.setOrigin(btVector3(rand_position_on_tile.x,
					0 - aabb_min.y + kPadding, rand_position_on_tile.z));
				world_->SetTransformation(robot, transform);
				t = 100;
				find_sol = true;
				success++;
			} 

			rand_position_on_tile = glm::vec3(
				GetRandom(room_AABB->minX + kFirstLayerOffset, room_AABB->maxX - kFirstLayerOffset),
				0,
				GetRandom(room_AABB->minZ + kFirstLayerOffset, room_AABB->maxZ - kFirstLayerOffset)
			);
		}

		if(!find_sol)
		{
			world_->RemoveRobot2(robot);
			delete bbox;
		}

	}
}

void Map::SpawnOnObject(const int num)
{
	if(!on_object_list_.size()) return;

	for (int i = 0, success = 0; i < num << 1 && success < num; ++i)
	{
		int object_index = rand() % on_object_list_.size();
		int first_layer_index = rand() % first_layer_AABB_.size();
		const string object = on_object_list_[object_index];
		const AABB * object_AABB = first_layer_AABB_[first_layer_index];

		if(first_layer_map_[first_layer_index] < 0 ||
			cannot_be_topped_list.find(on_floor_list_[(int)first_layer_map_[first_layer_index]])
			!= cannot_be_topped_list.end())
		{
			continue;
		}


		glm::vec3 rand_position_on_object = glm::vec3(
			GetRandom(object_AABB->minX + kSecondLayerOffset, object_AABB->maxX - kSecondLayerOffset),
			0,
			GetRandom(object_AABB->minZ + kSecondLayerOffset, object_AABB->maxZ - kSecondLayerOffset)
		);

		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(btVector3(rand_position_on_object.x, 0, rand_position_on_object.z));

		Robot * robot = world_->LoadURDF2(
			object,
			transform.getOrigin(),
			transform.getRotation()
		);

		AABB * bbox = new AABB();

		bool find_sol = false;
		for(int t = 0; t < 15; ++t){

			transform.setOrigin(btVector3(rand_position_on_object.x, 0, rand_position_on_object.z));
			world_->SetTransformation(robot, transform);

			vec3 aabb_min, aabb_max;
			robot->root_part_->GetAABB(aabb_min, aabb_max);

			bbox->update(aabb_min.x, aabb_min.y, aabb_min.z, aabb_max.x, aabb_max.y, aabb_max.z);

			if(!Overlap(second_layer_AABB_, bbox))
			{
				// secondLayer.insertObject(bbox);
				second_layer_AABB_.push_back(bbox);

				transform.setOrigin(btVector3(rand_position_on_object.x, object_AABB->maxY - aabb_min.y + kPadding,
					rand_position_on_object.z));
				world_->SetTransformation(robot, transform);
				t = 100;
				find_sol = true;
				success++;
			}

			glm::vec3 rand_position_on_object = glm::vec3(
				GetRandom(object_AABB->minX + kSecondLayerOffset, object_AABB->maxX - kSecondLayerOffset),
				0,
				GetRandom(object_AABB->minZ + kSecondLayerOffset, object_AABB->maxZ - kSecondLayerOffset)
			);
		}

		if(!find_sol)
		{
			world_->RemoveRobot2(robot);
			delete bbox;
		}

	}
}

void Map::SpawnEither(const int n)
{
	if(!either_list_.size()) return;


	for (int i = 0, success = 0; i < n << 1 && success < n; ++i)
	{

		bool hasSpawnOnFloor = rand() % 2;

		if(hasSpawnOnFloor)
		{
			int objIdx = rand() % either_list_.size();

			const string object = either_list_[objIdx];

			int roomIdx = rand() % sections_AABB_.size();

			const AABB * rAABB = sections_AABB_[roomIdx];

			glm::vec3 randPos = glm::vec3(
				GetRandom(rAABB->minX + kFirstLayerOffset, rAABB->maxX - kFirstLayerOffset),
				0,
				GetRandom(rAABB->minZ + kFirstLayerOffset, rAABB->maxZ - kFirstLayerOffset)
			);

			btTransform transform;
			transform.setIdentity();
			transform.setOrigin(btVector3(randPos.x, 0, randPos.z));

			Robot * r = world_->LoadURDF2(
				object,
				transform.getOrigin(),
				transform.getRotation()
			);

			AABB * bbox = new AABB();

			bool find_sol = false;
			for(int t = 0; t < 15; ++t){

				transform.setOrigin(btVector3(randPos.x, 0, randPos.z));
				world_->SetTransformation(r, transform);

				vec3 aabbMin, aabbMax;
				r->root_part_->GetAABB(aabbMin, aabbMax);

				bbox->update(
					aabbMin.x, aabbMin.y, aabbMin.z,
				 	aabbMax.x, aabbMax.y, aabbMax.z
				 );

				if(!Overlap(first_layer_AABB_, bbox))
				{
					// firstLayer.insertObject(bbox);
					first_layer_AABB_.push_back(bbox);
					first_layer_map_.push_back(objIdx);

					transform.setOrigin(btVector3(randPos.x, 0 - aabbMin.y + kPadding, randPos.z));
					world_->SetTransformation(r, transform);
					t = 100;
					find_sol = true;
					success++;
				} 

				randPos = glm::vec3(
					GetRandom(rAABB->minX + kFirstLayerOffset, rAABB->maxX - kFirstLayerOffset),
					0,
					GetRandom(rAABB->minZ + kFirstLayerOffset, rAABB->maxZ - kFirstLayerOffset)
				);
			}

			if(!find_sol)
			{
				world_->RemoveRobot2(r);
				delete bbox;
			}
		}
		else
		{
			int objIdx = rand() % either_list_.size();

			int firstIdx = rand() % first_layer_AABB_.size();

			const string object = either_list_[objIdx];

			const AABB * rAABB = first_layer_AABB_[firstIdx];

			if(first_layer_map_[firstIdx] < 0 ||
			cannot_be_topped_list.find(on_floor_list_[(int)first_layer_map_[firstIdx]]) != cannot_be_topped_list.end())
			{
				continue;
			}


			glm::vec3 randPos = glm::vec3(
				GetRandom(rAABB->minX + kSecondLayerOffset, rAABB->maxX - kSecondLayerOffset),
				0,
				GetRandom(rAABB->minZ + kSecondLayerOffset, rAABB->maxZ - kSecondLayerOffset)
			);

			btTransform transform;
			transform.setIdentity();
			transform.setOrigin(btVector3(randPos.x, 0, randPos.z));

			Robot * r = world_->LoadURDF2(
				object,
				transform.getOrigin(),
				transform.getRotation()
			);

			AABB * bbox = new AABB();

			bool find_sol = false;
			for(int t = 0; t < 15; ++t){

				transform.setOrigin(btVector3(randPos.x, 0, randPos.z));
				world_->SetTransformation(r, transform);

				vec3 aabbMin, aabbMax;
				r->root_part_->GetAABB(aabbMin, aabbMax);

				bbox->update(aabbMin.x, aabbMin.y, aabbMin.z, aabbMax.x, aabbMax.y, aabbMax.z);

				if(rAABB->getXZArea() > bbox->getXZArea())
				{
					if(!Overlap(second_layer_AABB_, bbox))
					{
						// secondLayer.insertObject(bbox);
						second_layer_AABB_.push_back(bbox);

						transform.setOrigin(btVector3(randPos.x, rAABB->maxY - aabbMin.y + kPadding, randPos.z));
						world_->SetTransformation(r, transform);
						t = 100;
						find_sol = true;
						success++;
					}
				} else {
					t = 100;
				}

				glm::vec3 randPos = glm::vec3(
					GetRandom(rAABB->minX + kSecondLayerOffset, rAABB->maxX - kSecondLayerOffset),
					0,
					GetRandom(rAABB->minZ + kSecondLayerOffset, rAABB->maxZ - kSecondLayerOffset)
				);
			}

			if(!find_sol)
			{
				world_->RemoveRobot2(r);
				delete bbox;
			}
		}

		

	}

}

void Map::Spawn(const int onFloor, const int onObject, const int onEither)
{
	SpawnOnFloor(onFloor);

	SpawnOnObject(onObject);

	SpawnEither(onEither);
}

}