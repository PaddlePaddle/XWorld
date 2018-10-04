#include "map_suncg.h"

namespace xrobot
{

MapSuncg::~MapSuncg()
{
	delete world_;
}

MapSuncg::MapSuncg() : world_(nullptr),
					   map_AABB_(nullptr),
					   all_labels_(),
					   map_bullet_label_(),
					   rand_device_(),
					   mt_(rand_device_()),
					   remove_all_doors_(false),
					   remove_all_stairs_(false),
					   remove_randomly_(false),
					   map_labels_properity()
{
	world_ = new World();
	world_->BulletInit(-9.81f, 0.01f);
}

// This function is adapted from SUNCG!
int MapSuncg::GetJsonArrayEntry(Json::Value *&result, Json::Value *array, unsigned int k, int expected_type)
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
int MapSuncg::GetJsonObjectMember(Json::Value *&result, Json::Value *object, const char *str, int expected_type)
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
void MapSuncg::LoadJSON(const char * houseFile, const char * input_data_directory, const bool concave,
		const vec3 offset)
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

				rotation = conjugate(rotation);
				translation += offset;

				// Create scene node(s) based on type
				char obj_name[4096], node_name[4096];
				if (!strcmp(node_type, "Ground")) {
					sprintf(obj_name, "%s/room/%s/%sf.obj", input_data_directory, scene_id, modelId); 
					if (!hideFloor) {

						RobotBase * object = world_->LoadRobot(
							obj_name,
							btVector3(translation.x,translation.y,translation.z),
							btQuaternion(rotation.x,rotation.y,rotation.z,rotation.w),
							btVector3(scale.x, scale.y, scale.z),
							"Floor",
							true,
							0,
							isMirrored == 1 ? -1.0f : 1.0f
						);
						map_bullet_label_[object->robot_data_.bullet_handle_] = "Floor";
					}
				}
				else if(!strcmp(node_type, "Room"))
				{
					// Create node for floor
					sprintf(obj_name, "%s/room/%s/%sf.obj", input_data_directory, scene_id, modelId); 
					if (!hideFloor) {

						RobotBase * object = world_->LoadRobot(
							obj_name,
							btVector3(translation.x,translation.y,translation.z),
							btQuaternion(rotation.x,rotation.y,rotation.z,rotation.w),
							btVector3(scale.x, scale.y, scale.z),
							"Floor",
							true,
							0,
							isMirrored == 1 ? -1.0f : 1.0f
						);
						map_bullet_label_[object->robot_data_.bullet_handle_] = "Floor";
					}


					// Create node for walls
					sprintf(obj_name, "%s/room/%s/%sw.obj", input_data_directory, scene_id, modelId); 
					if (!hideWalls) {

						RobotBase * object = world_->LoadRobot(
							obj_name,
							btVector3(translation.x,translation.y,translation.z),
							btQuaternion(rotation.x,rotation.y,rotation.z,rotation.w),
							btVector3(scale.x, scale.y, scale.z),
							"Wall",
							true,
							0,
							isMirrored == 1 ? -1.0f : 1.0f,
							concave
						);
						map_bullet_label_[object->robot_data_.bullet_handle_] = "Wall";
					}

					sprintf(obj_name, "%s/room/%s/%sc.obj", input_data_directory, scene_id, modelId); 
					if (!hideCeiling) {
						//printf("Load Wall: %s\n", obj_name);
						// createObjectAtTransform(obj_name,
						// 	translation.x, translation.y, translation.z,
						// 	rotation.x, rotation.y, rotation.z, rotation.w
						// 	//glm::max(glm::max(scale.x, scale.y), scale.z)
						// );
						RobotBase * object = world_->LoadRobot(
							obj_name,
							btVector3(translation.x,translation.y,translation.z),
							btQuaternion(rotation.x,rotation.y,rotation.z,rotation.w),
							btVector3(scale.x, scale.y , scale.z),
							"Ceiling",
							true,
							0,
							isMirrored == 1 ? -1.0f : 1.0f,
							concave
						);
						map_bullet_label_[object->robot_data_.bullet_handle_] = "Ceiling";
					}
				}
				else if(!strcmp(node_type, "Object"))
				{
					if (state) sprintf(obj_name, "%s/object/%s/%s_0.obj", input_data_directory, modelId, modelId); 
					else sprintf(obj_name, "%s/object/%s/%s.obj", input_data_directory, modelId, modelId); 
					
					float mass = 0.0f;
					bool concave = false;

					if (remove_all_doors_ && all_labels_[modelId] == "door") {
						continue;
					}

					if (remove_all_stairs_ && all_labels_[modelId] == "stairs") {
						continue;
					}

					if (all_labels_[modelId] == "window") {
						//continue;
					}

					// if (all_labels_[modelId] == "window") {
					// 	mass = 0.0f;
					// }

					if(map_labels_properity.find(all_labels_[modelId]) != map_labels_properity.end()) {
						mass = map_labels_properity[all_labels_[modelId]].mass;
						concave = map_labels_properity[all_labels_[modelId]].concave;
					}

					// if(all_labels_[modelId] == "chair") {
					// 	mass = 100.0f;
					// 	concave = false;
					// }

					// if(all_labels_[modelId] == "fruit_bowl") {
					// 	mass = 100.0f;
					// 	concave = false;
					// }

					// if(all_labels_[modelId] == "trash_can") {
					// 	mass = 100.0f;
					// 	concave = false;
					// }

					// if(all_labels_[modelId] == "coffee_machine") {
					// 	mass = 100.0f;
					// 	concave = false;
					// }
					// if (remove_randomly_  && GetRandom(0, 5) > 2) {
					// 	if(all_labels_[modelId] != "window") {
					// 		continue;
					// 	}
					// }


					RobotBase * object = world_->LoadRobot(
						obj_name,
						btVector3(translation.x,translation.y,translation.z),
						btQuaternion(rotation.x,rotation.y,rotation.z,rotation.w),
						btVector3(scale.x, scale.y, scale.z),
						all_labels_[modelId],
						true,
						mass,
						isMirrored == 1 ? -1.0f : 1.0f,
						concave
					);

					object->robot_data_.root_part_->ChangeLinearDamping(0.9f);
					object->robot_data_.root_part_->ChangeAngularDamping(0.9f);

					map_bullet_label_[object->robot_data_.bullet_handle_] = all_labels_[modelId];
					//printf("    label: %s\n", all_labels_[modelId].c_str());
				}

			}
		}
	}

}


// This function is adapted from SUNCG!
void MapSuncg::LoadCategoryCSV(const char * metadataFile)
{
	FILE *fp = fopen(metadataFile, "r");
	if (!fp) {
	    fprintf(stderr, "Unable to open metadata file %s\n", metadataFile);
	    return;
	}

	int line_number = 1;
	char key_buffer[4096];
	std::vector<char *> keys;
	if (fgets(key_buffer, 4096, fp)) {
	    char *token = strtok(key_buffer, ",\n");
	    while (token) {
	    	keys.push_back(token);
	    	token = strtok(NULL, ",\n");
	    }
	}

	int model_id_k = -1;
  	for (int i = 0; i < keys.size(); i++) {
	    if (!strcmp(keys[i], "model_id")) {
	    	model_id_k = i;
	    	break;
	    }
	}

	if (model_id_k < 0) {
	    fprintf(stderr, "Did not find \"model_id\" in header on line %d of %s\n",
	    		line_number, metadataFile);
	    return;
	}

	int fine_grained_class_k = -1;
  	for (int i = 0; i < keys.size(); i++) {
	    if (!strcmp(keys[i], "fine_grained_class")) {
	    	fine_grained_class_k = i;
	    	break;
	    }
	}

	if (fine_grained_class_k < 0) {
	    fprintf(stderr, "Did not find \"fine_grained_class\" in header on line %d of %s\n",
	    		line_number, metadataFile);
	    return;
	}

	char value_buffer[4096];
	while (fgets(value_buffer, 4096, fp)) {
		line_number++;

		std::vector<char *> values;
		char *token = strtok(value_buffer, ",\n");
		while (token) {
			values.push_back(token);
			token = strtok(NULL, ",\n");
		}

		if (values.size() == 0) continue;
	    if (values.size() != keys.size()) {
			fprintf(stderr, "Invalid number of entries at line %d in %s\n",
					line_number, metadataFile);
			return;
		}

		const char *model_id = values[model_id_k];
	    if (!model_id) continue;
	    int model_id_length = strlen(model_id);
		if (model_id_length == 0) continue;

		const char *fine_grained_class = values[fine_grained_class_k];
	    if (!fine_grained_class) continue;
	    int fine_grained_class_length = strlen(fine_grained_class);
		if (fine_grained_class_length == 0) continue;

		//printf("%s %s\n", model_id, fine_grained_class);

		all_labels_.insert(std::pair<std::string, std::string>
				(std::string(model_id), std::string(fine_grained_class)));
	}
}

void MapSuncg::ResetMap()
{
	world_->CleanEverything();
	map_bullet_label_.clear();
	all_labels_.clear();
	map_labels_properity.clear();
	remove_all_doors_ = false;
	remove_all_stairs_ = false;
	remove_randomly_ = false;
}

void MapSuncg::AddPhysicalProperties(const std::string& label, const Properity& prop)
{
	map_labels_properity[label] = prop;
}

void MapSuncg::SetRemoveAll(const unsigned int remove)
{
	if (remove == 1) {
		remove_all_doors_ = true;
	} else if (remove == 2) {
		remove_all_stairs_ = true;
	} else if (remove == 3) {
		remove_all_doors_ = true;
		remove_all_stairs_ = true;
	}
}

float MapSuncg::GetRandom(const float low, const float high)
{
	std::uniform_real_distribution<float> dist(low, high);
	return dist(mt_);
}

void MapSuncg::SetMapSize(const float min_x, const float min_z,
				const float max_x, const float max_z)
{
	world_->set_world_size(min_x, min_z, max_x, max_z);
}

void MapSuncg::GetMapAABB()
{

}

}