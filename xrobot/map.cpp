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
map_(nullptr), map_AABB_(nullptr)
{
	world_ = new World();
	world_->BulletInit(-9.81f, 0.01f);
	srand(time(NULL));
}

void Map::CreateSectionType(const std::string& floor, const std::string& wall, const std::string& door)
{
	assert(!floor.empty());

	SectionType st(false, floor, wall, door);

	section_types_.push_back(st);
}

glm::vec3 Map::GenerateTestFloorPlan(const int w, const int l)
{
	assert(section_types_.size() > 0);

	glm::vec3 random_center;

	bool first = true;

	// Convert to AABBs
	for(int i = 0; i < w; ++i)
	for(int j = 0; j < l; ++j)
	{
		const int id = 1;
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
					// Do Nothing
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

	GetMapAABB();

	return random_center;
}

glm::vec3 Map::GenerateFloorPlan(const int w, const int l)
{
	assert(section_types_.size() > 0);

	glm::vec3 random_center;

	map_ = new MapGenerator(mt_, w, l, section_types_.size(), 12, 0.6);
	map_->generate();

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
						glm::vec3 minAABB = door_center - glm::vec3(1.0, 0, 1.0);
						glm::vec3 maxAABB = door_center + glm::vec3(1.0, 0, 1.0);

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

	GetMapAABB();
	return random_center;
}

void Map::GenerateCeiling(const float x, const float y, const float z, const std::string& st)
{

}

void Map::GenerateDoor(const float x, const float y, const float z, const int face, const std::string st)
{
	const int dir[4] = {0, 0, 1, 1};

	RobotBase * robot = world_->LoadRobot(
		st,
		btVector3(x,y,z),
		btQuaternion(btVector3(0,1,0),-1.57 * dir[face]),
		btVector3(1.0f, 1.0f, 1.0f),
		"Door",
		true
	);
}


void Map::GenerateWall(const float x, const float y, const float z, const int face, const std::string st)
{
	const int dir[4] = {0, 0, 1, 1};

	RobotBase * robot = world_->LoadRobot(
		st,
		btVector3(x,y,z),
		btQuaternion(btVector3(0,1,0),-1.57 * dir[face]),
		btVector3(1.0f, 1.0f, 1.0f),
		"Wall",
		true
	);
}

void Map::GenerateFloor(const float x, const float y, const float z, const std::string& st)
{
	RobotBase * robot = world_->LoadRobot(
		st,
		btVector3(x,y,z),
		btQuaternion(0,0,0,1),
		btVector3(1.0f, 1.0f, 1.0f),
		"Floor",
		true
	);
}

std::string Map::FindLabel(const std::string& path)
{
	assert(!path.empty());
	if(labels_.find(path) != labels_.end())
		return labels_[path];
	else
		return "not_found";
}

void Map::CreateLabel(const std::string& path,
		const std::string& label)
{
	if(!label.empty())
		labels_[path] = label;
	else
		labels_[path] = "unlabeled";
}

void Map::ClearRules()
{
	on_floor_list_.clear();
	on_object_list_.clear();
	either_list_.clear();
	cannot_be_topped_list.clear();
	empty_map_.clear();
	section_types_.clear();
}

void Map::CreateSpawnOnFloor(const std::string name)
{
	if(!name.empty() &&
	 std::find(on_floor_list_.begin(), on_floor_list_.end(), name) == on_floor_list_.end())
	{
		on_floor_list_.push_back(name);
	}
}

void Map::CreateSpawnOnObject(const std::string name)
{
	if(!name.empty() &&
	 std::find(on_object_list_.begin(), on_object_list_.end(), name) == on_object_list_.end())
	{
		on_object_list_.push_back(name);
	}
}

void Map::CreateSpawnEither(const std::string name)
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

void Map::CreateSpawnConstraint(const std::string cannot_be_topped)
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
	const std::string name, 
	const float tx, const float ty, const float tz,
	const float rx, const float ry, const float rz, const float rw,
	const float s
)
{
	assert(!name.empty());

	RobotBase * robot = world_->LoadRobot(
		name,
		btVector3(tx,ty,tz),
		btQuaternion(rx,ry,rz,rw),
		btVector3(s,s,s),
		"Unnamed_Object"
	);

	vec3 aabbMin, aabbMax;
	robot->robot_data_.root_part_->GetAABB(aabbMin, aabbMax);

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

void Map::ForceResetMap()
{
	delete map_;

	if(map_AABB_)
		delete map_AABB_;

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

void Map::ResetMap()
{
	delete map_;

	if(map_AABB_)
		delete map_AABB_;

	if(world_->reset_count_ % 1000) {
		world_->CleanEverything2();
	} else {
		world_->CleanEverything();
	}

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

		int object_index = (int) GetRandom(0, on_floor_list_.size());
		int room_index = (int) GetRandom(0, sections_AABB_.size());
		const std::string object = on_floor_list_[object_index];
		const AABB * room_AABB = sections_AABB_[room_index];

		glm::vec3 rand_position_on_tile = glm::vec3(
			GetRandom(room_AABB->minX + kFirstLayerOffset, room_AABB->maxX - kFirstLayerOffset),
			0,
			GetRandom(room_AABB->minZ + kFirstLayerOffset, room_AABB->maxZ - kFirstLayerOffset)
		);

		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(btVector3(rand_position_on_tile.x, 0, rand_position_on_tile.z));

		RobotBase * robot = world_->LoadRobot(
			object,
			transform.getOrigin(),
			transform.getRotation(),
			btVector3(1.0f, 1.0f, 1.0f),
			FindLabel(object)
		);

		AABB * bbox = new AABB();

		bool find_sol = false;
		for(int t = 0; t < 15; ++t){

			transform.setOrigin(btVector3(rand_position_on_tile.x, 0, rand_position_on_tile.z));
			world_->SetTransformation(robot, transform);
			//world_->BulletStep();

			vec3 aabb_min, aabb_max;
			robot->robot_data_.root_part_->GetAABB(aabb_min, aabb_max);

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
			robot->RemoveRobotTemp();
			delete bbox;
		}

	}
}

void Map::SpawnOnObject(const int num)
{
	if(!on_object_list_.size()) return;

	for (int i = 0, success = 0; i < num << 1 && success < num; ++i)
	{
		int object_index = (int) GetRandom(0, on_object_list_.size());
		int first_layer_index = (int) GetRandom(0, first_layer_AABB_.size());
		const std::string object = on_object_list_[object_index];
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

		RobotBase * robot = world_->LoadRobot(
			object,
			transform.getOrigin(),
			transform.getRotation(),
			btVector3(1.0f, 1.0f, 1.0f),
			FindLabel(object)
		);

		AABB * bbox = new AABB();

		bool find_sol = false;
		for(int t = 0; t < 15; ++t){

			transform.setOrigin(btVector3(rand_position_on_object.x, 0, rand_position_on_object.z));
			world_->SetTransformation(robot, transform);

			vec3 aabb_min, aabb_max;
			robot->robot_data_.root_part_->GetAABB(aabb_min, aabb_max);

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
			robot->RemoveRobotTemp();
			delete bbox;
		}

	}
}

void Map::SpawnEither(const int n)
{
	if(!either_list_.size()) return;


	for (int i = 0, success = 0; i < n << 1 && success < n; ++i)
	{

		bool hasSpawnOnFloor = (bool) GetRandom(0, 2);

		if(hasSpawnOnFloor)
		{
			int objIdx = (int) GetRandom(0, either_list_.size());

			const std::string object = either_list_[objIdx];

			int roomIdx = (int) GetRandom(0, sections_AABB_.size());

			const AABB * rAABB = sections_AABB_[roomIdx];

			glm::vec3 randPos = glm::vec3(
				GetRandom(rAABB->minX + kFirstLayerOffset, rAABB->maxX - kFirstLayerOffset),
				0,
				GetRandom(rAABB->minZ + kFirstLayerOffset, rAABB->maxZ - kFirstLayerOffset)
			);

			btTransform transform;
			transform.setIdentity();
			transform.setOrigin(btVector3(randPos.x, 0, randPos.z));

			RobotBase * r = world_->LoadRobot(
				object,
				transform.getOrigin(),
				transform.getRotation(),
				btVector3(1.0f, 1.0f, 1.0f),
				FindLabel(object)
			);

			AABB * bbox = new AABB();

			bool find_sol = false;
			for(int t = 0; t < 15; ++t){

				transform.setOrigin(btVector3(randPos.x, 0, randPos.z));
				world_->SetTransformation(r, transform);

				vec3 aabbMin, aabbMax;
				r->robot_data_.root_part_->GetAABB(aabbMin, aabbMax);

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
				r->RemoveRobotTemp();
				delete bbox;
			}
		}
		else
		{
			int objIdx = (int) GetRandom(0, either_list_.size());

			int firstIdx = (int) GetRandom(0, first_layer_AABB_.size());

			const std::string object = either_list_[objIdx];

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

			RobotBase * r =  world_->LoadRobot(
				object,
				transform.getOrigin(),
				transform.getRotation(),
				btVector3(1.0f, 1.0f, 1.0f),
				FindLabel(object)
			);

			AABB * bbox = new AABB();

			bool find_sol = false;
			for(int t = 0; t < 15; ++t){

				transform.setOrigin(btVector3(randPos.x, 0, randPos.z));
				world_->SetTransformation(r, transform);

				vec3 aabbMin, aabbMax;
				r->robot_data_.root_part_->GetAABB(aabbMin, aabbMax);

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
				r->RemoveRobotTemp();
				delete bbox;
			}
		}

		

	}

}

void Map::GetMapAABB()
{
	AABB temp;

	for (const AABB * bbox : sections_AABB_)
	{
		temp = temp.merge(*bbox);
	}

	map_AABB_ = new AABB(temp.minX, temp.minY, temp.minZ,
							  temp.maxX, temp.maxY, temp.maxZ);

	world_->set_world_size(temp.minX, temp.minZ, temp.maxX, temp.maxZ);
}

void Map::Spawn(const int onFloor, const int onObject, const int onEither)
{
	SpawnOnFloor(onFloor);

	SpawnOnObject(onObject);

	SpawnEither(onEither);
}

}
