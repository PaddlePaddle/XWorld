#include "map_grid.h"

namespace xrobot
{

MapGrid::~MapGrid() {
	world_ = nullptr;
}

MapGrid::MapGrid() : wall_urdf_path_(""),
					 unlocked_door_path_(""),
					 locked_door_path_list_(0),
					 tile_urdf_list_(0),
					 key_path_list_(0),
					 object_path_list_(0),
					 pile_list_(0),
					 bbox_list_(0),
					 object_bbox_list_(0),
					 object_height_list_(0),
					 rooms_(0),
					 roomgroups_(0),
					 map_(nullptr),
					 doors_list_(0),
					 doors_map_(),
					 map_bounding_(),
					 agent_spawn_position_(vec2(0)),
					 tiles_(),
					 resolve_path_(false),
					 agent_spawn_(false),
					 rand_device_(),
					 mt_(rand_device_()),
					 Map() {
	world_ = std::make_shared<World>();
	world_->BulletInit(-9.81f, 0.01f);
}

// Loading Resources

void MapGrid::LoadWallURDF(const std::string& urdf_path) {
	assert(urdf_path.size());

	wall_urdf_path_ = urdf_path;
}

void MapGrid::LoadUnlockedDoorJSON(const std::string& json_path) {
	assert(json_path.size());

	unlocked_door_path_ = json_path;
}

void MapGrid::CreateAndLoadLockedDoorJSON(const std::string& json_path) {
	assert(json_path.size());
	assert(locked_door_path_list_.size() < 5); // Four Door-Key Pairs

	locked_door_path_list_.push_back(json_path);
}

void MapGrid::CreateAndLoadTileURDF(const std::string& urdf_path) {
	assert(urdf_path.size());

	tile_urdf_list_.push_back(urdf_path);
}

void MapGrid::CreateAndLoadObjectFILE(const std::string& file_path,
									  const std::string& tag) {
	assert(file_path.size() && tag.size());

	FileTag object_path_tag = {file_path, tag};
	object_path_list_.push_back(object_path_tag);

	auto obj = world_->LoadRobot(
		file_path,
		glm::vec3(0,0,0),
		glm::vec3(0,1,0),
        0.0,
		glm::vec3(1.0f, 1.0f, 1.0f),
		tag,
		true
	);

	glm::vec3 min_aabb, max_aabb;
	if(auto obj_sptr = obj.lock()) {
		obj_sptr->root_part_->GetAABB(min_aabb, max_aabb);
	
		BBox b(vec2(min_aabb.x, min_aabb.z), vec2(max_aabb.x, max_aabb.z));
		object_bbox_list_.push_back(b);
	
		object_height_list_.push_back(vec2(min_aabb.y, max_aabb.y));

		obj_sptr->recycle();
	}
}

void MapGrid::CreateAndLoadKeyURDF(const std::string& urdf_path,
								   const std::string& tag) {
	assert(urdf_path.size() && tag.size());

	FileTag key_path_tag = {urdf_path, tag};
	key_path_list_.push_back(key_path_tag);
}

// Scatter

inline float MapGrid::VanDerCorpus(unsigned int bits) {
	bits = (bits << 16u) | (bits >> 16u);
    bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
    bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
    bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
    bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
    return (float) bits * 2.3283064365386963e-10;
}

inline vec2 MapGrid::Hammersley(const unsigned int i, const unsigned int N) {
	return vec2((float) i / (float) N, VanDerCorpus(i));
}

// Generate Assets

void MapGrid::GenerateWall(const vec3 position, const int d) {
	const int dir[4] = {0, 0, 1, 1};

	auto obj = world_->LoadRobot(
		wall_urdf_path_,
		glm::vec3(position.x,position.y * kUniformScale,position.z),
		glm::vec3(0,1,0),
        -1.57 * dir[d],
		glm::vec3(1.0f, 1.0f, 1.0f) * kUniformScale,
		"Wall",
		true
	);

	glm::vec3 min_aabb, max_aabb;
	if(auto obj_sptr = obj.lock()) {
		obj_sptr->root_part_->GetAABB(min_aabb, max_aabb);
	}

	BBox b(vec2(min_aabb.x, min_aabb.z), vec2(max_aabb.x, max_aabb.z));
	bbox_list_.push_back(b);
}

void MapGrid::GenerateKey(const vec3 position, const int key_id) {
	const int dir[4] = {0, 0, 1, 1};

	if(key_path_list_[key_id].tag.size()) {
		world_->UpdatePickableList(key_path_list_[key_id].tag, true);
		auto obj = world_->LoadRobot(
			key_path_list_[key_id].file_path,
			glm::vec3(position.x,position.y,position.z),
			glm::vec3(0,1,0),
			0,
			glm::vec3(1.0f, 1.0f, 1.0f) * kUniformScale,
			key_path_list_[key_id].tag,
			true
		);

		glm::vec3 min_aabb, max_aabb;
		if(auto obj_sptr = obj.lock()) {
			obj_sptr->root_part_->GetAABB(min_aabb, max_aabb);
		}

		BBox b(vec2(min_aabb.x, min_aabb.z), vec2(max_aabb.x, max_aabb.z));
		bbox_list_.push_back(b);
	}
}

void MapGrid::GenerateTile(const vec3 position, const int tile_id) {
	world_->LoadRobot(
		tile_urdf_list_[tile_id],
		glm::vec3(position.x,position.y,position.z),
		glm::vec3(0,1,0),
		0,
		glm::vec3(1.0f, 1.0f, 1.0f) * kUniformScale,
		"Floor",
		true
	);
}

void MapGrid::GenerateLockedDoor(const vec3 position, const int d, const int door_id) {
	const int dir[4] = {0, 0, 1, 1};

	if(locked_door_path_list_[door_id].size()) {
		auto obj = world_->LoadRobot(
			locked_door_path_list_[door_id],
			glm::vec3(position.x,position.y * kUniformScale,position.z),
			glm::vec3(0,1,0),
            -1.57 * dir[d],
			glm::vec3(1.0f, 1.0f, 1.0f) * kUniformScale,
			"Door",
			true,0,false,true
		);

		glm::vec3 min_aabb, max_aabb;
		if(auto obj_sptr = obj.lock()) {
			obj_sptr->root_part_->GetAABB(min_aabb, max_aabb);
		}

		BBox b(vec2(min_aabb.x, min_aabb.z), vec2(max_aabb.x, max_aabb.z));
		bbox_list_.push_back(b);
	}
}

void MapGrid::GenerateUnlockedDoor(const vec3 position, const int d) {
	const int dir[4] = {0, 0, 1, 1};

	if(unlocked_door_path_.size()) {
		auto obj = world_->LoadRobot(
			unlocked_door_path_,
			glm::vec3(position.x,position.y * kUniformScale,position.z),
			glm::vec3(0,1,0),
            -1.57 * dir[d],
			glm::vec3(1.0f, 1.0f, 1.0f) * kUniformScale,
			"Door",
			true,0,false,true
		);

		glm::vec3 min_aabb, max_aabb;
		if(auto obj_sptr = obj.lock()) {
			obj_sptr->root_part_->GetAABB(min_aabb, max_aabb);
		}

		BBox b(vec2(min_aabb.x, min_aabb.z), vec2(max_aabb.x, max_aabb.z));
		bbox_list_.push_back(b);

	}
}

void MapGrid::GenerateObjects() {
	for(const Pile& pile : pile_list_) {
		for (int i = 0; i < pile.object_id_list.size(); ++i) {

			int object_id = pile.object_id_list[i];
			std::string file_path = object_path_list_[object_id].file_path;
			std::string tag = object_path_list_[object_id].tag;
			vec3 position = pile.positions[i];

			auto obj = world_->LoadRobot(
				file_path,
				glm::vec3(position.x,position.y,position.z),
				glm::vec3(0,1,0),
                0,
				glm::vec3(1.0f, 1.0f, 1.0f),
				tag,
				true
			);
		}
	}

	// Debug
	UpdateDebugVisualization();

	UpdateSceneBBox();
}

int MapGrid::FindObject(const std::string& path) {
	for(int i = 0; i < object_path_list_.size(); ++i) {
		if(object_path_list_[i].file_path == path)
			return i;
	}
	return -1;
}

void MapGrid::SpawnSingleObjectAt(const std::string& path, const vec3 position) {
	int obj_id = FindObject(path);
	assert(obj_id > -1);

	SpawnSingleObjectAt(obj_id, position);
}

bool MapGrid::SpawnSingleObject(const std::string& path, const int roomgroup_id) {
	int obj_id = FindObject(path);
	assert(obj_id > -1);

	return SpawnSingleObject(obj_id, roomgroup_id);
}

bool MapGrid::SpawnPairOfObjects(const std::string& left_path, 
						 		 const std::string& right_path, 
					     		 const int roomgroup_id) {
	int left_id = FindObject(left_path);
	int right_id = FindObject(right_path);
	assert(left_id > -1 && right_id > -1);

	return SpawnPairOfObjects(left_id, right_id, roomgroup_id);
}


bool MapGrid::SpawnStackOfObjects(const std::string& top_path, 
						 const std::string& bottom_path, 
					     const int num, const int roomgroup_id) {
	int top_id = FindObject(top_path);
	int bottom_id = FindObject(bottom_path);
	assert(top_id > -1 && bottom_id > -1);

	return SpawnStackOfObjects(top_id, bottom_id, num, roomgroup_id);
}

bool MapGrid::SpawnSingleObject(const int obj_id, const int roomgroup_id) {

	const int max_attemps = 8;

	int select_roomgroup = roomgroup_id;
	if(roomgroup_id < 0 || roomgroup_id >= roomgroups_.size()) 
		select_roomgroup = (int) GetRandom(0, roomgroups_.size());

	int num_attempts_in_room = 0;
	int n_rooms_in_group = roomgroups_[select_roomgroup].rooms.size();
	bool found = false;
	while(!found && num_attempts_in_room++ < max_attemps) {
		int select_room = (int) GetRandom(0, n_rooms_in_group);
		auto set_rooms = roomgroups_[select_roomgroup].rooms;
		std::vector<int> tmp(set_rooms.begin(), set_rooms.end());
		int select_room_id = tmp[select_room] - 1;
		auto tiles = rooms_[select_room_id].tiles;

		// printf("01\n");

		std::vector<std::shared_ptr<Tile>> tiles_copy(tiles);
		std::random_shuffle(tiles_copy.begin(), tiles_copy.end());

		std::shared_ptr<Tile> select_tile = nullptr;
		for(auto& tile : tiles_copy) {
			if(!tile->occupied) {
				select_tile = tile;
				break;
			}
		}

		// printf("02\n");

		if(!select_tile) 
			select_tile = tiles_copy[(int) GetRandom(0, tiles.size())];

		// printf("03\n");

		int num_attempts_in_tile = 0;
		bool found_empty_in_subtile = false;
		BBox object_bbox = object_bbox_list_[obj_id];
		while(!found_empty_in_subtile && num_attempts_in_tile++ < max_attemps) {
			int select_subtile   = (int) GetRandom(0, kNSubTile * kNSubTile);
			auto select_subtile_ptr = select_tile->subtiles[select_subtile];
			vec2 select_subtile_center = select_subtile_ptr->center;
			BBox object_bbox_offset = object_bbox.Offset(select_subtile_center);

			// printf("03\n");

			bool neighbor_clear = true;
			auto subtile_neighbors = GetSubTileNeighbors(select_subtile_ptr);
			for(const auto& neighbor : subtile_neighbors) {
				if(neighbor) {
					auto n_p_sptr = neighbor->parent.lock();
					if(n_p_sptr->roomgroup_id != select_roomgroup ||
					   neighbor->occupied) {
						neighbor_clear = false;
						break;
					} 
				}
			}

			// printf("04\n");

			if(!select_subtile_ptr->occupied && neighbor_clear) {
				if(IntersectWithBBox(object_bbox_offset)) {
					found = true;

					float height_min = object_height_list_[obj_id].x;

					vec3 point(select_subtile_center.x,
					           -height_min + kPadding,
					           select_subtile_center.y);

					select_subtile_ptr->occupied = true;
					select_tile->occupied = true;

					bbox_list_.push_back(object_bbox_offset);

					Pile p;
					p.object_id_list.push_back(obj_id);
					p.positions.push_back(point);
					p.bbox = object_bbox_offset;
					pile_list_.push_back(p);

					if(!resolve_path_) {
						auto& waypoints = roomgroups_[select_roomgroup].waypoints;
						waypoints.push_back(select_subtile_ptr);
					}

					return true;
				}
				found_empty_in_subtile = true;
			}
		}
	}

	return false;
}

vec2 MapGrid::GetASpaceNearPosition(const vec2 position, const float radius) {
	assert(radius > 0.0f);

	constexpr float sqrt2 = 1.41421f;
	constexpr float padding = 0.01f;

	const vec2 offset_raw[8] = { 
		vec2(0, 1),
		vec2(sqrt2, sqrt2),
		vec2(1, 0),
		vec2(sqrt2, -sqrt2),
		vec2(0, -1),
		vec2(-sqrt2, -sqrt2),
		vec2(-1, 0),
		vec2(-sqrt2, sqrt2)
	};
	std::vector<vec2> offset(offset_raw, offset_raw + sizeof(offset_raw) / sizeof(vec2));

	std::random_shuffle(offset.begin(), offset.end());

	// TODO
	// Use Bounding Sphere Instead Of AABB
	
	float bbox_half_width = radius / sqrt2;
	BBox b(
		vec2(-bbox_half_width, -bbox_half_width),
		vec2( bbox_half_width,  bbox_half_width)
	);

	vec2 res(-1000, -1000);

	for (int i = 0; i < 8; ++i) {

		vec2 offset_vector = position + ((radius + padding) * offset[i]);
		BBox b_offset = b.Offset(offset_vector);

		if(IntersectWithBBox(b_offset)) {
			res = offset_vector;
			break;
		}
	}

	return res;
}

bool MapGrid::SpawnPairOfObjects(const int left_id, const int right_id, 
							     const int roomgroup_id) {
	const int max_attemps_left = 4;
	const float min_distance = 2.0;
	const float max_distance = 4.0;
	const bool need_empty_space = true;

	int num_attempts_left = 0;
	bool found_left = false;
	while(num_attempts_left++ < max_attemps_left && !found_left) {
		found_left = SpawnSingleObject(left_id, roomgroup_id);
	}

	if(found_left) {
		auto pile = pile_list_.back();
		vec2 bbox_center = pile.bbox.Center();
		float bbox_bounding_sphere = pile.bbox.BoundingSphereRadius();
		float min_distance_w_sph = min_distance + bbox_bounding_sphere;
		float max_distance_w_sph = max_distance + bbox_bounding_sphere;

		int tile_room_id = -1;
		auto subtile_sptr = GetSubTileFromWorldPosition(bbox_center);
		if(subtile_sptr) {
			if(auto tile_sptr = subtile_sptr->parent.lock()) {
				tile_room_id = tile_sptr->room_id;
			} 
		}


		std::vector<std::shared_ptr<SubTile>> subtiles_candidates;
		if(tile_room_id > -1) {
			auto& tiles = rooms_[tile_room_id - 1].tiles;
			for(auto& tile : tiles) {
				for(auto& subtile : tile->subtiles) {
					float dist = glm::distance(subtile->center, bbox_center);
					if(!subtile->occupied && 
					   dist > min_distance_w_sph &&
					   dist < max_distance_w_sph) {
						subtiles_candidates.push_back(subtile);
					} else if(need_empty_space &&
							  dist <= min_distance_w_sph) {
						subtile->occupied = true;
						tile->occupied = true;
					}
				}
			}
		}

		std::random_shuffle(subtiles_candidates.begin(),
						    subtiles_candidates.end());

		BBox object_bbox = object_bbox_list_[right_id];

		int max_attemps_right = max(8, (int)(subtiles_candidates.size() * 0.4));
		int num_attempts_right = 0;
		bool found_right = false;
		while(subtiles_candidates.size() && 
			  num_attempts_right++ < max_attemps_right &&
			  !found_right) 
		{
			auto select_subtile_ptr = subtiles_candidates.front();
			vec2 select_subtile_center = select_subtile_ptr->center;
			BBox object_bbox_offset = object_bbox.Offset(select_subtile_center);
			if(IntersectWithBBox(object_bbox_offset)) {
				found_right = true;

				float height_min = object_height_list_[right_id].x;

				vec3 point(select_subtile_center.x,
				           -height_min + kPadding,
				           select_subtile_center.y);

				select_subtile_ptr->occupied = true;

				if(auto select_tile_sptr = select_subtile_ptr->parent.lock()) {
					select_tile_sptr->occupied = true;
				}

				bbox_list_.push_back(object_bbox_offset);

				Pile p;
				p.object_id_list.push_back(right_id);
				p.positions.push_back(point);
				p.bbox = object_bbox_offset;
				pile_list_.push_back(p);
			} else {
				subtiles_candidates.erase(subtiles_candidates.begin());
			}
		}
	}

	return false;
}

bool MapGrid::SpawnStackOfObjects(const int top_id, const int bottom_id, 
							      const int num, const int roomgroup_id) {
	
	const int max_attemps_bottom = 2;
	const int max_attemps_top = 2 << num;

	int num_attempts_bottom = 0;
	bool found_bottom = false;
	while(num_attempts_bottom++ < max_attemps_bottom && !found_bottom) {
		found_bottom = SpawnSingleObject(bottom_id, roomgroup_id);
	}

	if(found_bottom) {
		auto& pile = pile_list_.back();
		auto& bbox = pile.bbox;
		auto& top_bbox = object_bbox_list_[top_id];
		float height = object_height_list_[bottom_id].y - 
			object_height_list_[bottom_id].x + kPadding;
		float height_min = object_height_list_[top_id].x;

		for (int i = 0; i < num; ++i) {
			
			int num_attempts_top = 0;
			bool collide = false;
			while(num_attempts_top++ < max_attemps_top) {
				float x = GetRandom(bbox.min_aabb.x + 0.05f, 
									bbox.max_aabb.x - 0.05f);
				float y = GetRandom(bbox.min_aabb.y + 0.05f,
									bbox.max_aabb.y - 0.05f);

				BBox top_bbox_offset = top_bbox.Offset(vec2(x,y));

				for(const auto& box : pile.upper_bbox_list) {
					if(box.Intersect(top_bbox_offset)) {
						collide = true;
					}
				}

				if(!collide) {
					vec3 point(x, height - height_min + kPadding, y);
					pile.object_id_list.push_back(top_id);
					pile.positions.push_back(point);
					pile.upper_bbox_list.push_back(top_bbox_offset);
					break;
				}
			}
		}		
	}

	return false;
}

void MapGrid::SpawnSingleObjectAt(const int obj_id, const vec3 position) {
	vec2 position_2d(position.x, position.z);
	auto subtile_sptr = GetSubTileFromWorldPosition(position_2d);

	if(subtile_sptr) {
		vec2 select_subtile_center = subtile_sptr->center;
		BBox object_bbox = object_bbox_list_[obj_id];
		BBox object_bbox_offset = object_bbox.Offset(select_subtile_center);
		bbox_list_.push_back(object_bbox_offset);

		Pile p;
		p.object_id_list.push_back(obj_id);
		p.positions.push_back(position);
		p.bbox = object_bbox_offset;
		pile_list_.push_back(p);

		auto tile_sptr = subtile_sptr->parent.lock();

		if(!resolve_path_) {
			auto& waypoints = roomgroups_[tile_sptr->roomgroup_id].waypoints;
			waypoints.push_back(subtile_sptr);
		}

		subtile_sptr->occupied = true;;
		tile_sptr->occupied = true;
	}
}

void MapGrid::GenerateEmpty(const BBox box) {
	
	for (auto& room : rooms_) {
		for (int i = 0; i < room.tiles.size(); ++i) {
			std::shared_ptr<Tile> tile_ptr = room.tiles[i];
			BBox tile_bbox = tile_ptr->GetTileBBox();

			// Tile Intersect / Contains
			if(box.Intersect(tile_bbox) || box.Contains(tile_bbox)) {
				for (int j = 0; j < kNSubTile * kNSubTile; ++j) {
					std::shared_ptr<SubTile> subtile_ptr = tile_ptr->subtiles[j];
					BBox subtile_bbox = tile_ptr->GetSubTileBBox(j);

					if(box.Intersect(subtile_bbox) || box.Contains(subtile_bbox)) {
						subtile_ptr->occupied = true;
						tile_ptr->occupied = true;
					}
				}
			}
		}
	}

	UpdateDebugVisualization();
}

void MapGrid::UpdateSceneBBox() {
	BBox base = bbox_list_.front();
	for (const BBox& bbox : bbox_list_) {
		base = base.Union(bbox);
	}
	map_bounding_ = base;

	world_->set_world_size(map_bounding_.min_aabb.x,
						   map_bounding_.min_aabb.y,
						   map_bounding_.max_aabb.x,
						   map_bounding_.max_aabb.y);
}

bool MapGrid::IntersectWithBBox(const BBox& box) {
	for (const BBox& bbox : bbox_list_) {
		if(box.Intersect(bbox) || box.Contains(bbox)) 
			return false;
	}
	return true;
}

// Generate Layout
void MapGrid::GenerateArena(const int w, const int l) {
	assert(w > 0 && l > 0 && tile_urdf_list_.size() > 0);

	const int dirx[4] = {1, -1, 0, 0};
	const int diry[4] = {0, 0, 1, -1};

	rooms_.resize(1);
	roomgroups_.resize(1);

	std::unordered_set<int> rooms;
	rooms.insert(1);
	roomgroups_[0].rooms = rooms;

	Room room;

	for(int i = 0; i < w; ++i)
	for(int j = 0; j < l; ++j) {
		vec3 tile_center(kTileSize * i, 0, kTileSize * j);
		vec2 tile_center_2d(kTileSize * i, kTileSize * j);
		vec3 tile_bbox_min = tile_center - 0.5f * vec3(kTileSize, 0, kTileSize);
		vec3 tile_bbox_max = tile_center + 0.5f * vec3(kTileSize, 0, kTileSize);

		std::shared_ptr<Tile> tile = std::make_shared<Tile>(tile_urdf_list_[0], 
			tile_center_2d);

		for (int d = 0; d < 4; ++d) {
			int x = i + dirx[d];
			int y = j + diry[d];

			if(!(x >= 0 && x < w && y >= 0 && y < l)) {
				tile->has_wall[d] = true;
				GenerateWall(vec3(kTileSize * i + 0.5f * kTileSize * dirx[d],
								  1,
								  kTileSize * j + 0.5f * kTileSize * diry[d]), d);
			}
		}

		tile->room_id = 1; // start from 1
		tile->roomgroup_id = 0;
		tile->position = std::make_pair(i,j);
		tile->occupied = false;
		tile->UpdateSubTiles();
		tiles_[std::make_pair(i,j)] = tile;

		room.tiles.push_back(tile);
		GenerateTile(tile_center, 0);
	}

	rooms_[0] = room;

	UpdateSceneBBox();
}

std::shared_ptr<SubTile> 
MapGrid::GetSubTileFromWorldPosition(const vec2 position) {
	vec2 position_offset = position + vec2(kTileSize, kTileSize) * 0.5f;

	int tile_x = position_offset.x / kTileSize;
	int tile_y = position_offset.y / kTileSize;

	auto tile = tiles_[std::make_pair(tile_x, tile_y)];

	if(tile) {

		vec2 offset = position_offset - vec2(tile_x, tile_y) * kTileSize;

		int subtile_x = offset.x / (kTileSize / kNSubTile);
		int subtile_y = offset.y / (kTileSize / kNSubTile);

		auto subtile = tile->GetSubTile(subtile_x, subtile_y);

		return subtile;
	}

	return nullptr;
}

void MapGrid::ClearVisited() {
	for(auto& room : rooms_) {
		for(auto& tile : room.tiles) {
			tile->ClearVisited();
		}
	}
}

void MapGrid::ResolvePath() {

	// TEST
	std::vector<std::vector<vec2>> path(0);
	std::vector<std::vector<std::shared_ptr<SubTile>>> dests_map(roomgroups_.size());

	for (int i = 0; i < roomgroups_.size(); ++i) {

		std::vector<std::shared_ptr<SubTile>>& dests = dests_map[i];

		// Find Agent
		if(agent_spawn_) {
			auto agent_spawn_room_it = roomgroups_[i].rooms.find(1);
			if(agent_spawn_room_it != roomgroups_[i].rooms.end()) 
				dests.push_back(GetSubTileFromWorldPosition(agent_spawn_position_));
		}

		// Find Door
		for(auto& door : doors_list_) {
			auto edge = door.edge;
			auto door_spawn_room_it_0 = roomgroups_[i].rooms.find(edge.first);
			auto door_spawn_room_it_1 = roomgroups_[i].rooms.find(edge.second);
			
			bool door_spawn_room_0 = door_spawn_room_it_0 != roomgroups_[i].rooms.end();
			bool door_spawn_room_1 = door_spawn_room_it_1 != roomgroups_[i].rooms.end();

			if(door_spawn_room_0 || door_spawn_room_1) {

				vec2 door_position_2d = door.center;

				// Jitter the Center Position
				vec2 door_position_2d_0 = door_position_2d + vec2(0.05f,0.05f);
				vec2 door_position_2d_1 = door_position_2d - vec2(0.05f,0.05f);

				auto subtile_tmp_0 = GetSubTileFromWorldPosition(door_position_2d_0);
				auto subtile_tmp_1 = GetSubTileFromWorldPosition(door_position_2d_1);


				if(subtile_tmp_0 && subtile_tmp_0->parent.lock()->roomgroup_id == i) {
					auto it = std::find(dests.begin(), dests.end(), subtile_tmp_0);
					if(it == dests.end()) {
						dests.push_back(subtile_tmp_0);
					}
				}
				
				if(subtile_tmp_1 && subtile_tmp_1->parent.lock()->roomgroup_id == i) {
					auto it = std::find(dests.begin(), dests.end(), subtile_tmp_1);
					if(it == dests.end()) {
						dests.push_back(subtile_tmp_1);
					}
				}
				
			}
		}

		// Other Task Related Objects
		for(const auto& waypoint : roomgroups_[i].waypoints) {
			dests.push_back(waypoint);
		}
	}

	for (int i = 0; i < roomgroups_.size(); ++i) {

		std::vector<std::shared_ptr<SubTile>>& dests = dests_map[i];

		// Generate Path
		while(dests.size() > 1) {

			std::shared_ptr<SubTile> start = dests.back();
			dests.pop_back();
			std::shared_ptr<SubTile> end   = dests.back();
			dests.pop_back();
			
			// BFS
			ClearVisited();
			std::vector<std::shared_ptr<SubTile>> path;
			std::vector<std::shared_ptr<SubTile>> p;
			std::queue<std::vector<std::shared_ptr<SubTile>>> q;

			start->visited = true;
			p.push_back(start);
			q.push(p);

			while(!q.empty()) {
				std::vector<std::shared_ptr<SubTile>> vp = q.front();
				std::shared_ptr<SubTile> v  = vp.back();
				q.pop();

				if(v == end) 
					path = std::vector<std::shared_ptr<SubTile>>(vp);

				auto subtile_neighbors = GetSubTileNeighbors(v);

				std::random_shuffle(subtile_neighbors.begin(), 
									subtile_neighbors.end());

				for (auto& neighbor : subtile_neighbors) {
					if(neighbor && !neighbor->visited) {

						std::vector<std::shared_ptr<SubTile>> vp_tmp(vp);
						neighbor->visited = true;
						vp_tmp.push_back(neighbor);
						q.push(vp_tmp);
					}
				}
			}

			for (int i = 0; i < path.size(); ++i) {
				path[i]->occupied = true;
				path[i]->parent.lock()->occupied = true;
			}

			dests.push_back(start);
		}
	}

	resolve_path_ = true;

	UpdateDebugVisualization();
}


std::vector<std::shared_ptr<Tile>> 
MapGrid::GetTileNeighbors(const std::shared_ptr<Tile> tile) {

	const int dirx[4] = {1, -1, 0, 0};
	const int diry[4] = {0, 0, 1, -1};
	
	std::vector<std::shared_ptr<Tile>> neighbors(4); // +x -x +y -y

	if(tile) {
		std::pair<int,int> tile_position = tile->position;

		for (int i = 0; i < 4; ++i) {
			std::pair<int,int> neighbor_position(
				tile_position.first  + dirx[i],
				tile_position.second + diry[i]
			);

			std::shared_ptr<Tile> neighbor_tile = nullptr;
			if(tiles_.find(neighbor_position) != tiles_.end()) {
				neighbor_tile = tiles_[neighbor_position];
			}
			neighbors[i] = neighbor_tile;
		}
	}

	return neighbors;
}

std::vector<std::shared_ptr<SubTile>>
MapGrid::GetSubTileNeighbors(const std::shared_ptr<SubTile> subtile) {

	const int dirx[4] = {1, -1, 0, 0};
	const int diry[4] = {0, 0, 1, -1};

	std::vector<std::shared_ptr<SubTile>> neighbors(4); // +x -x +y -y

	if(subtile) {
		auto parent = subtile->parent.lock();
		auto parent_subtiles = parent->subtiles;
		auto it = std::find(parent_subtiles.begin(), parent_subtiles.end(), subtile);

		int found = -1;
		if(it != parent_subtiles.end()) {
			found = (int) (it - parent_subtiles.begin());

			// printf("found %d\n", found);

			vec2 subtile_position = parent->GetSubTileRelativePosition(found);

			for (int i = 0; i < 4; ++i) {
				vec2 neighbor_position(subtile_position.x + dirx[i],
									   subtile_position.y + diry[i]);

				std::shared_ptr<SubTile> neighbor_subtile = nullptr;
				if(neighbor_position.x > -1 && neighbor_position.x < kNSubTile &&
				   neighbor_position.y > -1 && neighbor_position.y < kNSubTile) {

					neighbor_subtile = parent->GetSubTile(neighbor_position.x,
														  neighbor_position.y);
				} else if(neighbor_position.x < 0){
					auto tile_n = GetTileNeighbors(parent)[1];
					if(tile_n && tile_n->roomgroup_id == parent->roomgroup_id) {
						// printf("gid: %d\n", tile_n->roomgroup_id);
						neighbor_subtile = tile_n->GetSubTile(
							kNSubTile - 1, neighbor_position.y);
					}
				} else if(neighbor_position.x == kNSubTile){
					auto tile_n = GetTileNeighbors(parent)[0];
					if(tile_n && tile_n->roomgroup_id == parent->roomgroup_id) {
						// printf("gid: %d\n", tile_n->roomgroup_id);
						neighbor_subtile = tile_n->GetSubTile(
							0, neighbor_position.y);
					}
				} else if(neighbor_position.y < 0){
					auto tile_n = GetTileNeighbors(parent)[3];
					if(tile_n && tile_n->roomgroup_id == parent->roomgroup_id) {
						// printf("gid: %d\n", tile_n->roomgroup_id);
						neighbor_subtile = tile_n->GetSubTile(
							neighbor_position.x, kNSubTile - 1);
					}
				} else if(neighbor_position.y == kNSubTile){
					auto tile_n = GetTileNeighbors(parent)[2];
					if(tile_n && tile_n->roomgroup_id == parent->roomgroup_id) {
						// printf("gid: %d\n", tile_n->roomgroup_id);
						neighbor_subtile = tile_n->GetSubTile(
							neighbor_position.x, 0);
					}
				}
				neighbors[i] = neighbor_subtile;
			}
		}
	}

	return neighbors;
}


vec3 MapGrid::GenerateLayout(const int w, const int l, const int n, const int d) {
	assert(w > 0 && l > 0 && n > 0 && d > 0);
	assert(locked_door_path_list_.size() >= d);
	assert(unlocked_door_path_.size());

	const int dirx[4] = {1, -1, 0, 0};
	const int diry[4] = {0, 0, 1, -1};

	// Generate Map
	map_ = std::make_shared<MapGenerator>(w, l, n, d, 8, 0.5);
	map_->generate();

	int** map_ptr = map_->M_;
	std::vector<std::pair<int,int>> edges = map_->edges_;
	std::unordered_map<int,int> pa = map_->pa_;
	std::unordered_map<int, std::unordered_set<int>> merge;
	std::vector<int> visit_sequence = map_->room_orders_;

	// Shuffle Doors
	std::vector<int> door_id_list;
	for (int i = 0; i < locked_door_path_list_.size(); ++i) 
		door_id_list.push_back(i);

	std::random_shuffle(door_id_list.begin(), door_id_list.end());

	// Get Room Groups
	int num_roomgroups = 0;
	for (auto& p : pa) {
		merge[p.second].insert(p.first);
		num_roomgroups = std::max(p.second, num_roomgroups);
	}

	bool agent_placed = false;
	vec3 agent_center;

	rooms_.resize(n);
	roomgroups_.resize(num_roomgroups);

	for (auto& group : merge) 
		roomgroups_[group.first - 1].rooms = group.second;

	for(int i = 0; i < w; ++i)
	for(int j = 0; j < l; ++j) {

		const int room_id = map_ptr[i][j];
		if(room_id > 0) {
			vec3 tile_center(kTileSize * i, 0, kTileSize * j);
			vec2 tile_center_2d(kTileSize * i, kTileSize * j);

			int tile_urdf_id = (room_id - 1) % (int) tile_urdf_list_.size();

			std::shared_ptr<Tile> tile = std::make_shared<Tile>(
				tile_urdf_list_[tile_urdf_id], vec2(tile_center.x, tile_center.z));
			tile->room_id = room_id;
			tile->position = std::make_pair(i,j);


			// Find Group ID
			for (auto& group : merge) {
				int group_id = group.first - 1;
				std::unordered_set<int> room_group = group.second;

				if(room_group.find(room_id) != room_group.end()) {
					tile->roomgroup_id = group_id;
					break;
				}
			}


			tile->UpdateSubTiles();
			rooms_[room_id - 1].tiles.push_back(tile);
			tiles_[std::make_pair(i,j)] = tile;

			if(!agent_placed && room_id == 1) {
				agent_placed = true;
				agent_spawn_ = true;
				agent_center = tile_center;
			}

			for (int d = 0; d < 4; ++d) {
				const int x = i + dirx[d];
				const int y = j + diry[d];

				if(x >= 0 && x < w && y >= 0 && y < l) {

					// Outside
					if(map_ptr[x][y] < 1) {
						tile->has_wall[d] = true;
						GenerateWall(vec3(kTileSize * i + 0.5f * kTileSize * dirx[d],
										  1,
										  kTileSize * j + 0.5f * kTileSize * diry[d]), d);
					}
					// Near Another Room 
					else if(map_ptr[x][y] != map_ptr[i][j]) {
						std::pair<int,int> edge01 = 
							std::make_pair(map_ptr[x][y], map_ptr[i][j]);
						std::pair<int,int> edge10 = 
							std::make_pair(map_ptr[i][j], map_ptr[x][y]);

						auto iter_door_between_rooms = doors_map_.find(edge01);
						auto iter_need_door01 = std::find(edges.begin(), edges.end(), edge01);
						auto iter_need_door10 = std::find(edges.begin(), edges.end(), edge10);

						bool door_between_rooms = iter_door_between_rooms != doors_map_.end();
						bool need_door01 = iter_need_door01 != edges.end();
						bool need_door10 = iter_need_door10 != edges.end();
					
						bool can_be_merged = false;
						for (auto& group : merge) {
							if(group.second.size() > 1 &&
							   group.second.find(map_ptr[x][y]) != group.second.end() && 
							   group.second.find(map_ptr[i][j]) != group.second.end()) {
								can_be_merged = true; 
								break;
							}
						}

						vec3 door_center(kTileSize * i + 0.5f * kTileSize * dirx[d],
										 1,
										 kTileSize * j + 0.5f * kTileSize * diry[d]);
						vec2 door_center_2d(door_center.x, door_center.z);
						vec2 door_bbox_min_2d = door_center_2d - 0.5f * vec2(kTileSize);
						vec2 door_bbox_max_2d = door_center_2d + 0.5f * vec2(kTileSize);

						// Locked Door
						if((need_door01 || need_door10) && !door_between_rooms) {

							doors_map_[edge01] = door_center;
							doors_map_[edge10] = door_center;

							int generate_door_id = door_id_list.back();
							door_id_list.pop_back();

							Door door;

							if(need_door01) 
								door = {edge01, door_center_2d, generate_door_id};
							else 
								door = {edge10, door_center_2d, generate_door_id};

							doors_list_.push_back(door);

							tile->has_wall[d] = true;

							GenerateLockedDoor(door_center, d, generate_door_id);
						}

						// UnLocked Door or Merge Two Room
						else if(can_be_merged && !door_between_rooms) {

							doors_map_[edge01] = door_center;
							doors_map_[edge10] = door_center;

							Door door = {edge01, door_center_2d, -2};
							doors_list_.push_back(door);

							// UnLocked Door
							if(GetRandom(0,1) > 0.5f) {
								tile->has_wall[d] = true;
								GenerateUnlockedDoor(door_center, d);
							}
						}

						// Wall
						else if(doors_map_[edge01] != door_center) {

							tile->has_wall[d] = true;

							GenerateWall(door_center, d);
						}
					}
				}
				else {
					tile->has_wall[d] = true;

					vec3 wall_center(kTileSize * i + 0.5f * kTileSize * dirx[d],
									 1,
									 kTileSize * j + 0.5f * kTileSize * diry[d]);

					GenerateWall(wall_center, d);
				}
			}

			GenerateTile(tile_center, tile_urdf_id);
		}
	}

	// Empty Space at Agent
	vec2 agent_center_2d(agent_center.x, agent_center.z);
	vec2 agent_bbox_min = agent_center_2d - 0.25f * vec2(kTileSize);
	vec2 agent_bbox_max = agent_center_2d + 0.25f * vec2(kTileSize);
	GenerateEmpty(BBox(agent_bbox_min,agent_bbox_max));
	agent_spawn_position_ = agent_center_2d;


	// Empty Space at Door Center
	for (auto door : doors_list_) {
		int room_0 = door.edge.first;
		int room_1 = door.edge.second;
		vec3 door_center = doors_map_[std::make_pair(room_0, room_1)];
		vec2 door_center_2d(door_center.x, door_center.z);
		vec2 door_bbox_min = door_center_2d - 0.21f * vec2(kTileSize);
		vec2 door_bbox_max = door_center_2d + 0.21f * vec2(kTileSize);
		GenerateEmpty(BBox(door_bbox_min,door_bbox_max));
	}

	// Visit
	for(auto& room : rooms_) 
		room.visit_sequence = -1;

	for(int i = 0; i < visit_sequence.size(); ++i) {
		rooms_[visit_sequence[i] - 1].visit_sequence = i;
	}


	// Generate Keys
	for (auto door : doors_list_) {
		if(door.door_id > -1) {

			auto it = std::find(visit_sequence.begin(), visit_sequence.end(),
				door.edge.second);
			int key_in_seq = (int) GetRandom(0, (int) (it - visit_sequence.begin()));
			int key_in_room = visit_sequence[key_in_seq] - 1;

			int rand_tile = 0, rand_subtile = 0;
			int num_attempts = 0;

			while(num_attempts++ < (kNSubTile * kNSubTile)) {
				rand_tile = (int) GetRandom(0, rooms_[key_in_room].tiles.size());
				auto tile_ptr = rooms_[key_in_room].tiles[rand_tile];

				int rand_subtile = (int) GetRandom(0, kNSubTile * kNSubTile);
				auto subtile_ptr = tile_ptr->subtiles[rand_subtile];

				if(subtile_ptr && !subtile_ptr->occupied) {
					vec2 key_pos = subtile_ptr->center;

					GenerateKey(vec3(key_pos.x, 0, key_pos.y), door.door_id);

					subtile_ptr->occupied = true;
					tile_ptr->occupied = true;
					tile_ptr->UpdateTile();
					break;
				}
			}
		}
	}

	UpdateSceneBBox();

	return agent_center;
}

// Clear

void MapGrid::ResetMap() {

	if(world_->reset_count_ % 1000) {
		world_->CleanEverything2();
	} else {
		world_->CleanEverything();
	}

	printf("[World] Reset %d\n", world_->reset_count_);

	agent_spawn_position_ = vec2();
	agent_spawn_ = false;
	resolve_path_ = false;
	map_ = nullptr;
	wall_urdf_path_ = "";
	unlocked_door_path_ = "";
	locked_door_path_list_.clear();
	tile_urdf_list_.clear();
	key_path_list_.clear();
	object_path_list_.clear();
	object_bbox_list_.clear();
	bbox_list_.clear();
	pile_list_.clear();
	object_height_list_.clear();

	tiles_.clear();
	rooms_.clear();
	roomgroups_.clear();
	doors_list_.clear();
	doors_map_.clear();
	pile_list_.clear();
}

// Utils

float MapGrid::GetRandom(const float low, const float high) {
	std::uniform_real_distribution<float> dist(low, high - 1e-4);
	return dist(mt_);
}

// Debug

void MapGrid::UpdateDebugVisualization() {
	world_->debug_subtiles_.clear();
	world_->debug_subtile_status_.clear();

	for (auto& room : rooms_) {
		for (int i = 0; i < room.tiles.size(); ++i) {
			std::shared_ptr<Tile> tile_ptr = room.tiles[i];

			// Tile Intersect / Contains
			if(tile_ptr) {
				for (int j = 0; j < kNSubTile * kNSubTile; ++j) {
					std::shared_ptr<SubTile> subtile_ptr = tile_ptr->subtiles[j];
					BBox subtile_bbox = tile_ptr->GetSubTileBBox(j);					

					if(tile_ptr->occupied) {
						world_->debug_subtiles_.push_back(
							std::make_pair(subtile_bbox.min_aabb, subtile_bbox.max_aabb));

						if(subtile_ptr->occupied) {
							world_->debug_subtile_status_.push_back(2);
						} else if(!subtile_ptr->occupied) {
							world_->debug_subtile_status_.push_back(0);
						}
					}

					// if(subtile_ptr && subtile_ptr->occupied) {
					// 	world_->debug_subtile_status_.push_back(2);
					// } else if(subtile_ptr && !subtile_ptr->occupied) {
					// 	world_->debug_subtile_status_.push_back(0);
					// }
				}
			}
		}
	}
}

}
