#ifndef MAP_H_
#define MAP_H_

#include <iostream>
#include <random>
#include <set>
#include <string>
#include <unordered_map>

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/matrix_decompose.hpp"

#include "world.h"
#include "AABB.h"
#include "utils.h"
#include "vendor/json.h"
#include "room_generator.cpp"

using namespace glm;

namespace xrobot
{
	constexpr float kCeiling = 10.0f;
	constexpr float kPadding = 0.01f;
	constexpr float kOnFloorThreshold = 0.1f;
	constexpr float kFirstLayerOffset = 0.5f;
	constexpr float kSecondLayerOffset = 0.2f;

	struct ObjectWithLabel
	{
		std::string path;
		std::string label;
	};

	struct SectionType
	{
		SectionType():
		empty(true), name(""), 
		urdf_floor(""), urdf_ceiling(""), urdf_wall(""), urdf_door("") {}

		SectionType(
			const bool empty,
			const std::string& urdf_floor,
			const std::string& urdf_wall,
			const std::string& urdf_door = "",
			const std::string& urdf_ceiling = "", 
			const std::string& name = ""
		):empty(empty), name(name),
		urdf_floor(urdf_floor),
		urdf_ceiling(urdf_ceiling),
		urdf_wall(urdf_wall),
		urdf_door(urdf_door) {} 

		SectionType& operator=( const SectionType& other ) {
			empty = other.empty;
			urdf_floor = other.urdf_floor;
			urdf_door = other.urdf_door;
			urdf_ceiling = other.urdf_ceiling;
			urdf_wall = other.urdf_wall;
			name = other.name;
			return *this;
		}

		bool empty;
		std::string name;
		std::string urdf_wall;
		std::string urdf_floor;
		std::string urdf_door;
		std::string urdf_ceiling;
	};


	class Map
	{
	public:
		Map();
		~Map();
		
		// Generate Floor Plan
		glm::vec3 GenerateFloorPlan(const int w, const int l);
		glm::vec3 GenerateTestFloorPlan(const int w, const int l);
		void ClearFloorPlan();
		void CreateSectionType(
			const std::string& floor,
			const std::string& ceiling = "",
			const std::string& door = ""
		);

		void CreateLabel(const std::string& path,
						 const std::string& label);
		std::string FindLabel(const std::string& path);
		
		void GenerateDoor(const float x, const float y, const float z, const int face, const std::string st);
		void GenerateWall(const float x, const float y, const float z, const int face, const std::string st);
		void GenerateFloor(const float x = 0, const float y = 0, const float z = 0, const std::string& st = "");
		void GenerateCeiling(const float x = 0, const float y = 0, const float z = 0, const std::string& st = "");

		// Create Section
		void CreateSectionWithSize(const float min_x, const float max_x, const float min_z, const float max_z);
		
		void Spawn(const int onFloor = 5, const int onObject = 5, const int onEither = 5);


		// Create Empty Placeholder
		void CreateEmptyVolume(const float min_x, const float max_x, const float min_z, const float max_z);
				
		// Create an Object
		void CreateObjectAtTransform(
			const std::string name, 
			const float tx, const float ty, const float tz,
			const float rx = 0, const float ry = 0, const float rz = 0, const float rw = 1,
			const float s = 1
		);

		// Clear Constraints
		void ClearRules();

		// Spawn an Object
		void CreateSpawnOnFloor(const std::string name);
		void CreateSpawnOnObject(const std::string name);
		void CreateSpawnEither(const std::string name);

		// Create a Constraint which an Object Cannot Be Topped
		void CreateSpawnConstraint(const std::string cannot_be_topped);

		// Reset Simulation
		void ResetMap();
		void ForceResetMap();


		// Labels
		std::map<std::string, std::string> labels_;

		// World
		World * world_;
		AABB * map_AABB_;

		// For AABB Debug Rendering
		std::vector<std::pair<vec3, vec3>> empty_map_; // Cannot Be Occupied Spaces
		std::vector<std::pair<vec3, vec3>> sections_map_; // Tiles (2m x 2m)

		// Map
		MapGenerator * map_;
		std::vector<SectionType> section_types_; // Defined Which Kind of Room Will Be Generated

	private:
		// Random
		std::random_device rand_device_;
		std::mt19937 mt_;

		// Random Spawn Object List
		std::vector<std::string> on_floor_list_;
		std::vector<std::string> on_object_list_;
		std::vector<std::string> either_list_;

		// Constraint
		std::set<std::string> cannot_be_topped_list;
		std::vector<int> first_layer_map_;

		std::vector<AABB *> sections_AABB_;
		Boardphase sections_;

		std::vector<AABB *> first_layer_AABB_;
		Boardphase first_layer_;

		std::vector<AABB *> second_layer_AABB_;
		Boardphase second_layer_;

		// Utils
		float GetRandom(const float low, const float high);
		bool Overlap(const std::vector<AABB *> aabbs, const AABB * other);
		void GetMapAABB();
		// Spawn
		void SpawnOnFloor(const int num);
		void SpawnOnObject(const int num);
		void SpawnEither(const int num);

	};
}

#endif // MAP_H_
