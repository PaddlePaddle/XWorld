#ifndef MAP_SUNCG_H_
#define MAP_SUNCG_H_

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

using namespace glm;

namespace xrobot
{

	enum SUNCGModelRemoveTypes {
		kRemoveDoor = 1,
		kRemoveStairs = 2,
	};

	class MapSuncg
	{
	public:
		MapSuncg();
		~MapSuncg();

		int GetJsonArrayEntry(Json::Value *&result, Json::Value *array,
				unsigned int k, int expected_type = -1);
		int GetJsonObjectMember(Json::Value *&result, Json::Value *object,
				const char *str, int expected_type = 0);
		void LoadJSON(const char * houseFile, const char * input_data_directory,
				const bool concave = false, const vec3 offset = vec3(-40, 0, -40));
		void LoadCategoryCSV(const char * metadataFile);
		void SetRemoveAll(const unsigned int remove);
		void SetRemoveRandomly(const bool remove) { remove_randomly_ = remove; }
		void ResetMap();
		void SetMapSize(const float min_x, const float min_z,
				const float max_x, const float max_z);

		World * world_;
		AABB * map_AABB_;
		std::unordered_map<int, std::string> map_bullet_label_;
		std::unordered_map<std::string, std::string> all_labels_;

	private:
		void RemoveRandomObject();
		void ReplaceRandomObject();
		void GetMapAABB();
		float GetRandom(const float low, const float high);

		bool remove_all_doors_;
		bool remove_all_stairs_;
		bool remove_randomly_;

		std::random_device rand_device_;
		std::mt19937 mt_;

		// std::vector<AABB *> sections_AABB_;
		// Boardphase sections_;

		// std::vector<AABB *> first_layer_AABB_;
		// Boardphase first_layer_;

		// std::vector<AABB *> second_layer_AABB_;
		// Boardphase second_layer_;
	};
}

#endif // MAP_SUNCG_H_
