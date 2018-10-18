#ifndef MAP_H_
#define MAP_H_

#include <iostream>
#include <random>
#include <set>
#include <string>
#include <unordered_map>
#include <memory>

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/matrix_decompose.hpp"

#include "world.h"
#include "AABB.h"
#include "utils.h"
#include "vendor/json.h"

using namespace glm;

namespace xrobot
{

	class Map
	{
	public:
		Map() : world_(nullptr) {}
		virtual ~Map() {}
		virtual void ResetMap() =0;

		std::shared_ptr<World> world_;
	};

}

#endif // MAP_H_
