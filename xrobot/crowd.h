#ifndef CROWD_H_
#define CROWD_H_

#include <unordered_map>
#include <vector>
#include <algorithm>

#include "glm/glm.hpp"
#include "bullet/LinearMath/btTransform.h"

#include "gl_context.h"
#include "shader.h"
#include "render_world.h"

namespace xrobot {
	
	class RobotBase;
	class World;

	inline float DepthLogToLinear (const float depth, const float near, const float far) 
	{
		return 2.0f * near * far / (far + near - (2.0f * depth - 1.0f) * (far - near));
	}

	struct Agent {
		RobotBase* object;
	};

	struct Node {
		bool block;
		glm::vec2 world_position;
		int x, y;
		int g_cost, h_cost;
		Node* parent;

		Node() : block(false),
				 world_position(glm::vec3(0,0,0)),
				 x(0),
				 y(0),
				 g_cost(0),
				 h_cost(0),
				 parent(nullptr) {}
		int FCost() { return g_cost + h_cost; }
	};

	class Grid {
	public:
		Grid(const int width, const int length);

		void Visualize();
		std::vector<Node*> GetNeighbours(Node* node);
		Node* GetNodeFromWorldPosition(const glm::vec2 position);

		int width_, length_;
		glm::vec2 grid_size_;
		glm::vec2 world_min_;
		std::vector<Node*> data_;
	};

	class Crowd {
	public:
		Crowd(
			render_engine::GLContext * ctx,
			World * world,
			const unsigned int width = 16,
			const unsigned int length = 16);
		~Crowd();


		void SpawnAgent(const glm::vec3 position);
		void BakeNavMesh();
		void SetBakeArea(
			const glm::vec3 world_min,
			const glm::vec3 world_max);

		void SetSurfaceLevel(const float level) { surface_level_ = level; }
		Grid GetGrid() { return grid_map_; }

	private:
		void Voxelization();

		float surface_level_;
		glm::vec3 world_min_, world_max_;
		unsigned int width_, length_;
		World * world_;
		xrobot::render_engine::Shader depth_shader_;
		GLuint fbo_;
		GLuint grid_texture_;
		render_engine::GLContext * ctx_;
		Grid grid_map_;
		std::vector<float> depth_map_;
		std::vector<Agent> crowd_;
	};
}

#endif //CROWD_H_