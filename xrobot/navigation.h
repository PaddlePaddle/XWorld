#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <queue>
#include <algorithm>
#include <memory>
#include <thread>

#include "glm/glm.hpp"
#include "bullet/LinearMath/btTransform.h"

#include "gl_context.h"
#include "shader.h"
#include "render_world.h"

namespace xrobot {
	
	class RobotBase;
	class World;

	struct Node {
		bool carve;
		bool block;
		glm::vec2 world_position;
		float angle;
		int x, y;
		int g_cost, h_cost;
		std::shared_ptr<Node> parent;

		Node() : block(false),
				 carve(false),
				 world_position(glm::vec3(0,0,0)),
				 x(0),
				 y(0),
				 g_cost(0),
				 h_cost(0),
				 parent(nullptr) {}
		int FCost() { return g_cost + h_cost; }
	};

	struct CarveShape {
		glm::vec2 position;
		float radius;
		bool moving;
	};

	class Agent {
	public:
		Agent();

		std::weak_ptr<RobotBase> robot_;
		bool request_update_;
		int target_index_;
		float speed_;
		float distance_;
		float angle_;
		glm::vec3 last_direction_;
		glm::vec3 current_position_;
		glm::vec3 target_position_;
		std::vector<std::shared_ptr<Node>> path_;

		void AssignTarget(const glm::vec3 position);
		void FollowPath();
	};

	class Grid {
	public:
		Grid() {};
		Grid(const int width, const int length);
		Grid(const Grid& copyFrom);

		void Visualize();
		std::vector<std::shared_ptr<Node>> GetNeighbours(
			std::shared_ptr<Node> node);
		std::shared_ptr<Node> GetNodeFromWorldPosition(
			const glm::vec2 position);

		int width_, length_;
		glm::vec2 grid_size_;
		glm::vec2 world_min_;
		std::vector<std::shared_ptr<Node>> data_;
	};

	class Pathfinding {
	public:
		Pathfinding(Grid grid_map);
		void UpdateGrid(Grid grid_map) { grid_map_ = grid_map; }
		std::vector<std::shared_ptr<Node>> FindPath(const glm::vec2 seek,
													const glm::vec2 target);

	private:
		std::vector<std::shared_ptr<Node>> RetracePath(
			std::shared_ptr<Node> start_node, std::shared_ptr<Node> end_node);
		std::vector<std::shared_ptr<Node>> SimplifyPath(
			std::vector<std::shared_ptr<Node>> path);
		int GetDistance(std::shared_ptr<Node> node_a,
						std::shared_ptr<Node> node_b);
		
		Grid grid_map_;
	};

	struct PathRequest
	{
		PathRequest() : path_start(glm::vec2(0, 0)),
						path_end(glm::vec2(0, 0)),
						id(-1) {}

		PathRequest(glm::vec2 start, glm::vec2 end) : path_start(start),
													  path_end(end),
													  id(-1) {}
		glm::vec2 path_start;
		glm::vec2 path_end;	
		int id;
	};

	class PathRequestManager {
	public:
		PathRequestManager();
		std::vector<std::shared_ptr<Node>> RequestPath(glm::vec2 path_start,
													   glm::vec2 path_end);
		std::vector<std::shared_ptr<Node>> ProcessNext();
		void Flush();
		void UpdateGrid(Grid grid_map) { grid_map_ = grid_map; }

	private:
		Grid grid_map_;
		PathRequest current_request_;
		std::queue<PathRequest> requests_;
		//std::vector<std::thread> workers_;
	};

	class Navigation {
	public:
		Navigation(
			render_engine::GLContext * ctx,
			World * world,
			const unsigned int width = 20,
			const unsigned int length = 20);
		~Navigation();

		void SpawnAgent(const glm::vec3 position,
						const std::string& path, 
						const std::string& label);
		void KillAgentOnceArrived();
		void Update();
		void Reset();
		void BakeNavMesh();
		void SetBakeArea(
			const glm::vec3 world_min,
			const glm::vec3 world_max);

		void SetSurfaceLevel(const float level) { surface_level_ = level; }
		Grid GetGrid() { return grid_map_; }
		GLuint GetGridTexture() { return grid_texture_; }

		std::vector<Agent> crowd_;
		std::vector<CarveShape> carve_shapes_;

	private:
		// Flocking Behavior
		void Speration(const int current_id);
		void Alignment(const int current_id);
		void Cohesion(const int current_id);

		// Collision Avoidance with Static Object
		void Blocking(const int current_id, const glm::vec3 current_position);

		// Collision Avoidance with Dynamic Object
		void ClearCarving();
		bool Carving();

		// Voxelize
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
		PathRequestManager request_manager_;
		int update_counter_;
		int counter_;
	};
}

#endif //CROWD_H_