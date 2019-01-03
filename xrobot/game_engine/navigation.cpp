#include <string>
#include "navigation.h"
#include "world.h"

namespace xrobot {

Grid::Grid(const int width, const int length) : 
        width_(width),
        length_(length),
        world_min_(glm::vec2(0,0)),
        grid_size_(glm::vec2(0,0)),
        data_(width * length) {}

Grid::Grid(const Grid& other) : 
        width_(other.width_),
        length_(other.length_),
        world_min_(other.world_min_),
        grid_size_(other.grid_size_),
        data_(other.data_) {}

void Grid::Visualize() {
    for (int i = 0; i < length_; ++i) {
        for (int j = 0; j < width_; ++j) {
            printf("%s ", data_[i * width_ + j]->block ? "x" : " ");
        }
        printf("\n");
    }
}

std::vector<NodeSPtr> Grid::GetNeighbours(const NodeSPtr& node) {
    std::vector<NodeSPtr> neighbours(0);

    for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
            if(x == 0 && y == 0) {
                continue;
            }
            int check_x = node->x + x;
            int check_y = node->y + y;
            if(check_x >= 0 && check_x < width_ &&
               check_y >= 0 && check_y < length_) {
                neighbours.push_back(data_[check_y * width_ + check_x]);
            }
        }
    }

    return neighbours;
}

NodeSPtr Grid::GetNodeFromWorldPosition(const glm::vec2& pos2d) {
    glm::vec2 rel_pos =
            (pos2d - world_min_) / (glm::vec2(width_, length_) * grid_size_);
    rel_pos = glm::clamp(rel_pos, glm::vec2(0,0), glm::vec2(1,1));
    int x = round((width_ - 1) * rel_pos.x);
    int y = round((length_ - 1) * rel_pos.y);
    if(x >= 0 && x < width_ && y >= 0 && y < length_) {
        return data_[y*width_ + x];
    }

    return nullptr;
}

Navigation::Navigation(
		render_engine::GLContext* ctx,
		World* world,
		const unsigned int width,
		const unsigned int length) : ctx_(ctx),
									 world_(world),
									 width_(width),
									 length_(length),
									 grid_map_(width, length),
                                     depth_map_(width * length),
									 crowd_(0),
                                     surface_level_(-1.0f),
                                     request_manager_(),
                                     update_counter_(0),
                                     counter_(0),
                                     agent_id_base_(0),
                                     agent_radius_(0.0f) {
	assert(ctx && world);
	assert(width > 0 && length > 0);

	// Initialize Textures
	glGenFramebuffers(1, &fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    glGenTextures(1, &grid_texture_);
    glBindTexture(GL_TEXTURE_2D, grid_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, width_, length_,
    		0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
    		GL_TEXTURE_2D, grid_texture_, 0);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        std::cout << "Framebuffer not complete!" << std::endl;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Initialize Shaders

    size_t p = std::string(__FILE__).find_last_of("/");
    std::string pwd =std::string(__FILE__).substr(0, p);
    depth_shader_ = xrobot::render_engine::Shader(
    	pwd + "/render_engine/shaders/navmap.vs",
    	pwd + "/render_engine/shaders/navmap.fs");
}

Navigation::~Navigation() {
	glDeleteFramebuffers(1, &fbo_);
	glDeleteTextures(1, &grid_texture_);
    Reset();
}

void Navigation::Reset() {
    crowd_.clear();
    request_manager_.Flush();

    for (int i = 0; i < grid_map_.data_.size(); ++i) {
        grid_map_.data_[i] = nullptr;
    }
}

// Testing
void Navigation::SpawnAgent(
        const glm::vec3& pos,
        const glm::quat& quat,
        const std::string& path, 
        const std::string& label) {
    // Greate Robot
    std::weak_ptr<RobotBase> robot = world_->LoadRobot(
            path,
            glm::vec3(pos.x, 0.001, pos.z),
            glm::vec4(quat.x, quat.y, quat.z, quat.w),
            glm::vec3(1, 1, 1),
            label,
            false);

    if(auto robot_sptr = robot.lock()) {
        robot_sptr->ignore_baking(true);
    }

    world_->BulletStep();

    Agent agent;
    agent.current_pos_ = pos;
    agent.robot_ = robot;
    agent.speed_ = 0.005f;
    agent.uid_ = agent_id_base_++;

    crowd_.push_back(agent);
}

Agent::Agent() :
        robot_(),
        target_index_(0),
        speed_(0.1f),
        path_(0),
        distance_(0.0f),
        angle_(0.0f),
        uid_(0),
        last_direction_(glm::vec3(0,0,0)),
        current_pos_(glm::vec3(0,0,0)),
        target_pos_(glm::vec3(0,0,0)),
        request_update_(true) {}

void Agent::FollowPath() {

    if(path_.size() < 1) {
        return; 
    }
                
    if(target_index_ >= path_.size()) {
	    return; 
    }
                
    if (auto robot_sptr = robot_.lock()) {
        //Update Current Status
        btTransform current_transform_bt = robot_sptr->root_part_->object_position_;
        btVector3 current_pos_bt = current_transform_bt.getOrigin();

        current_pos_ = glm::vec3(
                current_pos_bt[0],
                current_pos_bt[1],
                current_pos_bt[2]);

        // On Track
        glm::vec2 current_pos_2d(current_pos_.x, current_pos_.z);
        glm::vec2 current_waypoint_2d = path_[target_index_]->world_pos;

        if(glm::distance(current_pos_2d, current_waypoint_2d) < 0.2f) {
            target_index_++;
            if(target_index_ >= path_.size()) {
                return;
            }
        }

        // Move to Waypoint
        glm::vec2 direction_2d =
                glm::normalize(current_waypoint_2d - current_pos_2d);
        glm::vec3 direction_3d = glm::vec3(direction_2d.x, 0, direction_2d.y);
        current_pos_ += direction_3d * speed_;
    }
}

void Agent::AssignTarget(const glm::vec3& pos) {
    target_pos_ = pos;
    request_update_ = true;
}

PathRequestManager::PathRequestManager() :
        requests_(),
        current_request_(),
        grid_map_() {

    // for (int i = 0; i < workers_.size(); ++i)
    // {
    //     workers.push_back(std::thread([i,&this] {
    //         PathRequest request;
    //         while(1) {
    //             if(requests_.dequeue(request)) {
    //                 Pathfinding(grid_map_).FindPath(
    //                     current_request_.path_start,
    //                     current_request_.path_end
    //                 );
    //             } else {
    //                 break;
    //             }
    //         }
    //     }));
    // }
}

std::vector<NodeSPtr> PathRequestManager::RequestPath(
        const glm::vec2& path_start, const glm::vec2& path_end) {
    PathRequest new_request(path_start, path_end);
    requests_.push(new_request);
    //requests_.enqueue(new_request);
    return ProcessNext();
}

std::vector<NodeSPtr> PathRequestManager::ProcessNext() {
    if(requests_.size()) {
        current_request_ = requests_.front();
        requests_.pop();

        // Calculate Path
        return Pathfinding(grid_map_).FindPath(
            current_request_.path_start,
            current_request_.path_end
        );
    }

    std::vector<NodeSPtr> no_result(0);
    return no_result;
}

void PathRequestManager::Flush() {
    std::queue<PathRequest> empty;
    std::swap(requests_, empty);
}

std::vector<NodeSPtr> Pathfinding::FindPath(
        const glm::vec2& seek, const glm::vec2& target) {
    NodeSPtr start_node = grid_map_.GetNodeFromWorldPosition(seek);
    NodeSPtr end_node = grid_map_.GetNodeFromWorldPosition(target);

    std::vector<NodeSPtr> open_set;
    std::unordered_set<NodeSPtr> closed_set;
    open_set.push_back(start_node);

    while(open_set.size() > 0) {
        NodeSPtr node = open_set.front();

        for (int i = 1; i < open_set.size(); ++i) {
            if(open_set[i]->FCost() <  node->FCost() || 
               open_set[i]->FCost() == node->FCost()) {
                if(open_set[i]->h_cost < node->h_cost)
                    node = open_set[i];
            }
        }

        auto it = std::find (open_set.begin(), open_set.end(), node);
        if(it != open_set.end()) {
            int offset = it - open_set.begin();
            open_set.erase(open_set.begin() + offset);
        }

        closed_set.insert(node);

        if(node == end_node) {
            return RetracePath(start_node, end_node);
        }

        for (NodeSPtr neighbour : grid_map_.GetNeighbours(node)) {
            auto it_t = closed_set.find(neighbour);
            if((neighbour->block || neighbour->carve) || it_t != closed_set.end()) {
                continue;
            }

            int new_cost_to_neighbour = node->g_cost + GetDistance(node, neighbour);

            auto it_s = std::find (open_set.begin(), open_set.end(), neighbour);
            if (new_cost_to_neighbour < neighbour->g_cost || 
                it_s == open_set.end()) 
            {
                neighbour->g_cost = new_cost_to_neighbour;
                neighbour->h_cost = GetDistance(neighbour, end_node);
                neighbour->parent = node;

                if(it_s == open_set.end()) {
                    open_set.push_back(neighbour);
                }
            }
        }
    }

    std::vector<NodeSPtr> no_result(0);
    return no_result;
}

std::vector<NodeSPtr> Pathfinding::RetracePath(
    const NodeSPtr& start_node, const NodeSPtr& end_node) {
    std::vector<NodeSPtr> path;

    NodeSPtr current_node = end_node;

    while (current_node != start_node) {
        path.push_back(current_node);
        current_node = current_node->parent;
    }

    SimplifyPath(path);

    std::reverse(path.begin(), path.end());

    return path;
}

void Pathfinding::SimplifyPath(std::vector<NodeSPtr>& path) {
    glm::vec2 direction_old = glm::vec2(0,0);
    int p = 0;
    for (int i = 1; i < path.size(); ++i) {
        glm::vec2 direction_new = glm::vec2(
                path[i - 1]->x - path[i]->x,
                path[i - 1]->y - path[i]->y);

        path[i-1]->angle = glm::dot(direction_new, direction_old);

        if(direction_new != direction_old) {
            path[p++] = path[i-1];
        }
        direction_old = direction_new;
    }
    path.resize(p);

    // Testing
    
    // for (int i = 0; i < grid_map_.length_; ++i)
    // {
    //     for (int j = 0; j < grid_map_.width_; ++j)
    //     {
    //         bool flag = true;
    //         bool way = false;
    //         for (auto node : path)
    //         {
    //             if(node->x == j && node->y == i) {
    //                 way = false;
    //                 flag = false;
    //             }
    //         }
    //         for (auto node : waypoints)
    //         {
    //             if(node->x == j && node->y == i) {
    //                 way = true;
    //                 flag = false;
    //             }
    //         }
    //         if(flag) {
    //             if (grid_map_.data_[i * grid_map_.width_ + j]->block) {
    //                 printf("%s ","x");
    //             } else if (grid_map_.data_[i * grid_map_.width_ + j]->carve) {
    //                 printf("%s ","c");
    //             } else {
    //                 printf("%s "," ");
    //             }
    //         }
    //         else {
    //             if(way) {
    //                 printf("%s ", "o");
    //             } else {
    //                 printf("%s ", "+");
    //             }
    //         }
    //     }
    //     printf("\n");
    // }
}

int Pathfinding::GetDistance(
        const NodeSPtr& node_a, const NodeSPtr& node_b) {
    int dst_x = abs(node_a->x - node_b->x);
    int dst_y = abs(node_a->y - node_b->y);

    if(dst_x > dst_y) {
        return 14 * dst_y + 10 * (dst_x - dst_y);
    }

    return 14 * dst_x + 10 * (dst_y - dst_x);
}

void Navigation::Speration(const int current_id) {
    glm::vec2 vel_speration = glm::vec2(0, 0);

    glm::vec2 agent_current = glm::vec2(
            crowd_[current_id].current_pos_.x, 
            crowd_[current_id].current_pos_.z
    );

    for (int j = 0; j < crowd_.size(); ++j) {
        if(j == current_id) { continue; }

        if(crowd_[j].path_.size() < 1) { continue; }

        glm::vec2 near_current = glm::vec2(
                crowd_[j].current_pos_.x, crowd_[j].current_pos_.z);

        if(glm::distance(near_current, agent_current) < 1.0f) {
            vel_speration -= (near_current - agent_current);
        }
    }

    for (int k = 0; k < carve_shapes_.size(); ++k) {
        glm::vec2 near_current = carve_shapes_[k].pos;
        if (glm::distance(near_current, agent_current) <
                carve_shapes_[k].radius * 0.75f) {
            vel_speration -= (near_current - agent_current) * 1.5f;
        }
    }

    vel_speration *= 0.00125f;

    crowd_[current_id].current_pos_ += glm::vec3(
            vel_speration.x, 0, vel_speration.y);
}

void Navigation::Alignment(const int current_id) {
    glm::vec2 vel_alignment = glm::vec2(0, 0);

    glm::vec2 agent_current = glm::vec2(
        crowd_[current_id].current_pos_.x, 
        crowd_[current_id].current_pos_.z
    );

    float num_alignment = 0.0f;

    for (int j = 0; j < crowd_.size(); ++j) {
        if(j == current_id) { continue; }

        if(crowd_[j].path_.size() < 1) { continue; }

        glm::vec2 near_current = glm::vec2(
                crowd_[j].current_pos_.x, 
                crowd_[j].current_pos_.z);

        if (glm::distance(near_current, agent_current) < 0.5f) {
            int target_index = crowd_[j].target_index_;
            if(target_index >= crowd_[j].path_.size()) {
                return;
            }

            glm::vec2 target = crowd_[j].path_[target_index]->world_pos;
            glm::vec2 direction = glm::normalize(target - near_current);

            vel_alignment += direction;
            num_alignment++;
        }
    }

    if(num_alignment > 0) {
        vel_alignment /= num_alignment;
        vel_alignment *= 0.0002f;
    }

    crowd_[current_id].current_pos_ += glm::vec3(
            vel_alignment.x, 0, vel_alignment.y);
}

void Navigation::Cohesion(const int current_id) {
    glm::vec2 vel_cohesion = glm::vec2(0, 0);

    glm::vec2 agent_current = glm::vec2(
            crowd_[current_id].current_pos_.x, 
            crowd_[current_id].current_pos_.z);

    float num_cohesion = 0.0f;

    for (int j = 0; j < crowd_.size(); ++j) {
        if(j == current_id) { continue; }

        if(crowd_[j].path_.size() < 1) { continue; }

        glm::vec2 near_current = glm::vec2(
                crowd_[j].current_pos_.x, 
                crowd_[j].current_pos_.z);

        if(glm::distance(near_current, agent_current) < 2.0f) {
            vel_cohesion += near_current;
            num_cohesion++;
        }
    }

    if(num_cohesion > 0) {
        vel_cohesion /= num_cohesion;
        vel_cohesion = (vel_cohesion - agent_current) * 0.0001f;
    }

    crowd_[current_id].current_pos_ += glm::vec3(
            vel_cohesion.x, 0, vel_cohesion.y);
}

void Navigation::Blocking(
        const int current_id, const glm::vec3& current_pos) {
    glm::vec3 next_pos = crowd_[current_id].current_pos_;

    glm::vec2 next_pos_2d(next_pos.x, next_pos.z);
    glm::vec2 current_pos_2d(current_pos.x, current_pos.z);

    NodeSPtr next_pos_locate_node = 
        grid_map_.GetNodeFromWorldPosition(next_pos_2d);
    NodeSPtr curr_pos_locate_node = 
        grid_map_.GetNodeFromWorldPosition(current_pos_2d);

    if(next_pos_locate_node->carve) {

        glm::vec2 reflect = current_pos_2d - next_pos_2d;

	    //crowd_[current_id].current_pos_ = current_pos + glm::vec3(reflect.x, 0, reflect.y);
        //printf("Agent #%d Request Re-Path (Block) \n", current_id);
        crowd_[current_id].target_index_ = 0;
        crowd_[current_id].request_update_ = true;
    }
}

void Navigation::ClearCarving() {
    for (int i = 0; i < grid_map_.data_.size(); ++i) {
        grid_map_.data_[i]->carve = false;
    }
}

bool Navigation::Carving() {
    bool update = false;

    ClearCarving();

    for (int i = 0; i < grid_map_.length_; ++i) {
        for (int j = 0; j < grid_map_.width_; ++j) {

            glm::vec2 world_pos = 
                grid_map_.data_[i * width_ + j]->world_pos;

            for (int k = 0; k < carve_shapes_.size(); ++k) {
                glm::vec2 carve_pos = carve_shapes_[k].pos;

                if(glm::distance(carve_pos, world_pos) <= carve_shapes_[k].radius) {
                    grid_map_.data_[i * width_ + j]->carve = true;
                    update |= carve_shapes_[k].moving;
                }
            }
        }
    }

    request_manager_.UpdateGrid(grid_map_);

    return update;
}

void Navigation::Update() {
    // Carve Navivation Grid
    bool update = Carving();

    for (int i = 0; i < crowd_.size(); ++i) {
        // Update NavMesh and Request Path
        if(crowd_[i].request_update_) {
            std::vector<NodeSPtr> new_path;

            glm::vec2 path_start(
                    crowd_[i].current_pos_.x, crowd_[i].current_pos_.z);
            glm::vec2 path_end(crowd_[i].target_pos_.x, crowd_[i].target_pos_.z);
            new_path = request_manager_.RequestPath(path_start, path_end);
            crowd_[i].path_ = new_path;
            crowd_[i].request_update_ = false;
        }

        glm::vec3 last_pos = crowd_[i].current_pos_;

        // Check Arrive?
        float dist = glm::distance(crowd_[i].current_pos_, crowd_[i].target_pos_);
        if(dist > 0.4f) {
            // Apply Path Finding
            crowd_[i].FollowPath();
            glm::vec3 current_pos = crowd_[i].current_pos_;

            // Apply Flocking Behavior
            Speration(i);
            Alignment(i);
            Cohesion(i);

            // Auto Repath
            if(counter_ % 16) {
                glm::vec3 updated_pos = crowd_[i].current_pos_;
                crowd_[i].distance_ += glm::length(updated_pos - last_pos);
            } else {
                if(crowd_[i].distance_ < (crowd_[i].speed_ * 0.5f * 15)) {
                    crowd_[i].target_index_ = 0;
                    crowd_[i].request_update_ = true;
                    //printf("Agent #%d Request Re-Path (Freeze)\n", i);
                }
                crowd_[i].distance_ = 0;
            }

            if(counter_ % 3 == 0 && update) {
                Blocking(i, current_pos);
            }
        } else if(crowd_[i].target_index_ >= crowd_[i].path_.size() - 1) {
            continue;
        }

        // Update Agent Status
        
        // TODO
        // Agent Rotation
        auto robot = crowd_[i].robot_;

        if (auto robot_sptr = robot.lock()) {

            btQuaternion orn = btQuaternion(btVector3(-1,0,0), 1.57);

            glm::vec3 next_pos = crowd_[i].current_pos_;
            
            glm::vec3 current_direction = glm::normalize(next_pos - last_pos);
            float angle = glm::angle(current_direction, glm::vec3(-1,0,0));

            // if(glm::abs(angle - crowd_[i].angle_) > 0.05f)
            // {
            //     crowd_[i].current_pos_ = last_pos;
            //     next_pos = last_pos;
            // }

            crowd_[i].angle_ = crowd_[i].angle_ +
                    glm::sign(angle - crowd_[i].angle_) * 0.01f;

            btQuaternion angle_orn =
                    btQuaternion(btVector3(0,1,0), crowd_[i].angle_);

            btTransform transform_temp;
            transform_temp.setIdentity();
            transform_temp.setRotation(angle_orn * orn);
            transform_temp.setOrigin(
                    btVector3(next_pos.x, next_pos.y, next_pos.z));

            world_->SetTransformation(robot_sptr, transform_temp);

            crowd_[i].last_direction_ = current_direction;
        }
    }

    counter_++;
}

void Navigation::KillAgentOnceArrived() {
    for (int i = 0; i < crowd_.size(); ++i) {
        // Check Arrive?
        float dist = glm::distance(crowd_[i].current_pos_, crowd_[i].target_pos_);

        if(dist < 1.0f) {

            auto robot = crowd_[i].robot_;
            if(auto robot_sptr = robot.lock()) {
                robot_sptr->RemoveRobotFromBullet();
                world_->RemoveRobot(robot_sptr);
                robot_sptr = nullptr;
                crowd_.erase(crowd_.begin() + i);
            }

            // Rebake
            // BakeNavMesh();

            // for (int i = 0; i < crowd_.size(); ++i)
            // {
            //     if(crowd_[i].target_index_ >= crowd_[i].path_.size()) {
            //         crowd_[i].request_update_ = true;
            //         crowd_[i].target_index_ = 0;
            //     }
            // }

            //printf("Kill Agent #%d\n", i);
            break;
        }
    }
}

void Navigation::SetBakeArea(
        const glm::vec3& world_min, const glm::vec3& world_max) {
    world_min_ = world_min;
    world_max_ = world_max;
}

void Navigation::BakeNavMesh() {
    auto gl_extension_supported = [](const std::string& name) -> bool {
        // Conservative Rasterization EXT
        // At Least NVIDIA Maxwell
        int num_extenstions;
        glGetIntegerv(GL_NUM_EXTENSIONS, &num_extenstions);
        for(int i = 0; i < num_extenstions; i++) {
            std::string ext(reinterpret_cast<const char*>(
                        glGetStringi(GL_EXTENSIONS, i)));
            if (ext == name) {
                return true;
            }
        }
        return false;
    };
    
    bool supported = gl_extension_supported("GL_NV_conservative_raster");

	Voxelization();

    if (supported) {
        #define GL_CONSERVATIVE_RASTERIZATION_NV 0x9346
        glDisable(GL_CONSERVATIVE_RASTERIZATION_NV);
    }

    request_manager_.UpdateGrid(grid_map_);

    //printf("Update Navigation Map...\n");
}

void Navigation::Voxelization() {
	// Update Projection Matrices
	glm::vec3 center = (world_min_ + world_max_) * 0.5f;
    glm::vec3 axis_size = world_max_ - world_min_;
    glm::vec3 pixel_size = axis_size / glm::vec3(width_, 1, length_);
    glm::mat4 projection = glm::ortho(-axis_size.x * 0.5f, axis_size.x * 0.5f,
    		-axis_size.z * 0.5f, axis_size.x * 0.5f, 0.0f, axis_size.y);
    glm::mat4 view = glm::lookAt(
            center + glm::vec3(0.0, axis_size.y * 0.5f, 0.0f),
    		center - glm::vec3(0.0, axis_size.y * 0.5f, 0.0f), 
            glm::vec3(1.0f, 0.0f, 0.0f));

    // Update Grid Map
    grid_map_.grid_size_ = glm::vec2(pixel_size.x, pixel_size.z);
    grid_map_.world_min_ = glm::vec2(world_min_.x, world_min_.z);

    // Raster
    depth_shader_.use();
    depth_shader_.setFloat("radius", agent_radius_);
    depth_shader_.setMat4("view", view);
    depth_shader_.setMat4("projection", projection);

    auto do_drawing = [&](const xrobot::render_engine::RenderPart* c, bool is_root) {
        for (size_t i = 0; i < c->size(); ++i) {
            auto model = c->model_data(i);
            auto transform = c->transform(i);

            glm::mat4 translate = c->translation_matrix();
            glm::mat4 local_frame = 
                    glm::inverse(c->local_inertial_frame());
            glm::mat4 scale =  glm::scale(
                    glm::mat4(1), transform->local_scale);

            depth_shader_.setMat4("matrices.state", translate * local_frame);
            depth_shader_.setMat4("matrices.scale", scale);
            if(!is_root)
                depth_shader_.setMat4("matrices.model", transform->origin);
            else
                depth_shader_.setMat4("matrices.model", glm::mat4(1));
            depth_shader_.setFloat("flip", transform->flip);
            model->Draw(depth_shader_);
        }
    };


    glViewport(0, 0, width_, length_);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    world_->robot_iteration_begin();
    while (world_->has_next_robot()) {
        render_engine::RenderBody* body = world_->next_robot();
        if(body->ignore_baking() || body->is_hiding()) {
            continue;
        }
        // Root
        xrobot::render_engine::RenderPart* root = body->render_root_ptr();
        if (root && !body->is_recycled()) {
            do_drawing(root, true);
            // Parts
            for (size_t j = 0; j < body->size(); ++j) {
                if (body->render_part_ptr(j)) {
                    do_drawing(body->render_part_ptr(j), false);
                }
            }
        }
	} 

    // Sync Transfer Buffer To Array
	glFlush();
    glFinish();
    glReadBuffer(GL_DEPTH_ATTACHMENT); 
    glReadPixels(0, 0, width_, length_,  GL_DEPTH_COMPONENT,  GL_FLOAT, &depth_map_[0]); 
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Calculate Surface Level
    if (surface_level_ < 0) {
        std::unordered_map<float, int> major;
        for (int i = 0; i < depth_map_.size(); ++i) {
            if (major.find(depth_map_[i]) != major.end()) {
                int count = major[depth_map_[i]] + 1;
                if(count > depth_map_.size() / 2) {
                    // Find Surface Level
                    surface_level_ = depth_map_[i];
                    //printf("Find Surface Level: %f\n", surface_level_);
                    break;
                } else {
                    major[depth_map_[i]] = count;
                }
            } else {
                major[depth_map_[i]] = 1;
            }
        }
    }

    // Build NavMesh
    for (int i = 0; i < length_; ++i) {
        for (int j = 0; j < width_; ++j) {
            NodeSPtr node = std::make_shared<Node>();

            node->x = j;
            node->y = i;
            node->world_pos.x = world_min_.x + pixel_size.x*.5 + pixel_size.x*j;
            node->world_pos.y = world_min_.z + pixel_size.z*.5 + pixel_size.z*i;

            // ????
            if(depth_map_[j * length_ + i] == 1.0f) {
                node->block = true;
            } else if(depth_map_[j * length_ + i] <  surface_level_) {
                node->block = true;
            } else if(depth_map_[j * length_ + i] == surface_level_){
                node->block = false;
            } else {
                node->block = false;
            }

            grid_map_.data_[i * width_ + j] = node;
        }
    }
}

}
