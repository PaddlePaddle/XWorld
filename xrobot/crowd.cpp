#include "crowd.h"
#include "world.h"

namespace xrobot {

Grid::Grid(const int width, const int length) : width_(width),
                                                length_(length),
                                                world_min_(glm::vec2(0,0)),
                                                grid_size_(glm::vec2(0,0)),
                                                data_(width * length) 
{

}

void Grid::Visualize()
{
    for (int i = 0; i < length_; ++i)
    {
        for (int j = 0; j < width_; ++j)
        {
            printf("%s ", data_[i * width_ + j]->block ? "x" : " ");
        }
        printf("\n");
    }
}

std::vector<Node*> Grid::GetNeighbours(Node* node)
{
    std::vector<Node*> neighbours(0);

    for (int x = -1; x <= 1; ++x)
    {
        for (int y = -1; y <= 1; ++y)
        {
            if(x == 0 && y == 0)
                continue;

            int check_x = node->x + x;
            int check_y = node->y + y;

            if(check_x >= 0 && check_x < width_ && check_y >= 0 && check_y < length_) {
                neighbours.push_back(data_[check_y * width_ + check_x]);
            }
        }
    }

    return neighbours;
}

Node* Grid::GetNodeFromWorldPosition(const glm::vec2 position)
{

}

Crowd::Crowd(
		render_engine::GLContext * ctx,
		World * world,
		const unsigned int width,
		const unsigned int length) : ctx_(ctx),
									 world_(world),
									 width_(width),
									 length_(length),
									 grid_map_(width, length),
                                     depth_map_(width * length),
									 crowd_(0),
                                     surface_level_(-1.0f)
{
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
    depth_shader_ = xrobot::render_engine::Shader(
    	"/home/ziyuli/XWorld/xrobot/depth.vs",
    	"/home/ziyuli/XWorld/xrobot/depth.fs");
}

Crowd::~Crowd()
{
	glDeleteFramebuffers(1, &fbo_);
	glDeleteTextures(1, &grid_texture_);
}

void Crowd::SetBakeArea(const glm::vec3 world_min, const glm::vec3 world_max) 
{
    world_min_ = world_min;
    world_max_ = world_max;
}

void Crowd::BakeNavMesh()
{
    // Conservative Rasterization EXT
    // At Least NVIDIA Maxwell
    glEnable(GL_CONSERVATIVE_RASTERIZATION_NV);

	Voxelization();

    glDisable(GL_CONSERVATIVE_RASTERIZATION_NV);
}

void Crowd::Voxelization()
{
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
    depth_shader_.setMat4("view", view);
    depth_shader_.setMat4("projection", projection);

    auto do_drawing = [&](const xrobot::render_engine::RenderPart* c, bool is_root) {
        for (size_t i = 0; i < c->size(); ++i) {
            xrobot::render_engine::ModelData* model = c->model_data(i);
            xrobot::render_engine::OriginTransformation* transform = c->transform(i);

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

    for (size_t i = 0; i < world_->size(); ++i) {
        xrobot::render_engine::RenderBody* body = world_->render_body_ptr(i);

        if(body->move() || body->hide())
            continue;

        // Root
        xrobot::render_engine::RenderPart* root = body->render_root_ptr();
        if (root && !body->recycle()) {
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
        
        for (int i = 0; i < depth_map_.size(); ++i)
        {
            if (major.find(depth_map_[i]) != major.end()) {
                int count = major[depth_map_[i]] + 1;
                if(count > depth_map_.size() / 2) {
                    // Find Surface Level
                    surface_level_ = depth_map_[i];
                    printf("Find Surface Level: %f\n", surface_level_);
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
    for (int i = 0; i < length_; ++i)
    {
        for (int j = 0; j < width_; ++j)
        {
            Node * node = new Node();

            node->x = j;
            node->y = i;
            node->world_position.x = world_min_.x + pixel_size.x * 0.5f + pixel_size.x * j;
            node->world_position.y = world_min_.z + pixel_size.z * 0.5f + pixel_size.z * i;

            if(depth_map_[i * width_ + j] == 1.0f) {
                node->block = true;
            } else if(depth_map_[i * width_ + j] <  surface_level_) {
                node->block = true;
            } else if(depth_map_[i * width_ + j] == surface_level_){
                node->block = false;
            } else {
                node->block = false;
            }

            grid_map_.data_[i * width_ + j] = node;
        }
    }
}

}