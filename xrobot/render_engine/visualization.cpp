#include "visualization.h"

namespace xrobot {
namespace render_engine {

Visualization::Visualization(const int width,
				  			 const int height,
				  			 GLContext* ctx) :  ctx_(ctx),
												width_(width),
												height_(height),
												last_x_(width * 0.5f),
												last_y_(height * 0.5f),
												delta_time_(0),
												num_rays_(0),
												aabb_vao_(0),
												batch_ray_vao_(0),
												pc_vao_(0),
												first_mouse_(true),
												lidar_(false) {

	free_camera_ = Camera(glm::vec3(0.0f, 0.0f, 0.0f));
	free_camera_.SetAspect((float)width / height);
	free_camera_.SetNear(0.05f);
	free_camera_.SetFar(200.0f);
	free_camera_.front_ = glm::vec3(1, 0, 0);

	visualization_ = std::make_shared<RenderTarget>(width_, height_);
	visualization_->append_rgb_uint8();
	visualization_->append_depth_readonly();
	visualization_->init();

	InitShaders();
}

Visualization::~Visualization() {
	glDeleteVertexArrays(1, &aabb_vao_);
	glDeleteVertexArrays(1, &batch_ray_vao_);
	glDeleteBuffers(1, &aabb_vbo_);
	glDeleteBuffers(1, &batch_ray_vbo_);
}

void Visualization::InitShaders() {
	std::string pwd = get_pwd(__FILE__);
	shaders_.resize((int) kVisualizationShaders);

	shaders_[kAABB] = Shader(pwd+"/shaders/line.vs",
							 pwd+"/shaders/line.fs");

	shaders_[kRay]  = Shader(pwd+"/shaders/ray.vs",
							 pwd+"/shaders/ray.fs",
							 pwd+"/shaders/ray.gs",
							 -1);

	shaders_[kLambert]  = Shader(pwd+"/shaders/lambert.vs",
							     pwd+"/shaders/lambert.fs");

	shaders_[kPointCloud]  = Shader(pwd+"/shaders/point.vs",
							        pwd+"/shaders/point.fs");
}

void Visualization::DrawPointCloud(const GLuint tex, Camera* camera, 
		const int size, const int dir) {
	if(pc_vao_ == 0) {

		std::vector<glm::vec3> pc(size);

		glGenVertexArrays(1, &pc_vao_);
		glGenBuffers(1, &pc_vbo_);
		glBindBuffer(GL_ARRAY_BUFFER, pc_vbo_);
		glBufferData(GL_ARRAY_BUFFER, sizeof(pc.data()), pc.data(), GL_STATIC_DRAW);
		glBindVertexArray(pc_vao_);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
	}

	glm::mat4 projection = free_camera_.GetProjectionMatrix();
	glm::mat4 view = free_camera_.GetViewMatrix();
	glm::vec3 eye  = glm::vec3(0);
	glm::vec3 front = glm::vec3(1,0,0);
	glm::vec3 up = glm::vec3(0,1,0);

	glm::mat4 capture_projection = glm::perspective(glm::radians(90.0f),
			1.0f, 0.02f, 20.0f);

	front = camera->front_;
	up = camera->up_;

	glm::mat4 capture_views[] = 
	{
		glm::lookAt(eye,  front, glm::vec3(0.0f, 1.0f,  0.0f)),
	  	glm::lookAt(eye, -front, glm::vec3(0.0f, 1.0f,  0.0f)),
	  	glm::mat4(1),
	  	glm::mat4(1),
	  	glm::lookAt(eye, -cross(front, up), glm::vec3(0.0f, 1.0f,  0.0f)),
	  	glm::lookAt(eye, cross(front, up), glm::vec3(0.0f, 1.0f,  0.0f)),
	};

	auto pointcloud = shaders_[kPointCloud];

	glBindVertexArray(pc_vao_);
	glEnable(GL_PROGRAM_POINT_SIZE);
	glPointSize(1.5f);
	pointcloud.use();
	pointcloud.setInt("size", kLidarCaptureRes);
	pointcloud.setInt("dir", dir);
	pointcloud.setVec3("camera_worldpos", camera->position_);
	pointcloud.setMat4("camera_direction", capture_views[dir]);
	pointcloud.setMat4("inv_cubemap_projection", glm::inverse(capture_projection));
	pointcloud.setMat4("view", view);
	pointcloud.setMat4("projection", projection);
	glDrawArrays(GL_POINTS, 0, size);
	glBindVertexArray(0);
}

void Visualization::DrawRootAABB(RenderWorld* world, const Shader& shader) {
	for (size_t i = 0; i < world->size(); i++) {
		RenderBody* body = world->render_body_ptr(i);
		RenderPart * part = body->render_root_ptr();
		if (part && !body->is_recycled()) {
			glm::vec3 aabb_min, aabb_max;
			part->GetAABB(aabb_min, aabb_max);
			shader.setVec3("aabbMin", aabb_min);
			shader.setVec3("aabbMax", aabb_max);
			RenderAABB();
		}
	}
}

void Visualization::DrawSubTiles(RenderWorld* world, const Shader& shader) {
	for (size_t i = 0; i < world->debug_subtiles_.size(); i++) {
		glm::vec2 bbox_min = world->debug_subtiles_[i].first;
		glm::vec2 bbox_max = world->debug_subtiles_[i].second;

		glm::vec3 aabb_min(bbox_min.x, 0, bbox_min.y);
		glm::vec3 aabb_max(bbox_max.x, 0, bbox_max.y);

		if (world->debug_subtile_status_[i] == 0) {
			shader.setVec3("color", glm::vec3(1,0,0));
		} else if (world->debug_subtile_status_[i] == 2){
			shader.setVec3("color", glm::vec3(0,0,1));
			aabb_max.y = 0.01f;
			aabb_min.y = 0.001f;
		} else if (world->debug_subtile_status_[i] == 1){
			shader.setVec3("color", glm::vec3(1,1,0));
			aabb_max.y = 0.01f;
			aabb_min.y = 0.001f;
		}

		shader.setVec3("aabbMin", aabb_min);
		shader.setVec3("aabbMax", aabb_max);
		RenderAABB();
	}
}

void Visualization::DrawWorldAABB(RenderWorld* world, const Shader& shader) {
	float world_min_x, world_min_z, world_max_x, world_max_z;
	world->get_world_size(world_min_x, world_min_z, world_max_x, world_max_z);

	glm::vec3 aabb_min(world_min_x, 0, world_min_z);
	glm::vec3 aabb_max(world_max_x, 10, world_max_z);

	shader.setVec3("color", glm::vec3(0,1,1));
	shader.setVec3("aabbMin", aabb_min);
	shader.setVec3("aabbMax", aabb_max);
	RenderAABB();
}

void Visualization::DrawBatchRays() {
	glm::mat4 projection = free_camera_.GetProjectionMatrix();
	glm::mat4 view = free_camera_.GetViewMatrix();

	Shader ray = shaders_[kRay];
	ray.use();
	ray.setMat4("projection", projection);
	ray.setMat4("view", view);

	if(batch_ray_vao_) {
		glBindVertexArray(batch_ray_vao_);
		glDrawArrays(GL_LINES, 0, 2 * num_rays_);
		glBindVertexArray(0);
	}
}

void Visualization::UpdateRay(const int offset, 
		const glm::vec3 from, const glm::vec3 to) {
	float sub_buffer_temp[6];
	sub_buffer_temp[0] = from[0];
	sub_buffer_temp[1] = from[1];
	sub_buffer_temp[2] = from[2];
	sub_buffer_temp[3] = to[0];
	sub_buffer_temp[4] = to[1];
	sub_buffer_temp[5] = to[2];

	glBindBuffer(GL_ARRAY_BUFFER, batch_ray_vbo_);
	glBufferSubData(GL_ARRAY_BUFFER, offset * 6 * sizeof(float),
			6 * sizeof(float), sub_buffer_temp);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void Visualization::InitDrawBatchRays(const int rays) {
	lidar_ = true;

	num_rays_ = rays;
	if(batch_ray_vao_ == 0) {
		glGenVertexArrays(1, &batch_ray_vao_);
		glBindVertexArray(batch_ray_vao_);

		glGenBuffers(1, &batch_ray_vbo_);
		glBindBuffer(GL_ARRAY_BUFFER, batch_ray_vbo_);
		glBufferData(GL_ARRAY_BUFFER, 
				num_rays_ * 6 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT,
				GL_FALSE, 3 * sizeof(float), (void*)0);
		glBindVertexArray(0);
	}
}

void Visualization::Visualize(RenderWorld* world, Camera* camera, 
		GLuint tex) {

	GetDeltaTime();
	ProcessMouse();
	ProcessInput();

	glm::mat4 projection = free_camera_.GetProjectionMatrix();
	glm::mat4 view = free_camera_.GetViewMatrix();

	Shader lambert = shaders_[kLambert];
	Shader line = shaders_[kAABB];

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glClearColor(0.8, 0.8, 0.8, 1.0);

	glBindFramebuffer(GL_FRAMEBUFFER, visualization_->id());

	glViewport(0, 0, width_, height_);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glEnable( GL_POLYGON_OFFSET_FILL );
	glPolygonOffset(1.0f, 1.0f);

	lambert.use();
	lambert.setMat4("projection", projection);
	lambert.setMat4("view", view);
	Draw(world, lambert);

	glDisable( GL_POLYGON_OFFSET_FILL );

	glEnable(GL_LINE_SMOOTH);
	glLineWidth(1.0f);

	line.use();
	line.setMat4("projection", projection);
	line.setMat4("view", view);

	line.setVec3("color", glm::vec3(1,0,0));
	DrawRootAABB(world, line);

	// line.setVec3("color", glm::vec3(0,1,0));
	// DrawSubTiles(world, line);

	line.setVec3("color", glm::vec3(0,0,1));
	DrawWorldAABB(world, line);
	
	if(tex > 0) {
		DrawPointCloud(tex, camera, kLidarCaptureRes * kLidarCaptureRes, 0);
		DrawPointCloud(tex, camera, kLidarCaptureRes * kLidarCaptureRes, 4);
		DrawPointCloud(tex, camera, kLidarCaptureRes * kLidarCaptureRes, 5);
		DrawPointCloud(tex, camera, kLidarCaptureRes * kLidarCaptureRes, 1);
	}

	if(lidar_) {
		glDisable(GL_LINE_SMOOTH);
		DrawBatchRays();
	}

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Visualization::Draw(RenderWorld* world, const Shader& shader) {
	auto do_drawing = [&](const RenderPart* c, bool is_root) {
		for (size_t i = 0; i < c->size(); ++i) {
			ModelData* model = c->model_data(i).get();
			OriginTransformation* transform = c->transform(i).get();

			glm::mat4 translate = c->translation_matrix();
			glm::mat4 local_frame = 
					glm::inverse(c->local_inertial_frame());
			glm::mat4 scale =  glm::scale(
					glm::mat4(1), transform->local_scale);

			shader.setMat4("matrices.state", translate * local_frame);
			shader.setMat4("matrices.scale", scale);
			if(!is_root)
				shader.setMat4("matrices.model", transform->origin);
			else
				shader.setMat4("matrices.model", glm::mat4(1));
			shader.setFloat("flip", transform->flip);
			model->Draw(shader);
		}
	};

	for (size_t i = 0; i < world->size(); ++i) {
		RenderBody* body = world->render_body_ptr(i);

		if(body->is_hiding())
			continue;

		// Root
		RenderPart* root = body->render_root_ptr();
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
}

void Visualization::ProcessInput() {
	if(ctx_->GetKeyPressESC())
		ctx_->SetWindowShouldClose();
	if(ctx_->GetKeyPressW()) 
		free_camera_.ProcessKeyboard(Camera::kForward, delta_time_);
	if(ctx_->GetKeyPressS())
		free_camera_.ProcessKeyboard(Camera::kBackward, delta_time_);
	if(ctx_->GetKeyPressA())
		free_camera_.ProcessKeyboard(Camera::kLeft, delta_time_);
	if(ctx_->GetKeyPressD())
		free_camera_.ProcessKeyboard(Camera::kRight, delta_time_);
	if(ctx_->GetKeyPressQ())
		free_camera_.ProcessKeyboard(Camera::kUp, delta_time_);
	if(ctx_->GetKeyPressE())
		free_camera_.ProcessKeyboard(Camera::kDown, delta_time_);
}

void Visualization::ProcessMouse() {
	float x_position, y_position;
	ctx_->GetMouse(x_position, y_position);

	if(first_mouse_) {
		last_x_ = x_position;
		last_y_ = y_position;
		first_mouse_ = false;
	}

	float x_offset = x_position - last_x_;
	float y_offset = last_y_ - y_position;

	last_x_ = x_position;
	last_y_ = y_position;

	free_camera_.ProcessMouseMovement(x_offset, y_offset);
}

void Visualization::GetDeltaTime() {
	TimeFrame current_frame = std::chrono::high_resolution_clock::now();
	delta_time_ = std::chrono::duration_cast<std::chrono::duration<double>>(
			current_frame - last_frame_).count();
	last_frame_ = current_frame;
}

void Visualization::RenderAABB() {
	if (aabb_vao_ == 0) {
		constexpr float vertices[] = {
			// back face
			-1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 0.0f, // bottom-left
			 1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 0.0f, // bottom-right         
			 1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 1.0f, // top-right
			-1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 1.0f, // top-left
			1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 0.0f, // bottom-right
			1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 1.0f, // top-right
			-1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 1.0f, // top-left
			-1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 0.0f, // bottom-left
			// front face
			-1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 0.0f, // bottom-left
			 1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f, 0.0f, // bottom-right
			 1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f, 1.0f, // top-right
			 -1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 1.0f, // top-left
			 1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f, 0.0f, // bottom-right
			  1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f, 1.0f, // top-right
			-1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 1.0f, // top-left
			-1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 0.0f, // bottom-left
			// left face
			-1.0f, -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-left
			-1.0f, -1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 0.0f, 0.0f, // bottom-right
			-1.0f,  1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-right
			-1.0f,  1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 1.0f, 1.0f, // top-left
			-1.0f, -1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 0.0f, 0.0f, // bottom-right
			-1.0f,  1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-right
			-1.0f,  1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 1.0f, 1.0f, // top-left
			-1.0f, -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-left
			// right face
			1.0f, -1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 0.0f, 0.0f, // bottom-left
			 1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-right 
			1.0f,  1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 1.0f, 1.0f, // top-right   
			 1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-left
			 1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-right
			 1.0f,  1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 1.0f, 1.0f, // top-right         
			 1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-left
			 1.0f, -1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 0.0f, 0.0f, // bottom-left     
			// bottom face
			1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 1.0f, 0.0f, // bottom-left
			-1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 0.0f, // bottom-right
			-1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 1.0f, // top-right
			 1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 1.0f, 1.0f, // top-left
			-1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 0.0f, // bottom-right
			-1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 1.0f, // top-right
			 1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 1.0f, 1.0f, // top-left
			1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 1.0f, 0.0f, // bottom-left
			// top face
			-1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f, 0.0f, 0.0f,  // bottom-left
			 1.0f,  1.0f , 1.0f,  0.0f,  1.0f,  0.0f, 1.0f, 0.0f, // bottom-right
			  1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 1.0f, 1.0f, // top-right  
			-1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f, // top-left
			 1.0f,  1.0f , 1.0f,  0.0f,  1.0f,  0.0f, 1.0f, 0.0f, // bottom-right
			 1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 1.0f, 1.0f, // top-right     
			-1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f, // top-left
			-1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f, 0.0f, 0.0f  // bottom-left        
		};
		glGenVertexArrays(1, &aabb_vao_);
		glGenBuffers(1, &aabb_vbo_);
		glBindBuffer(GL_ARRAY_BUFFER, aabb_vbo_);
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
		glBindVertexArray(aabb_vao_);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(
				0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1,
							  3,
							  GL_FLOAT,
							  GL_FALSE,
							  8 * sizeof(float),
							  (void*)(3 * sizeof(float)));
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2,
							  2,
							  GL_FLOAT,
							  GL_FALSE,
							  8 * sizeof(float),
							  (void*)(6 * sizeof(float)));
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
	}

	glBindVertexArray(aabb_vao_);
	glDrawArrays(GL_LINES, 0, 48);
	glBindVertexArray(0);
}

}
}