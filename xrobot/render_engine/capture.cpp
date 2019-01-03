#include "capture.h"

namespace xrobot {
namespace render_engine {

Capture::Capture(const int resolution) : resolution_(resolution),
										 quad_vao_(0),
			  							 quad_vbo_(0),
										 raw_capture_fb_(0),
										 raw_capture_cubemap_(0) {
	
	capture_ = std::make_shared<RenderTarget>(resolution * 4, resolution);
	capture_->append_r_float();
	capture_->init();

	InitPBOs();
	InitShaders();
	InitCapture();
}

Capture::~Capture() {
	glDeleteFramebuffers(1, &raw_capture_fb_);
	glDeleteTextures(1, &raw_capture_cubemap_);
	glDeleteVertexArrays(1, &quad_vao_);
	glDeleteBuffers(1, &quad_vbo_);
	glDeleteBuffers(1, &lidar_pbo_);
}

void Capture::InitPBOs() {
	glGenBuffers(1, &lidar_pbo_);
	glBindBuffer(GL_PIXEL_PACK_BUFFER, lidar_pbo_);
	glBufferData(GL_PIXEL_PACK_BUFFER, 
			kLidarCaptureRes * kLidarCaptureRes * 4 * sizeof(float),
			nullptr, GL_STREAM_READ);
	glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
}

void Capture::InitCapture() {
	glGenFramebuffers(1, &raw_capture_fb_);
	glBindFramebuffer(GL_FRAMEBUFFER, raw_capture_fb_);

	glGenTextures(1, &raw_capture_cubemap_);
	glBindTexture(GL_TEXTURE_CUBE_MAP, raw_capture_cubemap_);

	for (GLuint i = 0; i < 6; ++i) {
		glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 
					 0, 
					 GL_DEPTH_COMPONENT24,
					 resolution_, 
					 resolution_, 
					 0, 
					 GL_DEPTH_COMPONENT,
					 GL_FLOAT, 
					 nullptr);
	}

	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
			raw_capture_cubemap_, 0);

	glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Capture::InitShaders() {
	std::string pwd = get_pwd(__FILE__);
	shaders_.resize((int) kCaptureShaders);

	shaders_[kCapture] = Shader(pwd + "/shaders/capture.vs",
							    pwd + "/shaders/capture.fs",
							    pwd + "/shaders/capture.gs",
							    -1);

	shaders_[kCubemap] = Shader(pwd + "/shaders/quad.vs",
							    pwd + "/shaders/cubemap.fs");
}

void Capture::Stitch(GLuint& rgb, Image<float>& lidar_image) {

	glBindFramebuffer(GL_FRAMEBUFFER, capture_->id()); 

	glClearColor(0, 0, 0, 1);

	glClear(GL_COLOR_BUFFER_BIT);

	auto cubemap  = shaders_[kCubemap];
	cubemap.use();

	glViewport(0, 0, resolution_, resolution_); 
	cubemap.setInt("dir", 5);
	cubemap.setInt("tex", 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_CUBE_MAP, raw_capture_cubemap_);
	RenderQuad();

	glViewport(resolution_, 0, resolution_, resolution_); 
	cubemap.setInt("dir", 0);
	cubemap.setInt("tex", 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_CUBE_MAP, raw_capture_cubemap_);
	RenderQuad();

	glViewport(resolution_ * 2, 0, resolution_, resolution_); 
	cubemap.setInt("dir", 4);
	cubemap.setInt("tex", 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_CUBE_MAP, raw_capture_cubemap_);
	RenderQuad();

	glViewport(resolution_ * 3, 0, resolution_, resolution_); 
	cubemap.setInt("dir", 1);
	cubemap.setInt("tex", 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_CUBE_MAP, raw_capture_cubemap_);
	RenderQuad();

	glReadBuffer(GL_COLOR_ATTACHMENT0);
	glBindBuffer(GL_PIXEL_PACK_BUFFER, lidar_pbo_);
	glReadPixels(0, 0, kLidarCaptureRes * 4, kLidarCaptureRes,
			GL_R, GL_FLOAT, 0);

	const int pixels = kLidarCaptureRes * kLidarCaptureRes * 4;

	glBindBuffer(GL_PIXEL_PACK_BUFFER, lidar_pbo_);
	GLubyte* pbo_ptr= (GLubyte*)glMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY);
	std::vector<float> temp(pbo_ptr, pbo_ptr + pixels);
	glUnmapBuffer(GL_PIXEL_PACK_BUFFER);

	lidar_image.data = temp;

	glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0); 

	rgb = capture_->texture_id(0);
}

void Capture::RenderCubemap(RenderWorld* world, Camera* camera) {
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glDepthFunc(GL_LEQUAL);

	glm::vec3 eye = camera->position_;
	glm::vec3 front = camera->front_;
	glm::vec3 up = camera->up_;


	glm::mat4 capture_projection = glm::perspective(glm::radians(90.0f),
			1.0f, 0.02f, 20.0f);

	glm::mat4 capture_views[] = 
	{
	   glm::lookAt(eye, eye + front, glm::vec3(0.0f, -1.0f,  0.0f)),
	   glm::lookAt(eye, eye - front, glm::vec3(0.0f, -1.0f,  0.0f)),
	   glm::lookAt(eye, eye + up, glm::vec3(0.0f,  0.0f,  1.0f)),
	   glm::lookAt(eye, eye - up, glm::vec3(0.0f,  0.0f, -1.0f)),
	   glm::lookAt(eye, eye + glm::cross(front, up), glm::vec3(0.0f, -1.0f,  0.0f)),
	   glm::lookAt(eye, eye - glm::cross(front, up), glm::vec3(0.0f, -1.0f,  0.0f))
	};

	auto capture = shaders_[kCapture];

	capture.use();
	for (GLuint i = 0; i < 6; ++i) {
		capture.setMat4("transform[" + std::to_string(i) + "]", 
				capture_projection * capture_views[i]);
	}

	glViewport(0, 0, resolution_, resolution_);

	glBindFramebuffer(GL_FRAMEBUFFER, raw_capture_fb_); 

	glClearColor(0, 0, 0, 1);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	Draw(world, capture);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);  

	glDisable(GL_DEPTH_TEST);
}

void Capture::Draw(RenderWorld* world, const Shader& shader) {
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

    world->robot_iteration_begin();
    while (world->has_next_robot()) {
        RenderBody* body = world->next_robot();
		if(body->is_hiding()) {
			continue;
        }
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

void Capture::RenderQuad() {
	if (quad_vao_ == 0) {
		constexpr float quadVertices[] = {
			-1.0f,  1.0f, 0.0f, 0.0f, 1.0f,
			-1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
			1.0f,  1.0f, 0.0f, 1.0f, 1.0f,
			1.0f, -1.0f, 0.0f, 1.0f, 0.0f,
		};
			
		glGenVertexArrays(1, &quad_vao_);
		glGenBuffers(1, &quad_vbo_);
		glBindVertexArray(quad_vao_);
		glBindBuffer(GL_ARRAY_BUFFER, quad_vbo_);
		glBufferData(GL_ARRAY_BUFFER,
					 sizeof(quadVertices),
					 &quadVertices,
					 GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(
				0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1,
							  2,
							  GL_FLOAT,
							  GL_FALSE,
							  5 * sizeof(float),
							  (void*)(3 * sizeof(float)));
	}
	glBindVertexArray(quad_vao_);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glBindVertexArray(0);
}

}
}
