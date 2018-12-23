#include <unistd.h>

#include "glm/glm.hpp"

#include "render.h"
#include "render_utils.h"

namespace xrobot {
namespace render_engine {

Render::Render(const int width,
	           const int height,
	           const Profile profile,
	           const bool headless,
	           const int device) : profile_(profile),
								   lighting_(),
								   shaders_(0),
								   width_(width),
								   height_(height),
								   quad_vao_(0),
								   quad_vbo_(0) {
	
	// Create Context
	if(headless) {
		profile_.visualize = false;
		ctx_ = render_engine::CreateHeadlessContext(height, width, device);
	} else {
		profile_.visualize = true;
		ctx_ = render_engine::CreateContext(height, width);
	}
	
	// Create Framebuffers
	geomtry_pass_ = std::make_shared<RenderTarget>(width, height);
	geomtry_pass_->append_rgba_float32(); // View-Space Position Height
	geomtry_pass_->append_rgba_float16(); // View-Space Normal AO
	geomtry_pass_->append_rgba_uint8(); // Albedo Roughness
	geomtry_pass_->append_depth_readonly();
	geomtry_pass_->init();

	lighting_pass_ = std::make_shared<RenderTarget>(width, height);
	lighting_pass_->append_rgb_float32(); // RGB
	lighting_pass_->init();

	hud_pass_ = std::make_shared<RenderTarget>(width, height);
	hud_pass_->append_bgra_uint8(); // RGBD
	hud_pass_->init();

	frustum_ = std::make_shared<CameraFrustum>();

	// Init
	InitPBOs();
	InitShaders();
	InitVCT();
	InitPostProcessing();
	InitVisualization();
	InitLidarCapture();
}

Render::~Render() {
	glDeleteVertexArrays(1, &quad_vao_);
	glDeleteBuffers(1, &quad_vbo_);
	glDeleteBuffers(2, camera_pbos_);
	CloseContext(ctx_);
}

// --------------------------------------------------------------------------------------------

void Render::StepRender(RenderWorld* world) {
	if(!world->cameras_size()) return;
	Camera* camera = world->camera(0);

	GLuint out, capture;

	// Visualization
	RenderVisualization(world, camera);

	// Shadow Map
	RenderShadowMaps(world, camera);

	// Render G-Buffer
	RenderGeometryPass(world, camera);

	// Lighting
	GLuint color, position;
	RenderLightingPass(world, camera, color);
	position = geomtry_pass_->texture_id(0);

	// Post-Processing
	std::vector<GLuint> postprocessing_in;
	postprocessing_in.push_back(color);
	postprocessing_in.push_back(geomtry_pass_->texture_id(0));
	postprocessing_in.push_back(geomtry_pass_->texture_id(1));
	RenderPostProcessingPasses(camera, postprocessing_in, out);

	// HUD -> IO
	RenderHUD(world, camera, out, position);

	// Capture
	RenderLidarCapture(world, camera, capture);

	// Visualization
	glClear(GL_COLOR_BUFFER_BIT);
	if(profile_.visualize) {

		float width_quater  = width_ * 0.25f;
		float height_quater = height_ * 0.25f;

		RenderTexture(visualize_->GetTexture(), width_, height_);
		RenderTexture(hud_pass_->texture_id(0), width_quater, height_quater,
				10, 10, false);
		RenderTexture(hud_pass_->texture_id(0), width_quater, height_quater,
				width_quater + 20, 10, true);

		if(profile_.multirays) 
			RenderTexture(capture, 280, 70, 
					width_quater * 2 + 30, 10, true, 0);
	}

	// Swap
	ctx_->PollEvent();
	ctx_->SwapBuffer();
}

// --------------------------------------------------------------------------------------------

void Render::RenderTexture(const GLuint id, const int width, const int height,
            const int x, const int y, const bool display_alpha, 
            const int channel) {
	glViewport(x, y, width, height); 
	auto flat  = shaders_[kFlat];
	auto alpha = shaders_[kAlpha];

	if(display_alpha) {
		alpha.use();
		alpha.setFloat("channel", channel);
		alpha.setInt("tex", 0);
	} else {
		flat.use();
		flat.setInt("tex", 0);
	}

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, id);
	RenderQuad();
}

void Render::RenderHUD(RenderWorld* world, 
		Camera* camera, const GLuint rgb, const GLuint depth) {
	glDisable(GL_DEPTH_TEST);
	glViewport(0, 0, width_, height_);

	glm::mat4 projection = camera->GetProjectionMatrix();
	float near = camera->GetNear();
	float far = camera->GetFar();
	float aspect = camera->GetAspect();


	// Draw Inventory
	float scl = (float) (world->inventory_size_ - 1.0f) / world->inventory_size_;
	float icon_size = (float)width_ * 0.1f * scl;
	float inv_aspect = (float)height_ / width_;
	float start_u = 0.5f - (world->inventory_size_ * 0.05f * inv_aspect);
	float end_u = 0.5f + (world->inventory_size_ * 0.05f * inv_aspect);
	float draw_inventory = world->inventory_size_ < 0 ? 0 : 1;
	int highlight = world->get_highlight_center();

	auto hud  = shaders_[kHUD];

	glBindFramebuffer(GL_FRAMEBUFFER, hud_pass_->id());

	glClear(GL_COLOR_BUFFER_BIT); 

	hud.use();
	hud.setFloat("start_u", start_u * draw_inventory);
	hud.setFloat("end_u", end_u * draw_inventory);

	if(highlight == 2) {
		hud.setVec3("disk_color", glm::vec3(0.0f, 0.9f, 0.0f));
		hud.setFloat("alpha", 1.0f);
	} else if(highlight == 1){
		hud.setVec3("disk_color", glm::vec3(0.9f, 0.0f, 0.0f));
		hud.setFloat("alpha", 1.0f);
	} else if(highlight == 0){
		hud.setVec3("disk_color", glm::vec3(0.5f, 0.5f, 0.5f));
		hud.setFloat("alpha", 0.3f);
	} else {
		hud.setFloat("alpha", 0.0f);
	}

	hud.setMat4("projection", projection);
	hud.setFloat("near", near);
	hud.setFloat("far", far);
	hud.setFloat("aspect_ratio", aspect);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, rgb);
	hud.setInt("rgb", 0);

	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, depth);
	hud.setInt("depth", 1);

	RenderQuad();

	if(world->inventory_size_) {
		Shader flat = shaders_[kFlat];
		flat.use();

		for (int n = 0; n < world->icon_inventory_.size(); ++n)
		{
			std::string filename = world->icon_inventory_[n];

			int icon_texture_id = -1;
			auto has_find = world->icon_cache_.find(filename);
			if(has_find != world->icon_cache_.end()) {
				int id = world->icon_cache_[filename];

				if(id < 0) {
					std::size_t ext = filename.find_last_of(".");
					std::string icon_path = filename.substr(0, ext) + ".png";

					icon_texture_id = TextureFromFile(icon_path);
					world->icon_cache_[filename] = icon_texture_id;
				} else {
					icon_texture_id = id;
				}

				glActiveTexture(GL_TEXTURE0);
				glBindTexture(GL_TEXTURE_2D, icon_texture_id);
				glUniform1i(glGetUniformLocation(flat.id(), "tex"), 0);
				glViewport((start_u + 0.1f * scl * n) * 
						(float)width_, 0, icon_size, icon_size);
				RenderQuad();
			}
		}
	}

	const float half_height = height_ * 0.5f;
	const int   half_pixels = width_ * height_ * 2;

	glPixelStorei(GL_PACK_ALIGNMENT, 4);
	glReadBuffer(GL_COLOR_ATTACHMENT0);           
	glBindBuffer(GL_PIXEL_PACK_BUFFER, camera_pbos_[0]);
	glReadPixels(0, 0, width_, half_height, GL_BGRA, GL_UNSIGNED_BYTE, 0);
	glBindBuffer(GL_PIXEL_PACK_BUFFER, camera_pbos_[1]);
	glReadPixels(0, half_height, width_, half_height, GL_BGRA, GL_UNSIGNED_BYTE, 0);

	glBindBuffer(GL_PIXEL_PACK_BUFFER, camera_pbos_[0]);
	GLubyte* pbo_ptr= (GLubyte*)glMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY);
	std::vector<unsigned char> temp0(pbo_ptr, pbo_ptr + half_pixels);
	glUnmapBuffer(GL_PIXEL_PACK_BUFFER);

	glBindBuffer(GL_PIXEL_PACK_BUFFER, camera_pbos_[1]);
	pbo_ptr= (GLubyte*)glMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY);
	std::vector<unsigned char> temp1(pbo_ptr, pbo_ptr + half_pixels);
	glUnmapBuffer(GL_PIXEL_PACK_BUFFER);

	temp0.insert(temp0.end(), temp1.begin(), temp1.end());
	render_image_.data = temp0;

	glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Render::RenderLightingPass(RenderWorld* world, Camera* camera, GLuint& rgb) {
	if (!profile_.shading) {
		rgb   = geomtry_pass_->texture_id(2);
	} else if (!profile_.vct) {
		rgb   = lighting_pass_->texture_id(0);
		RenderNormalShading(camera);
	} else {
		vct_->BakeGI(world, dir_light_);
		vct_->ConeTracing(world, camera, dir_light_, shadow_,
				lighting_.exposure, geomtry_pass_, rgb);
	}
}

void Render::RenderNormalShading(Camera* camera) {
	glDisable(GL_DEPTH_TEST);
	glViewport(0, 0, width_, height_);

	auto blinn = shaders_[kBlinn];

	glm::mat4 projection = camera->GetProjectionMatrix();
	glm::mat4 view = camera->GetViewMatrix();
	glm::vec3 camera_position = camera->position_;

	glBindFramebuffer(GL_FRAMEBUFFER, lighting_pass_->id());

	glClear(GL_COLOR_BUFFER_BIT); 

	blinn.use();
	geomtry_pass_->active(blinn.id(), "gPosition", 0, 0);
	geomtry_pass_->active(blinn.id(), "gNormal", 1, 1);
	geomtry_pass_->active(blinn.id(), "gAlbedo", 2, 2);

	blinn.setMat4("invView", glm::inverse(view));
	blinn.setMat4("projection", projection);
	blinn.setVec3("cameraPosition", camera_position);
	blinn.setFloat("exposure", lighting_.exposure);

	blinn.setVec3("light.ambient", dir_light_.ambient);
	blinn.setVec3("light.diffuse", dir_light_.diffuse);
	blinn.setVec3("light.specular", dir_light_.specular);
	blinn.setVec3("light.direction", dir_light_.direction);
	blinn.setFloat("use_shadow", (float) profile_.shadow);

	if(profile_.shadow) {
		blinn.setVec4("direction", shadow_.csm_uniforms.direction);
		blinn.setVec4("options", shadow_.csm_uniforms.options);
		blinn.setInt("num_cascades", shadow_.csm_uniforms.num_cascades);
		blinn.setFloat("bias_scale", shadow_.shadow_bias_scale);
		blinn.setFloat("bias_clamp", shadow_.shadow_bias_clamp);

		for (unsigned int i = 0; i < shadow_.csm_uniforms.num_cascades - 1; ++i) {
			std::string far_bounds_str = "far_bounds[" + std::to_string(i) + "]";
			blinn.setFloat(far_bounds_str.c_str(), shadow_.csm_uniforms.far_bounds[i]);

			std::string mat_str = "texture_matrices[" + std::to_string(i) + "]";
			blinn.setMat4(mat_str.c_str(), shadow_.csm_uniforms.texture_matrices[i]);
		}

		for (unsigned int i = 0; i < shadow_.csm_uniforms.num_cascades; ++i) {
			glActiveTexture(GL_TEXTURE3 + i);
			glBindTexture(GL_TEXTURE_2D, shadow_.csm.shadow_map()[i]);
			std::string temp = "shadowMap[" + std::to_string(i) + "]";
			glUniform1i(glGetUniformLocation(blinn.id(), temp.c_str()), 3 + i);
		}
	}

	RenderQuad();

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Render::RenderGeometryPass(RenderWorld* world, Camera* camera) {
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glViewport(0, 0, width_, height_);
	glClearColor(lighting_.bg.x, lighting_.bg.y, lighting_.bg.z, 1.0f); 

	auto gemotry = shaders_[kGeometry];
	
	glm::mat4 projection = camera->GetProjectionMatrix();
	glm::mat4 view = camera->GetViewMatrix();
	glm::vec3 camera_position = camera->position_;

	glBindFramebuffer(GL_FRAMEBUFFER, geomtry_pass_->id());

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	gemotry.use();
	gemotry.setMat4("projection", projection);
	gemotry.setMat4("view", view);
	gemotry.setVec3("viewPos", camera_position);

	frustum_->Update(camera);

	Draw(world, gemotry, true);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Render::Draw(RenderWorld* world, const Shader& shader, const bool cull) {
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
			shader.setVec3("urdf_color", glm::vec3(transform->color));
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

			if(cull) {
				glm::vec3 root_aabb_min, root_aabb_max;
				root->GetAABB(root_aabb_min, root_aabb_max);
				if(!frustum_->Intersect(root_aabb_min, root_aabb_max)) {
					continue;
				}
			}

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

// --------------------------------------------------------------------------------------------

void Render::InitPostProcessing() {
	if(profile_.ao) 
		ssao_ = std::make_shared<SSAO>(width_, height_);

	if(profile_.ssr) 
		ssr_ = std::make_shared<SSR>(width_, height_);
	
	if(profile_.fxaa) 
		fxaa_ = std::make_shared<FXAA>(width_, height_);

	composite_ = std::make_shared<Composite>(width_, height_);
}

void Render::RenderPostProcessingPasses(Camera* camera, 
		const std::vector<GLuint>& in, GLuint& out) {
	// SSAO
	std::vector<GLuint> ssao_out(1);
	if (profile_.ao) {
		glm::mat4 view = camera->GetViewMatrix();
		glm::mat4 projection = camera->GetProjectionMatrix();

		std::vector<GLuint> ssao_in;
		ssao_in.push_back(in[1]);
		ssao_in.push_back(in[2]);
		ssao_->Draw(ssao_in, ssao_out, view, projection);
	}

	// SSR
	std::vector<GLuint> ssr_out(1);
	if (profile_.ssr) {
		glm::mat4 view = camera->GetViewMatrix();
		glm::mat4 projection = camera->GetProjectionMatrix();

		std::vector<GLuint> ssr_in;
		ssr_in.push_back(in[0]);
		ssr_in.push_back(in[1]);
		ssr_in.push_back(in[2]);
		ssr_->Draw(ssr_in, ssr_out, view, projection);
	}

	// Composite
	std::vector<GLuint> composite_out(1);
	std::vector<GLuint> composite_in;
	composite_in.push_back(in[0]);
	composite_in.push_back(ssao_out[0]);
	composite_in.push_back(ssr_out[0]);
	composite_->Draw(composite_in, composite_out, 
			profile_.ao, profile_.ssr, profile_.shading);

	// FXAA
	std::vector<GLuint> fxaa_out(1);
	if(profile_.fxaa) {
		std::vector<GLuint> fxaa_in;
		fxaa_in.push_back(composite_out[0]);
		fxaa_->Draw(fxaa_in, fxaa_out);

		out = fxaa_out[0];
	} else {
		out = composite_out[0];
	}
}

// --------------------------------------------------------------------------------------------

void Render::InitShadowMaps(const float fov) {
	if(!profile_.shadow) 
		return;

	if(shadow_.first_run) {
		shadow_.csm_uniforms.direction = glm::vec4(-1.0f * dir_light_.direction, 0.0f);
		shadow_.csm_uniforms.direction = glm::normalize(shadow_.csm_uniforms.direction);
		shadow_.csm_uniforms.options.x = 1;
		shadow_.csm_uniforms.options.y = 0;
		shadow_.csm_uniforms.options.z = 1;

		shadow_.csm.initialize(
			shadow_.pssm_lamda,
			shadow_.near_offset,
			shadow_.cascade_count,
			shadow_.shadow_map_size,
			fov,
			width_,
			height_,
			shadow_.csm_uniforms.direction
		);

		shadow_.first_run = false;
	}
}

void Render::RenderShadowMaps(RenderWorld* world, Camera* camera) {
	if(!profile_.shadow) 
		return;

	InitShadowMaps(camera->GetFOV());

	shadow_.csm.update(camera, shadow_.csm_uniforms.direction);

	shadow_.csm_uniforms.num_cascades = shadow_.csm.frustum_split_count();
	for (int i = 0; i < shadow_.csm_uniforms.num_cascades; ++i) {
		shadow_.csm_uniforms.far_bounds[i] = shadow_.csm.far_bound(i);
		shadow_.csm_uniforms.texture_matrices[i] = shadow_.csm.texture_matrix(i);
	}

	glEnable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glCullFace(GL_FRONT);

	auto pssm = shaders_[kPSSM];
	pssm.use();

	glm::mat4 projection = camera->GetProjectionMatrix();
	glm::mat4 view = camera->GetViewMatrix();

	for (int i = 0; i < shadow_.csm.frustum_split_count(); ++i) {
		pssm.setMat4("projection", projection);
		pssm.setMat4("view", view);
		pssm.setMat4("crop", shadow_.csm.split_view_proj(i));

		glBindFramebuffer(GL_FRAMEBUFFER, shadow_.csm.framebuffers()[i]);
		glViewport(0, 0, shadow_.csm.shadow_map_size(), shadow_.csm.shadow_map_size());

		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		Draw(world, pssm);

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}

	glCullFace(GL_BACK);
}

// --------------------------------------------------------------------------------------------

void Render::InitVCT() {
	if(profile_.vct) {
		if(!profile_.shadow) {
			printf("[Renderer] VCT requires shadow mapping!\n");
			exit(-1);
		}
		vct_ = std::make_shared<VCT>(width_, height_);
	}
}

void Render::BakeGI() {
	if(profile_.vct) 
		vct_->ForceBakeOnNextFrame();
}

// --------------------------------------------------------------------------------------------

void Render::InitVisualization() {
	if(profile_.visualize) {
		visualize_ = std::make_shared<Visualization>(width_, height_, ctx_);
	}
}

void Render::RenderVisualization(RenderWorld* world, Camera *camera) {
	if(profile_.visualize) {
		visualize_->Visualize(world, camera, capture_->GetRawCubeMap());
	}
}

// --------------------------------------------------------------------------------------------

void Render::UpdateRay(const int offset, const glm::vec3 from, const glm::vec3 to) {
	if(profile_.visualize && visualize_)
		visualize_->UpdateRay(offset, from, to);
}

void Render::InitDrawBatchRays(const int rays) {
	if(profile_.visualize && visualize_) {
		visualize_->InitDrawBatchRays(rays);
	}
}

// --------------------------------------------------------------------------------------------

void Render::InitLidarCapture() {
	if(profile_.multirays) {
		capture_ = std::make_shared<Capture>(kLidarCaptureRes);
	}
}

void Render::RenderLidarCapture(RenderWorld* world, Camera* camera, GLuint& depth) {
	if(profile_.multirays) {
		capture_->RenderCubemap(world, camera);
		capture_->Stitch(depth, lidar_image_);
	}
}

// --------------------------------------------------------------------------------------------


void Render::InitPBOs() {
	glGenBuffers(2, camera_pbos_);
	glBindBuffer(GL_PIXEL_PACK_BUFFER, camera_pbos_[0]);
	glBufferData(GL_PIXEL_PACK_BUFFER, 
				 width_ * height_ * 2 * sizeof(unsigned char),
				 nullptr, GL_STREAM_READ);
	glBindBuffer(GL_PIXEL_PACK_BUFFER, camera_pbos_[1]);
	glBufferData(GL_PIXEL_PACK_BUFFER, 
				 width_ * height_ * 2 * sizeof(unsigned char),
				 nullptr, GL_STREAM_READ);
	glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
}

void Render::InitShaders() {
	std::string pwd = get_pwd(__FILE__);
	shaders_.resize((int) kRenderShaders);

	shaders_[kGeometry] = Shader(pwd + "/shaders/geometry.vs",
								 pwd + "/shaders/geometry.fs");

	shaders_[kPSSM] = Shader(pwd + "/shaders/pssm.vs",
							 pwd + "/shaders/pssm.fs");

	shaders_[kAlpha] = Shader(pwd + "/shaders/quad.vs",
							  pwd + "/shaders/alpha.fs");

	shaders_[kFlat] = Shader(pwd + "/shaders/quad.vs",
							 pwd + "/shaders/flat.fs");

	shaders_[kBlinn] = Shader(pwd + "/shaders/quad.vs",
							  pwd + "/shaders/blinn.fs");

	shaders_[kHUD] = Shader(pwd + "/shaders/quad.vs",
							pwd + "/shaders/hud.fs");
}

void Render::RenderQuad() {
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

}}

								   