// MIT License
//
// Copyright (c) 2017 José Villegas
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "vct.h"

namespace xrobot {
namespace render_engine {

VCT::VCT(const int width, const int height) : baked_(false),
											  scene_min_(glm::vec3(-2)),
											  scene_max_(glm::vec3(12)),
											  voxel_count_(0),
											  volume_dimension_(kVoxelDimension),
											  volume_grid_size_(0),
											  voxel_size_(0),
											  voxel_vao_(0),
											  voxel_albedo_(nullptr),
											  voxel_normal_(nullptr),
											  voxel_emissive_(nullptr),
											  voxel_radiance_(nullptr),
											  vct_out_(nullptr),
											  width_(width),
											  height_(height),
											  quad_vao_(0) {

	view_projection_matrix_[0] = glm::mat4(1);
	view_projection_matrix_[1] = glm::mat4(1);
	view_projection_matrix_[2] = glm::mat4(1);
	view_projection_matrix_inv_[0] = glm::mat4(1);
	view_projection_matrix_inv_[1] = glm::mat4(1);
	view_projection_matrix_inv_[2] = glm::mat4(1);

	for(GLint i = 0; i < 6; ++i)
		voxel_mipmaps_[i] = nullptr;

	vct_out_ = std::make_shared<RenderTarget>(width, height);
	vct_out_->append_rgba_float32();
	vct_out_->init();

	InitShaders();
	InitVoxelization();
}

VCT::~VCT() {
	glDeleteVertexArrays(1, &voxel_vao_);
	glDeleteVertexArrays(1, &quad_vao_);
	glDeleteBuffers(1, &quad_vbo_);
}

void VCT::Draw(RenderWorld* world, const Shader& shader) {
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

void VCT::InitShaders() {
	std::string pwd = get_pwd(__FILE__);
	shaders_.resize((int) kVCTShaders);
	shaders_[kVoxelize] = Shader(pwd+"/shaders/voxelize.vs",
								 pwd+"/shaders/voxelize.fs",
								 pwd+"/shaders/voxelize.gs",
								 -1);

	shaders_[kRadianceInjection] = Shader(pwd+"/shaders/inject.comp");

	shaders_[kRadiancePropagation] = Shader(pwd+"/shaders/prop.comp");

	shaders_[kMipMapBase] = Shader(pwd+"/shaders/mip_base.comp");

	shaders_[kMipMapVolume] = Shader(pwd+"/shaders/mip_vol.comp");

	shaders_[kClear] = Shader(pwd+"/shaders/clear.comp");

	shaders_[kConeTracing] = Shader(pwd+"/shaders/quad.vs",
									pwd+"/shaders/cone_trace.fs");
}

void VCT::InitVoxelization() {

	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	std::vector<unsigned char> clean_texture(pow(volume_dimension_, 3), 0);

	voxel_albedo_ = std::make_shared<Texture3D>(
			volume_dimension_, volume_dimension_, volume_dimension_);
	voxel_normal_ = std::make_shared<Texture3D>(
			volume_dimension_, volume_dimension_, volume_dimension_);
	voxel_emissive_ = std::make_shared<Texture3D>(
			volume_dimension_, volume_dimension_, volume_dimension_);
	voxel_radiance_ = std::make_shared<Texture3D>(
			volume_dimension_, volume_dimension_, volume_dimension_);

	for (GLuint i = 0; i < 6; ++i) {
		voxel_mipmaps_[i] = std::make_shared<Texture3D>(
														volume_dimension_ / 2,
														volume_dimension_ / 2,
														volume_dimension_ / 2,
														6);
	}

	glGenVertexArrays(1, &voxel_vao_);
}

void VCT::UpdateProjectionMatrices(RenderWorld* world) {

	float min_x, min_z, max_x, max_z;
	world->get_world_size(min_x, min_z, max_x, max_z);
	scene_min_ = glm::vec3(min_x - kScenePadding,
						   kSceneHeightMin,
						   min_z - kScenePadding);
	scene_max_ = glm::vec3(max_x + kScenePadding,
						   kSceneHeightMax,
						   max_z + kScenePadding);

	glm::vec3 center = (scene_min_ + scene_max_) * 0.5f;
	glm::vec3 axis_size = scene_max_ - scene_min_;
	volume_grid_size_ = glm::max(glm::max(axis_size.x, axis_size.y), axis_size.z);
	voxel_size_ = (float) volume_grid_size_ / (float) volume_dimension_;
	voxel_count_ = volume_dimension_ * volume_dimension_ * volume_dimension_;

	float half_size = volume_grid_size_ * 0.5f;
	glm::mat4 projection = glm::ortho(-half_size, half_size,
			-half_size, half_size, 0.0f, volume_grid_size_);


	view_projection_matrix_[0] = glm::lookAt(
			center + glm::vec3(half_size, 0.0f, 0.0f), center, glm::vec3(0.0f, 1.0f, 0.0f));
	view_projection_matrix_[1] = glm::lookAt(
			center + glm::vec3(0.0f, half_size, 0.0f),center, glm::vec3(0.0f, 0.0f, -1.0f));
	view_projection_matrix_[2] = glm::lookAt(
			center + glm::vec3(0.0f, 0.0f, half_size),center, glm::vec3(0.0f, 1.0f, 0.0f));

	int i = 0;
	for (auto &matrix : view_projection_matrix_) {
		matrix = projection * matrix;
		view_projection_matrix_inv_[i++] = glm::inverse(matrix);
	}
}

void VCT::VoxelizeScene(RenderWorld* world) {
	UpdateProjectionMatrices(world);
	ClearTextures();

	glColorMask(false, false, false, false);
	glViewport(0, 0, volume_dimension_, volume_dimension_);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);

	auto voxelize = shaders_[kVoxelize];
	voxelize.use();

	voxelize.setMat4("viewProjections[0]", view_projection_matrix_[0]);
	voxelize.setMat4("viewProjections[1]", view_projection_matrix_[1]);
	voxelize.setMat4("viewProjections[2]", view_projection_matrix_[2]);
	voxelize.setMat4("viewProjectionsI[0]", view_projection_matrix_inv_[0]);
	voxelize.setMat4("viewProjectionsI[1]", view_projection_matrix_inv_[1]);
	voxelize.setMat4("viewProjectionsI[2]", view_projection_matrix_inv_[2]);
	voxelize.setInt("volumeDimension", volume_dimension_);
	voxelize.setVec3("worldMinPoint", scene_min_);
	voxelize.setFloat("voxelScale", 1.0f / volume_grid_size_);
	voxelize.setFloat("ambient", ambient_);

	voxel_radiance_->clear();

	for (int i = 0; i < 6; ++i)
		voxel_mipmaps_[i]->clear();

	glBindImageTexture(0, voxel_albedo_->id(), 
			0, GL_TRUE, 0, GL_READ_WRITE, GL_R32UI);
	glBindImageTexture(1, voxel_normal_->id(), 
			0, GL_TRUE, 0, GL_READ_WRITE, GL_R32UI);
	glBindImageTexture(2, voxel_emissive_->id(), 
			0, GL_TRUE, 0, GL_READ_WRITE, GL_R32UI);

	Draw(world, voxelize);

	glMemoryBarrier(
		GL_TEXTURE_FETCH_BARRIER_BIT | 
		GL_SHADER_IMAGE_ACCESS_BARRIER_BIT |
		GL_ATOMIC_COUNTER_BARRIER_BIT
	);

	glColorMask(true, true, true, true);
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_BLEND);
}

void VCT::InjectRadiance(const DirectionalLight& light) {

	auto injection = shaders_[kRadianceInjection];
	injection.use();

	auto vSize = volume_grid_size_ / volume_dimension_;

	injection.setInt("numDirectionalLight", 1);
	injection.setVec3("directionalLight[0].diffuse", light.diffuse);
	injection.setVec3("directionalLight[0].direction", light.direction);
	
	injection.setFloat("traceShadowHit", kTraceShadowDistance);
	injection.setFloat("voxelSize", vSize);
	injection.setFloat("voxelScale", 1.0f / volume_grid_size_);
	injection.setVec3("worldMinPoint", scene_min_);
	injection.setInt("volumeDimension", volume_dimension_);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_3D, voxel_albedo_->id());
	glUniform1i(glGetUniformLocation(injection.id(), "voxelAlbedo"), 0);

	glBindImageTexture(1, voxel_normal_->id(),
			0, GL_TRUE, 0, GL_READ_WRITE, GL_RGBA8);
	glBindImageTexture(2, voxel_radiance_->id(),
			0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA8);
	glBindImageTexture(3, voxel_emissive_->id(),
			0, GL_TRUE, 0, GL_READ_ONLY, GL_RGBA8);

	auto workGroups = static_cast<unsigned>(
			glm::ceil(volume_dimension_ / 8.0f));

	glDispatchCompute(workGroups, workGroups, workGroups);

	glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT |
					GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

	GenerateMipmapBase();
	GenerateMipmapVolume();
	PropagateRadiance();
	GenerateMipmapBase();
	GenerateMipmapVolume();
}

void VCT::GenerateMipmapBase() {
	auto mipbase = shaders_[kMipMapBase];
	mipbase.use();

	float halfDimension = volume_dimension_ * 0.5f;

	mipbase.setInt("mipDimension", halfDimension);

	for (GLint i = 0; i < 6; ++i) {
		glBindImageTexture(i, voxel_mipmaps_[i]->id(),
				0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA8);   
	}

	glActiveTexture(GL_TEXTURE6);
	glBindTexture(GL_TEXTURE_3D, voxel_radiance_->id());
	glUniform1i(glGetUniformLocation(mipbase.id(), "voxelBase"), 6);

	auto workGroups = static_cast<unsigned int>(
			glm::ceil(halfDimension / 8.0f));

	glDispatchCompute(workGroups, workGroups, workGroups);

	glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT |
				 	GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
}

void VCT::GenerateMipmapVolume() {
	auto mipvol = shaders_[kMipMapVolume];
	auto mipDimension = volume_dimension_ * 0.25f;
	auto mipLevel = 0;

	mipvol.use();
	while(mipDimension >= 1) {

		auto volumeSize = glm::vec3(mipDimension, mipDimension, mipDimension);

		mipvol.setVec3("mipDimension", volumeSize);
		mipvol.setInt("mipLevel", mipLevel);

		for (GLint i = 0; i < 6; ++i) {
			glActiveTexture(GL_TEXTURE6 + i);
			glBindTexture(GL_TEXTURE_3D, voxel_mipmaps_[i]->id());
			std::string temp = "voxelMipmapSrc[" + std::to_string(i) + "]";
			glUniform1i(glGetUniformLocation(mipvol.id(), temp.c_str()), 6 + i); 

			glBindImageTexture(i, voxel_mipmaps_[i]->id(), 
					mipLevel + 1, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA8);                 
		}

		auto workGroups = static_cast<unsigned>(
				glm::ceil(mipDimension / 8.0f));

		glDispatchCompute(workGroups, workGroups, workGroups);

		glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | 
						GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

		mipLevel++;
		mipDimension /= 2;
	}
}

void VCT::PropagateRadiance() {
	auto propagation = shaders_[kRadiancePropagation];
	auto vSize = volume_grid_size_ / volume_dimension_;

	propagation.use();
	propagation.setInt("volumeDimension", volume_dimension_);
	propagation.setFloat("maxTracingDistanceGlobal", kPropagationDistance);

	glBindImageTexture(0, voxel_radiance_->id(),
			0, GL_TRUE, 0, GL_READ_WRITE, GL_RGBA8);

	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_3D, voxel_albedo_->id());
	glUniform1i(glGetUniformLocation(propagation.id(), "voxelAlbedo"), 1);

	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_3D, voxel_normal_->id());
	glUniform1i(glGetUniformLocation(propagation.id(), "voxelNormal"), 2);

	for (GLint i = 0; i < 6; ++i) {
		glActiveTexture(GL_TEXTURE3 + i);
		glBindTexture(GL_TEXTURE_3D, voxel_mipmaps_[i]->id());
		std::string temp = "voxelTexMipmap[" + std::to_string(i) + "]";
		glUniform1i(glGetUniformLocation(propagation.id(), temp.c_str()), 3 + i);
	}

	auto workGroups = static_cast<unsigned>(
			glm::ceil(volume_dimension_ / 8.0f));

	glDispatchCompute(workGroups, workGroups, workGroups);

	glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | 
					GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
}

void VCT::ClearTextures() {
	auto clear = shaders_[kClear];
	clear.use();

	glBindImageTexture(0, voxel_albedo_->id(), 
			0, GL_TRUE, 0, GL_READ_WRITE, GL_RGBA8);      
	glBindImageTexture(1, voxel_normal_->id(), 
			0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA8);
	glBindImageTexture(2, voxel_emissive_->id(), 
			0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA8);

	auto workGroups = static_cast<unsigned>(
			glm::ceil(volume_dimension_ / 8.0f));

	glDispatchCompute(workGroups, workGroups, workGroups);

	glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | 
					GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
}

void VCT::ConeTracing(RenderWorld* world,
                      Camera* camera,
                      DirectionalLight& light,
                      PSSM& shadow,
                      const float exposure,
                      std::shared_ptr<RenderTarget> gbuffer,
                      GLuint& out) {

	auto ct = shaders_[kConeTracing];

	glBindFramebuffer(GL_FRAMEBUFFER, vct_out_->id());

	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_BLEND);
	glColorMask(true, true, true, true);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glViewport(0, 0, width_, height_);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glm::mat4 projection = camera->GetProjectionMatrix();
	glm::mat4 view = camera->GetViewMatrix();

	// Upload Varyings
	ct.use();
	ct.setInt("shadowMode", 0);
	ct.setFloat("samplingFactor", kSampleFactor);
	ct.setFloat("aoFalloff", kAOFalloff);
	ct.setFloat("exposure", exposure);
	ct.setFloat("bounceStrength", kGIStrength);
	ct.setFloat("maxTracingDistanceGlobal", kConeTracingDistance);
	ct.setVec3("cameraPosition", camera->position_);
	ct.setMat4("invView", glm::inverse(view));
	ct.setMat4("projection", projection);

	// Upload PSSM

	ct.setVec4("direction", shadow.csm_uniforms.direction);
	ct.setVec4("options", shadow.csm_uniforms.options);
	ct.setInt("num_cascades", shadow.csm_uniforms.num_cascades);
	ct.setFloat("bias_scale", shadow.shadow_bias_scale);
	ct.setFloat("bias_clamp", shadow.shadow_bias_clamp);

	for (unsigned int i = 0; i < shadow.csm_uniforms.num_cascades - 1; ++i) {
		std::string far_bounds_str = "far_bounds[" + std::to_string(i) + "]";
		ct.setFloat(far_bounds_str.c_str(), shadow.csm_uniforms.far_bounds[i]);

		std::string mat_str = "texture_matrices[" + std::to_string(i) + "]";
		ct.setMat4(mat_str.c_str(), shadow.csm_uniforms.texture_matrices[i]);
	}

	for (unsigned int i = 0; i < shadow.csm_uniforms.num_cascades; ++i) {
		glActiveTexture(GL_TEXTURE15 + i);
		glBindTexture(GL_TEXTURE_2D, shadow.csm.shadow_map()[i]);
		std::string temp = "shadowMap[" + std::to_string(i) + "]";
		glUniform1i(glGetUniformLocation(ct.id(), temp.c_str()), 15 + i);
	}

	// Upload Directional Light

	ct.setInt("numDirectionalLight", 1);
	ct.setVec3("directionalLight[0].ambient", light.ambient);
	ct.setVec3("directionalLight[0].diffuse", light.diffuse);
	ct.setVec3("directionalLight[0].specular", light.specular);
	ct.setVec3("directionalLight[0].direction", light.direction);

	// Upload Voxels

	ct.setFloat("voxelScale", 1.0f / volume_grid_size_);
	ct.setVec3("worldMinPoint", scene_min_);
	ct.setVec3("worldMaxPoint", scene_max_);
	ct.setInt("volumeDimension", volume_dimension_);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_3D, voxel_normal_->id());
	glUniform1i(glGetUniformLocation(ct.id(), "voxelVisibility"), 0);

	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_3D, voxel_radiance_->id());
	glUniform1i(glGetUniformLocation(ct.id(), "voxelTex"), 1);

	for (GLuint i = 0; i < 6; ++i) {
		glActiveTexture(GL_TEXTURE9 + i);
		glBindTexture(GL_TEXTURE_3D, voxel_mipmaps_[i]->id());
		std::string temp = "voxelTexMipmap[" + std::to_string(i) + "]";
		glUniform1i(glGetUniformLocation(ct.id(), temp.c_str()), 9 + i);
	}

	// Upload G-Buffer

	gbuffer->active(ct.id(), "gAlbedo", 2, 5);
	gbuffer->active(ct.id(), "gNormal", 1, 6);
	gbuffer->active(ct.id(), "gPosition", 0, 7);

	RenderQuad();

	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	out = vct_out_->texture_id(0);
}

void VCT::BakeGI(RenderWorld* world,
                const DirectionalLight& light,
                const bool force) {
	if(force)
		baked_ = false;

	if(!baked_) {
		VoxelizeScene(world);
		InjectRadiance(light);
		baked_ = true;
	}
}

void VCT::RenderQuad() {
	if (quad_vao_ == 0) {
		constexpr float quadVertices[] = {
			// positions        // texture Coords
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