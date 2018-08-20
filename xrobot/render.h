#ifndef RENDER_H_
#define RENDER_H_

#include <stdio.h>
#include <string.h>

#include <vector>
#include <ctime>
#include <ratio>
#include <chrono>

#include "glm/gtc/type_ptr.hpp"

#include "gl_context.h"
#include "world.h"
#include "map.h"

using namespace std::chrono;
using namespace glm;

namespace xrobot {

typedef high_resolution_clock::time_point TimeFrame;

enum RenderMode
{
    kNormal,
    kDebug
};

enum ShaderTypes
{
    kLambert = 0,
    kDepth = 1,
    kColor = 2,
    kLine = 3,
    kCrossHair = 4,
    kMultiply = 5
};

inline glm::mat4 ScalarsToMat4(btScalar* matrix) {
    return glm::mat4(
        matrix[0], matrix[1], matrix[2], matrix[3],
        matrix[4], matrix[5], matrix[6], matrix[7],
        matrix[8], matrix[9], matrix[10], matrix[11],
        matrix[12], matrix[13], matrix[14], matrix[15]
    );
}

inline void IdToColor(const int id, int& r, int& g, int& b)
{
    r = (id & 0x000000FF) >> 0;
    g = (id & 0x0000FF00) >> 8;
    b = (id & 0x00FF0000) >> 16;
}

inline int ColorToId(const int r, const int g, const int b)
{
    return r + g * 256 + b * 256 * 256;
}

struct Image {
    Image() : data(0), camera_id(-1) {}

    vector<unsigned char> data;
    int camera_id;
};

class Render
{
public:
    // Framerate
    // This Could Be Removed In the Future...
    float current_framerate_;
    float max_framerate_;
    double avg_framerate_;
    double all_rendered_frames_;

    // Free Camera
    Camera free_camera_;
    float last_x_, last_y_;
    float delta_time_;
    bool first_mouse_;
    TimeFrame last_frame_;

    // Buffers
    GLuint crosshair_vao_;
    GLuint crosshair_vbo_;
    GLuint cube_vao_;
    GLuint cube_vbo_;
    GLuint quad_vao_;
    GLuint quad_vbo_;

    // Shaders and Other Basic Rendering Variables
	std::vector<Shader> shaders_;
	float width_, height_;
	vec3 background_color_;
    GLContext * ctx_;    

    // Frame Buffers
    FBO * free_camera_framebuffer_;
    std::vector<FBO*> camera_framebuffer_list_;
    std::vector<FBO*> multiplied_framebuffer_list_;

    // Cube Map
    //
    // Load Cube Map is Not Intuitive for User!
    // This Will Be Replaced by Environment Map in PBR Version!
    //
    GLuint reflection_map_;

    // I/O
    int pixel_buffer_index_;
    int pixel_buffer_next_index_;
    GLuint pixel_buffers_[2];
    int num_frames_;
    std::vector<Image> img_buffers_;

    
    void ProcessInput()
    {
        if(ctx_->GetKeyPressESC())
            ctx_->SetWindowShouldClose();
        if(ctx_->GetKeyPressW()) 
            free_camera_.ProcessKeyboard(FORWARD, delta_time_);
        if(ctx_->GetKeyPressS())
            free_camera_.ProcessKeyboard(BACKWARD, delta_time_);
        if(ctx_->GetKeyPressA())
            free_camera_.ProcessKeyboard(LEFT, delta_time_);
        if(ctx_->GetKeyPressD())
            free_camera_.ProcessKeyboard(RIGHT, delta_time_);
        if(ctx_->GetKeyPressQ())
            free_camera_.ProcessKeyboard(UP, delta_time_);
        if(ctx_->GetKeyPressE())
            free_camera_.ProcessKeyboard(DOWN, delta_time_);
    }

    void ProcessMouse()
    {
        float x_position, y_position;
        ctx_->GetMouse(x_position, y_position);

        if(first_mouse_)
        {
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

    void RenderMarker()
    {
        if(crosshair_vao_ == 0)
        {
            constexpr float vertices[] = {
                -0.075, 0, 0,
                0.075, 0, 0,
                0, -0.1, 0,
                0, 0.1, 0
            };

            glGenVertexArrays(1, &crosshair_vao_);
            glGenBuffers(1, &crosshair_vbo_);
            glBindBuffer(GL_ARRAY_BUFFER, crosshair_vbo_);
            glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
            glBindVertexArray(crosshair_vao_);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            glBindVertexArray(0);
        }

        glBindVertexArray(crosshair_vao_);
        glDrawArrays(GL_LINES, 0, 4);
        glBindVertexArray(0);
    }

    void RenderCube()
    {
        if (cube_vao_ == 0)
        {
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
            glGenVertexArrays(1, &cube_vao_);
            glGenBuffers(1, &cube_vbo_);
            glBindBuffer(GL_ARRAY_BUFFER, cube_vbo_);
            glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
            glBindVertexArray(cube_vao_);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
            glEnableVertexAttribArray(1);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
            glEnableVertexAttribArray(2);
            glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            glBindVertexArray(0);
        }
        // render Cube
        glBindVertexArray(cube_vao_);
        glDrawArrays(GL_LINES, 0, 48);
        glBindVertexArray(0);
    }

    void RenderQuad()
    {
        if (quad_vao_ == 0)
        {
            constexpr float quadVertices[] = {
                // positions        // texture Coords
                -1.0f,  1.0f, 0.0f, 0.0f, 1.0f,
                -1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
                1.0f,  1.0f, 0.0f, 1.0f, 1.0f,
                1.0f, -1.0f, 0.0f, 1.0f, 0.0f,
            };
            // setup plane VAO
            glGenVertexArrays(1, &quad_vao_);
            glGenBuffers(1, &quad_vbo_);
            glBindVertexArray(quad_vao_);
            glBindBuffer(GL_ARRAY_BUFFER, quad_vbo_);
            glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
            glEnableVertexAttribArray(1);
            glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
        }
        glBindVertexArray(quad_vao_);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glBindVertexArray(0);
    }

    void InitFramebuffers(const unsigned int num_cameras)
    {
        camera_framebuffer_list_.resize(num_cameras);
        multiplied_framebuffer_list_.resize(num_cameras);
        for(unsigned int i = 0; i < num_cameras; ++i)
        {
            camera_framebuffer_list_[i] = new FBO(width_, height_, true, true);
            multiplied_framebuffer_list_[i] = new FBO(width_, height_, false, false);
        }
    }

    void DrawRootAABB(World * world, Shader shader)
    {
        glDisable( GL_POLYGON_OFFSET_FILL );

        for (unsigned int i = 0; i < world->robot_list_.size(); i++)
        {
            Object * part = world->robot_list_[i]->root_part_;
            if (part && !world->robot_list_[i]->recycle_)
            {
                vec3 aabb_min, aabb_max;
                part->GetAABB(aabb_min, aabb_max);
                shader.setVec3("aabbMin", aabb_min);
                shader.setVec3("aabbMax", aabb_max);
                RenderCube();
            }
        }

        glPolygonOffset(1.0f, 1.0f);
        glEnable( GL_POLYGON_OFFSET_FILL );
    }

    void DrawEmptyAABB(Map * map, Shader shader)
    {
        glDisable( GL_POLYGON_OFFSET_FILL );

        for (int i = 0; i < map->empty_map_.size(); ++i)
        {
            vec3 aabb_min = map->empty_map_[i].first;
            vec3 aabb_max = map->empty_map_[i].second;

            shader.setVec3("aabbMin", aabb_min);
            shader.setVec3("aabbMax", aabb_max);
            RenderCube();
        }

        glPolygonOffset(1.0f, 1.0f);
        glEnable( GL_POLYGON_OFFSET_FILL );
    }

    void DrawRoomAABB(Map * map, Shader shader)
    {
        glDisable( GL_POLYGON_OFFSET_FILL );

        for (int i = 0; i < map->sections_map_.size(); ++i)
        {
            vec3 aabb_min = map->sections_map_[i].first;
            vec3 aabb_max = map->sections_map_[i].second;

            shader.setVec3("aabbMin", aabb_min);
            shader.setVec3("aabbMax", aabb_max);
            RenderCube();
        }

        glPolygonOffset(1.0f, 1.0f);
        glEnable( GL_POLYGON_OFFSET_FILL );
    }

    void Draw(World * world, Shader shader)
    {
    	for (unsigned int i = 0; i < world->robot_list_.size(); ++i)
        {
        	// Root
        	Object * root = world->robot_list_[i]->root_part_;

        	if (root && !world->robot_list_[i]->recycle_)
			{
				std::vector<ModelData *> model_data = root->model_list_;
				std::vector<OriginTransformation *> transform_list = root->transformation_list_;
                int bullet_handle = root->bullet_handle_;

                int id_r, id_g, id_b;
                IdToColor(bullet_handle, id_r, id_g, id_b);
                glm::vec3 id_color((float)id_r/255.0f, (float)id_g/255.0f, (float)id_b/255.0f);

				for (unsigned int model_index = 0; model_index < model_data.size(); model_index++)
				{
					ModelData * model = model_data[model_index];
					OriginTransformation * transform = transform_list[model_index];

					glm::mat4 translate = TransformToMat4(root->object_position_);
					glm::mat4 local_frame = glm::inverse(TransformToMat4(root->object_local_inertial_frame_));
					glm::mat4 scale =  glm::scale(glm::mat4(1),transform->local_scale);

					shader.setMat4("state", translate * local_frame);
					shader.setMat4("scale",scale);
                    shader.setFloat("flip", transform->flip);
                    shader.setVec3("id_color", id_color);
				  	model->Draw(shader);
				}
			}
            else
            {
                continue;
            }

			// Parts
			for (int j = 0; j < world->robot_list_[i]->other_parts_list_.size(); j++)
			{
				Object * part = world->robot_list_[i]->other_parts_list_[j];
				if(part)
				{
                    std::vector<ModelData *> model_data = part->model_list_;
                    std::vector<OriginTransformation *> transform_list = part->transformation_list_;
                    int bullet_handle = part->bullet_handle_;

                    int id_r, id_g, id_b;
                    IdToColor(bullet_handle, id_r, id_g, id_b);
                    glm::vec3 id_color((float)id_r/255.0f, (float)id_g/255.0f, (float)id_b/255.0f);

                    for (unsigned int model_index = 0; model_index < model_data.size(); model_index++)
                    {
                        ModelData * model = model_data[model_index];
                        OriginTransformation * transform = transform_list[model_index];

                        glm::mat4 translate = TransformToMat4(part->object_position_);
                        glm::mat4 local_frame = glm::inverse(TransformToMat4(part->object_local_inertial_frame_));
                        glm::mat4 scale = glm::scale(glm::mat4(1),transform->local_scale);

						shader.setMat4("state", translate * local_frame);
						shader.setMat4("scale", scale);
						shader.setMat4("model", transform->origin);	
                        shader.setFloat("flip", transform->flip);
                        shader.setVec3("id_color", id_color);
					 	model->Draw(shader);
					}
				}
			}

        }

    }

    inline void GetDeltaTime()
    {
        TimeFrame current_frame = high_resolution_clock::now();
        delta_time_ = (float)(duration_cast<duration<double>>(current_frame - last_frame_)).count();
        last_frame_ = current_frame;
    }

    inline void GetFrameRate()
    {
        current_framerate_ = 1.0f / delta_time_;
        avg_framerate_ += current_framerate_;
        all_rendered_frames_ += 1;
        max_framerate_ = glm::max(max_framerate_, current_framerate_);

        std::string framerate_str = "Cur: " + to_string((int)current_framerate_)
            + " Avg: " + to_string((int)(avg_framerate_ / all_rendered_frames_))
            + " Max: " + to_string((int)max_framerate_);
        ctx_->SetTitle(framerate_str.c_str());
    }

    inline void StepRenderFreeCamera(Map * map)
    {
        ProcessMouse();
        ProcessInput();

        glm::mat4 projection = free_camera_.GetProjectionMatrix();
        glm::mat4 view = free_camera_.GetViewMatrix();

        Shader lambert_shader = shaders_[kLambert];
        Shader line_shader = shaders_[kLine];

        glBindFramebuffer(GL_FRAMEBUFFER, free_camera_framebuffer_->frameBuffer);
        glEnable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
        lambert_shader.use();
        lambert_shader.setMat4("projection", projection);
        lambert_shader.setMat4("view", view);
        lambert_shader.setVec3("camPos", free_camera_.Position);

        glActiveTexture(GL_TEXTURE5);
        glBindTexture(GL_TEXTURE_CUBE_MAP, reflection_map_);
        glUniform1i(glGetUniformLocation(lambert_shader.id_, "cmap"), 5);

        Draw(map->world_, lambert_shader);

        line_shader.use();
        line_shader.setMat4("projection", projection);
        line_shader.setMat4("view", view);

        line_shader.setVec3("color", vec3(1,0,0));
        DrawRootAABB(map->world_, line_shader);

        line_shader.setVec3("color", vec3(0,1,0));
        DrawEmptyAABB(map, line_shader);

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    inline void StepRenderAllCameras(Map * map, std::vector<int>& picks, const bool color_pick)
    {
        for (int i = 0; i < camera_framebuffer_list_.size(); ++i)
        {
            Shader lambert_shader = shaders_[kLambert];

            glBindFramebuffer(GL_FRAMEBUFFER, camera_framebuffer_list_[i]->frameBuffer);
            glClearColor(1,1,1,0);      
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            Camera * camera = map->world_->camera_list_[i];
            glm::mat4 projection = camera->GetProjectionMatrix();
            glm::mat4 view = camera->GetViewMatrix();

            lambert_shader.use();
            lambert_shader.setMat4("projection", projection);
            lambert_shader.setMat4("view", view);
            lambert_shader.setVec3("camPos", camera->Position);
            Draw(map->world_, lambert_shader);

            // crosshairShader.use();
            // crosshairShader.setVec3("color", vec3(0,0,1));
            // renderMarker();
            // lineShader.setVec3("color", vec3(0,0,1));
            // drawRoomAABB(s, lineShader);

            if(color_pick)
            {
                glFlush();
                glFinish();
                glReadBuffer(GL_COLOR_ATTACHMENT1); 
                glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

                unsigned char data[4];
                glReadPixels(width_ / 2, height_ / 2, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, data);

                picks[i] = ColorToId(data[0], data[1], data[2]);
            }

            glBindFramebuffer(GL_FRAMEBUFFER, 0);
        }
    }

    inline void StepRenderGetDepthAllCameras()
    {
        glDisable(GL_DEPTH_TEST);
        img_buffers_.clear();

        for (int i = 0; i < camera_framebuffer_list_.size(); ++i)
        {     
            glBindFramebuffer(GL_FRAMEBUFFER, multiplied_framebuffer_list_[i]->frameBuffer);
            glViewport(0, 0, width_, height_);
            glClearColor(background_color_.x, background_color_.y, background_color_.z, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            Shader multiply_shader = shaders_[kMultiply];

            multiply_shader.use();
            camera_framebuffer_list_[i]->ActivateAsTexture(multiply_shader.id_, "tex", 0);
            camera_framebuffer_list_[i]->ActivateDepthAsTexture(multiply_shader.id_, "dep", 1);
            RenderQuad();

            pixel_buffer_index_ = (pixel_buffer_index_ + 1) % 2;
            pixel_buffer_next_index_ = (pixel_buffer_index_ + 1) % 2;
            
            glReadBuffer(GL_COLOR_ATTACHMENT0);            
            if(num_frames_ * (int)multiplied_framebuffer_list_.size() < 2)
            {
                glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, pixel_buffers_[pixel_buffer_index_]);
                glReadPixels(0, 0, width_, height_, GL_RGBA, GL_UNSIGNED_BYTE, 0);
                
                GLubyte* ptr = (GLubyte*)glMapBufferARB(GL_PIXEL_PACK_BUFFER_ARB,GL_READ_ONLY_ARB);
                if(ptr)
                {
                    Image image_temp;
                    vector<unsigned char> temp(ptr, ptr + (int)(width_ * height_ * 4));
                    image_temp.data = temp;
                    image_temp.camera_id = i;
                    img_buffers_.push_back(image_temp);
                    
                    glUnmapBufferARB(GL_PIXEL_PACK_BUFFER_ARB);
                }
                
            } else {

                glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, pixel_buffers_[pixel_buffer_next_index_]);
                GLubyte* ptr = (GLubyte*)glMapBufferARB(GL_PIXEL_PACK_BUFFER_ARB,GL_READ_ONLY_ARB);
                if(ptr)
                {
                    Image image_temp;
                    vector<unsigned char> temp(ptr, ptr + (int)(width_ * height_ * 4));
                    image_temp.data = temp;
                    image_temp.camera_id = i;
                    img_buffers_.push_back(image_temp);
                    
                    glUnmapBufferARB(GL_PIXEL_PACK_BUFFER_ARB);
                }
                glReadPixels(0, 0, width_, height_, GL_RGBA, GL_UNSIGNED_BYTE, 0);
            }
            glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, 0);

            glBindFramebuffer(GL_FRAMEBUFFER, 0);
        }
    }
    
    inline void Visualization()
    {
        glDisable(GL_BLEND); 
        glViewport(0, 0, width_, height_);
        glClearColor(background_color_.x, background_color_.y, background_color_.z, 1.0f); 
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        shaders_[kColor].use();
        free_camera_framebuffer_->ActivateAsTexture(shaders_[kColor].id_, "tex", 0);
        RenderQuad();     

        for (int i = 0; i < multiplied_framebuffer_list_.size(); ++i)
        {
            glViewport(width_ * 1, i * height_, width_, height_);
            shaders_[kColor].use();
            multiplied_framebuffer_list_[i]->ActivateAsTexture(shaders_[kColor].id_, "tex", 0);
            RenderQuad();
        }

        for (int i = 0; i < multiplied_framebuffer_list_.size(); ++i)
        {
            glViewport(width_ * 2, i * height_, width_, height_);
            shaders_[kDepth].use();
            camera_framebuffer_list_[i]->ActivateDepthAsTexture(shaders_[kDepth].id_, "tex", 0);
            RenderQuad();
        }
    }

    int StepRender(Map * map, int pick_camera_id = -1)
    {
        GetDeltaTime();
        GetFrameRate();

        glEnable(GL_LINE_SMOOTH);
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
		glViewport(0, 0, width_, height_);
        glClearColor(background_color_.x, background_color_.y, background_color_.z, 1.0f);        

        StepRenderFreeCamera(map);

        int pick_result = -1;
        std::vector<int> picks(camera_framebuffer_list_.size(), -1);
        if(pick_camera_id >= 0)
        {
            StepRenderAllCameras(map, picks, true);
            pick_result = picks[pick_camera_id];
        }
        else
        {
            StepRenderAllCameras(map, picks, false);
        }

        StepRenderGetDepthAllCameras();

        Visualization();
		
        num_frames_++;
        return pick_result;
	}

    // This Function Has To Be Called After StepRender(...)!
    std::vector<Image> GetRenderedImages()
    {
        return img_buffers_;
    }
    
    void InitShaders()
    {
        shaders_.resize(6);
        shaders_[kLambert] = Shader("./shaders/lambert.vs",
                               "./shaders/lambert.fs");
        
        shaders_[kDepth] = Shader("./shaders/quad.vs",
                              "./shaders/depth.fs");
        
        shaders_[kColor] = Shader("./shaders/quad.vs",
                              "./shaders/flat.fs");

        shaders_[kLine] = Shader("./shaders/line.vs",
                              "./shaders/line.fs");

        shaders_[kCrossHair] = Shader("./shaders/crosshair.vs",
                                    "./shaders/line.fs");

        shaders_[kMultiply] = Shader("./shaders/quad.vs",
                                    "./shaders/premult.fs");
    }


    
    Render(const int width, const int height, const int num_cameras, GLContext * ctx)
    : current_framerate_(0.0f), max_framerate_(0.0f), avg_framerate_(0.0), all_rendered_frames_(0.0),
    free_camera_(), last_x_(0.0f), last_y_(0.0f), delta_time_(0.0f), first_mouse_(true), 
    last_frame_(), crosshair_vao_(0), crosshair_vbo_(0), cube_vao_(0), cube_vbo_(0),
    quad_vao_(0), quad_vbo_(0), shaders_(0), width_(width), height_(height), 
    background_color_(glm::vec3(0.5f, 0.5f, 0.5f)), ctx_(ctx), free_camera_framebuffer_(nullptr),
    camera_framebuffer_list_(num_cameras), multiplied_framebuffer_list_(num_cameras),
    reflection_map_(0), pixel_buffer_index_(0), pixel_buffer_next_index_(1), pixel_buffers_{0, 0},
    num_frames_(0), img_buffers_(0)
    {

        free_camera_framebuffer_ = new FBO(width, height, false, false);

        free_camera_ = Camera(glm::vec3(0.0f, 0.0f, 0.0f));
        free_camera_.Front = glm::vec3(1, 0, 0);
        free_camera_.Aspect = (float)width / (float)height;
        free_camera_.Near = 0.01f;
        free_camera_.Far = 100.0f;
        
        last_x_ = (float)width / 2.0f;
        last_y_ = (float)height / 2.0f;

        glGenBuffers(2, pixel_buffers_);
        glBindBuffer(GL_PIXEL_PACK_BUFFER, pixel_buffers_[0]);
        glBufferData(GL_PIXEL_PACK_BUFFER, width * height * 4 * sizeof(unsigned char), nullptr, GL_STREAM_READ);
        glBindBuffer(GL_PIXEL_PACK_BUFFER, pixel_buffers_[1]);
        glBufferData(GL_PIXEL_PACK_BUFFER, width * height * 4 * sizeof(unsigned char), nullptr, GL_STREAM_READ);
        glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

        vector<std::string> faces
        {
            "./cubemaps/right.tga",
            "./cubemaps/left.tga",
            "./cubemaps/top.tga",
            "./cubemaps/bottom.tga",
            "./cubemaps/front.tga",
            "./cubemaps/back.tga",
        };
        reflection_map_ = LoadCubemap(faces);
        
        InitShaders();
        InitFramebuffers(num_cameras);
    }

    ~Render()
    {
        glDeleteTextures(1, &reflection_map_);
        glDeleteVertexArrays(1, &crosshair_vao_);
        glDeleteVertexArrays(1, &cube_vao_);
        glDeleteVertexArrays(1, &quad_vao_);
        glDeleteBuffers(1, &crosshair_vbo_);
        glDeleteBuffers(1, &cube_vbo_);
        glDeleteBuffers(1, &quad_vbo_);
        glDeleteBuffers(2, pixel_buffers_);
        
        for(int i = 0; i < camera_framebuffer_list_.size(); ++i)
        {
            delete camera_framebuffer_list_[i];
            delete multiplied_framebuffer_list_[i];
        }

        delete free_camera_framebuffer_;
        img_buffers_.clear();
    }

};

}

#endif // RENDER_H_