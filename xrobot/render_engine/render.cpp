#include "glm/glm.hpp"

#include "render.h"
#include "render_utils.h"

namespace xrobot {
namespace render_engine {

Render::Render(const int width,
               const int height,
               const int num_cameras,
               GLContext * ctx) : current_framerate_(0.0f),
                                  max_framerate_(0.0f),
                                  avg_framerate_(0.0),
                                  all_rendered_frames_(0.0),
                                  free_camera_(),
                                  last_x_(0.0f),
                                  last_y_(0.0f),
                                  delta_time_(0.0f),
                                  first_mouse_(true), 
                                  last_frame_(),
                                  crosshair_vao_(0),
                                  crosshair_vbo_(0),
                                  cube_vao_(0),
                                  cube_vbo_(0),
                                  quad_vao_(0),
                                  quad_vbo_(0),
                                  shaders_(0),
                                  width_(width),
                                  height_(height), 
                                  background_color_(glm::vec3(0.5f, 0.5f, 0.5f)),
                                  ctx_(ctx),
                                  free_camera_framebuffer_(nullptr),
                                  camera_framebuffer_list_(num_cameras),
                                  multiplied_framebuffer_list_(num_cameras),
                                  reflection_map_(0),
                                  pixel_buffer_index_(0),
                                  pixel_buffer_next_index_(1),
                                  pixel_buffers_{0, 0},
                                  num_frames_(0),
                                  img_buffers_(0) {

    free_camera_framebuffer_ = new FBO(width, height, false, false);

    free_camera_ = Camera(glm::vec3(0.0f, 0.0f, 0.0f));
    free_camera_.Front = glm::vec3(1, 0, 0);
    free_camera_.Aspect = (float)width / height;
    free_camera_.Near = 0.01f;
    free_camera_.Far = 100.0f;
    
    last_x_ = width / 2.0f;
    last_y_ = height / 2.0f;

    glGenBuffers(2, pixel_buffers_);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, pixel_buffers_[0]);
    glBufferData(GL_PIXEL_PACK_BUFFER,
                 width * height * 4 * sizeof(unsigned char),
                 nullptr,
                 GL_STREAM_READ);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, pixel_buffers_[1]);
    glBufferData(GL_PIXEL_PACK_BUFFER,
                 width * height * 4 * sizeof(unsigned char),
                 nullptr,
                 GL_STREAM_READ);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

    std::string pwd = get_pwd(__FILE__);
    std::vector<std::string> faces {
        pwd + "/../data/cubemaps/right.tga",
        pwd + "/../data/cubemaps/left.tga",
        pwd + "/../data/cubemaps/top.tga",
        pwd + "/../data/cubemaps/bottom.tga",
        pwd + "/../data/cubemaps/front.tga",
        pwd + "/../data/cubemaps/back.tga",
    };
    reflection_map_ = LoadCubemap(faces);
    
    InitShaders();
    InitFramebuffers(num_cameras);
}

Render::~Render() {
    glDeleteTextures(1, &reflection_map_);
    glDeleteVertexArrays(1, &crosshair_vao_);
    glDeleteVertexArrays(1, &cube_vao_);
    glDeleteVertexArrays(1, &quad_vao_);
    glDeleteBuffers(1, &crosshair_vbo_);
    glDeleteBuffers(1, &cube_vbo_);
    glDeleteBuffers(1, &quad_vbo_);
    glDeleteBuffers(2, pixel_buffers_);
    
    for(int i = 0; i < camera_framebuffer_list_.size(); ++i) {
        delete camera_framebuffer_list_[i];
        delete multiplied_framebuffer_list_[i];
    }

    delete free_camera_framebuffer_;
    img_buffers_.clear();
}

void Render::ProcessInput() {
    if(ctx_->GetKeyPressESC())
        ctx_->SetWindowShouldClose();
    if(ctx_->GetKeyPressW()) 
        free_camera_.ProcessKeyboard(Camera::FORWARD, delta_time_);
    if(ctx_->GetKeyPressS())
        free_camera_.ProcessKeyboard(Camera::BACKWARD, delta_time_);
    if(ctx_->GetKeyPressA())
        free_camera_.ProcessKeyboard(Camera::LEFT, delta_time_);
    if(ctx_->GetKeyPressD())
        free_camera_.ProcessKeyboard(Camera::RIGHT, delta_time_);
    if(ctx_->GetKeyPressQ())
        free_camera_.ProcessKeyboard(Camera::UP, delta_time_);
    if(ctx_->GetKeyPressE())
        free_camera_.ProcessKeyboard(Camera::DOWN, delta_time_);
}

void Render::ProcessMouse() {
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

void Render::RenderMarker() {
    if(crosshair_vao_ == 0) {
        constexpr float vertices[] = {
            -0.075, 0, 0,
            0.075, 0, 0,
            0, -0.1, 0,
            0, 0.1, 0
        };

        glGenVertexArrays(1, &crosshair_vao_);
        glGenBuffers(1, &crosshair_vbo_);
        glBindBuffer(GL_ARRAY_BUFFER, crosshair_vbo_);
        glBufferData(
                GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
        glBindVertexArray(crosshair_vao_);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(
                0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

    glBindVertexArray(crosshair_vao_);
    glDrawArrays(GL_LINES, 0, 4);
    glBindVertexArray(0);
}

void Render::RenderCube() {
    if (cube_vao_ == 0) {
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
    // render Cube
    glBindVertexArray(cube_vao_);
    glDrawArrays(GL_LINES, 0, 48);
    glBindVertexArray(0);
}

void Render::RenderQuad() {
    if (quad_vao_ == 0) {
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

void Render::InitFramebuffers(const unsigned int num_cameras) {
    camera_framebuffer_list_.resize(num_cameras);
    multiplied_framebuffer_list_.resize(num_cameras);
    for(unsigned int i = 0; i < num_cameras; ++i) {
        camera_framebuffer_list_[i] = new FBO(width_, height_, true, true);
        multiplied_framebuffer_list_[i] = new FBO(width_, height_, false, false);
    }
}

void Render::DrawRootAABB(RenderWorld* world, const Shader& shader) {
    glDisable( GL_POLYGON_OFFSET_FILL );

    for (size_t i = 0; i < world->size(); i++)
    {
        RenderBody* body = world->render_body_ptr(i);
        RenderPart * part = body->render_root_ptr();
        if (part && !body->recycle())
        {
            glm::vec3 aabb_min, aabb_max;
            part->GetAABB(aabb_min, aabb_max);
            shader.setVec3("aabbMin", aabb_min);
            shader.setVec3("aabbMax", aabb_max);
            RenderCube();
        }
    }

    glPolygonOffset(1.0f, 1.0f);
    glEnable( GL_POLYGON_OFFSET_FILL );
}

//void Render::DrawEmptyAABB(Map* map, const Shader& shader) {
//    glDisable( GL_POLYGON_OFFSET_FILL );
//
//    for (int i = 0; i < map->empty_map_.size(); ++i)
//    {
//        vec3 aabb_min = map->empty_map_[i].first;
//        vec3 aabb_max = map->empty_map_[i].second;
//
//        shader.setVec3("aabbMin", aabb_min);
//        shader.setVec3("aabbMax", aabb_max);
//        RenderCube();
//    }
//
//    glPolygonOffset(1.0f, 1.0f);
//    glEnable( GL_POLYGON_OFFSET_FILL );
//}

//void Render::DrawRoomAABB(Map* map, const Shader& shader) {
//    glDisable( GL_POLYGON_OFFSET_FILL );
//
//    for (int i = 0; i < map->sections_map_.size(); ++i)
//    {
//        vec3 aabb_min = map->sections_map_[i].first;
//        vec3 aabb_max = map->sections_map_[i].second;
//
//        shader.setVec3("aabbMin", aabb_min);
//        shader.setVec3("aabbMax", aabb_max);
//        RenderCube();
//    }
//
//    glPolygonOffset(1.0f, 1.0f);
//    glEnable( GL_POLYGON_OFFSET_FILL );
//}

void Render::Draw(RenderWorld* world, const Shader& shader) {
    auto do_drawing = [&](const RenderPart* c, bool is_root) {
        int id_r, id_g, id_b;
        IdToColor(c->id(), id_r, id_g, id_b);
        glm::vec3 id_color(id_r/255.0f, id_g/255.0f, id_b/255.0f);

        for (size_t i = 0; i < c->size(); ++i) {
            ModelData* model = c->model_data(i);
            OriginTransformation* transform = c->transform(i);

            glm::mat4 translate = c->position();
            glm::mat4 local_frame = 
                    glm::inverse(c->local_inertial_frame());
            glm::mat4 scale =  glm::scale(
                    glm::mat4(1), transform->local_scale);

            shader.setMat4("state", translate * local_frame);
            shader.setMat4("scale", scale);
            if (is_root) {
                shader.setMat4("model", transform->origin);	
            }
            shader.setFloat("flip", transform->flip);
            shader.setVec3("id_color", id_color);
            model->Draw(shader);
        }
    };

    for (size_t i = 0; i < world->size(); ++i) {
        RenderBody* body = world->render_body_ptr(i);
        // Root
        RenderPart* root = body->render_root_ptr();
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
}

void Render::StepRenderFreeCamera(RenderWorld *world) {
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
    glUniform1i(glGetUniformLocation(lambert_shader.id(), "cmap"), 5);

    Draw(world, lambert_shader);

    line_shader.use();
    line_shader.setMat4("projection", projection);
    line_shader.setMat4("view", view);

    line_shader.setVec3("color", glm::vec3(1,0,0));
    DrawRootAABB(world, line_shader);

    line_shader.setVec3("color", glm::vec3(0,1,0));
    //DrawEmptyAABB(map, line_shader);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Render::StepRenderAllCameras(RenderWorld* world,
                                  std::vector<int>& picks,
                                  const bool color_pick) {
    for (int i = 0; i < camera_framebuffer_list_.size(); ++i) {
        Shader lambert_shader = shaders_[kLambert];

        glBindFramebuffer(
                GL_FRAMEBUFFER, camera_framebuffer_list_[i]->frameBuffer);
        glClearColor(1,1,1,0);      
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        Camera* camera = world->camera_list_[i];
        glm::mat4 projection = camera->GetProjectionMatrix();
        glm::mat4 view = camera->GetViewMatrix();

        lambert_shader.use();
        lambert_shader.setMat4("projection", projection);
        lambert_shader.setMat4("view", view);
        lambert_shader.setVec3("camPos", camera->Position);
        Draw(world, lambert_shader);

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
            glReadPixels(
                    width_/2, height_/2, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, data); 
            picks[i] = ColorToId(data[0], data[1], data[2]);
        }

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
}

void Render::StepRenderGetDepthAllCameras() {
    glDisable(GL_DEPTH_TEST);
    img_buffers_.clear();

    for (int i = 0; i < camera_framebuffer_list_.size(); ++i) {     
        glBindFramebuffer(
                GL_FRAMEBUFFER, multiplied_framebuffer_list_[i]->frameBuffer);
        glViewport(0, 0, width_, height_);
        glClearColor(background_color_.x,
                     background_color_.y,
                     background_color_.z,
                     1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        Shader multiply_shader = shaders_[kMultiply];

        multiply_shader.use();
        camera_framebuffer_list_[i]->ActivateAsTexture(
                multiply_shader.id(), "tex", 0);
        camera_framebuffer_list_[i]->ActivateDepthAsTexture(
                multiply_shader.id(), "dep", 1);
        RenderQuad();

        pixel_buffer_index_ = (pixel_buffer_index_ + 1) % 2;
        pixel_buffer_next_index_ = (pixel_buffer_index_ + 1) % 2;
        
        glReadBuffer(GL_COLOR_ATTACHMENT0);            
        if(num_frames_ * (int)multiplied_framebuffer_list_.size() < 2) {
            glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB,
                            pixel_buffers_[pixel_buffer_index_]);
            glReadPixels(0, 0, width_, height_, GL_RGBA, GL_UNSIGNED_BYTE, 0);
            
            GLubyte* ptr = (GLubyte*)glMapBufferARB(
                    GL_PIXEL_PACK_BUFFER_ARB,GL_READ_ONLY_ARB);
            if (ptr) {
                Image image_temp;
                std::vector<unsigned char> temp(
                        ptr, ptr + (int)(width_*height_*4));
                image_temp.data = temp;
                image_temp.camera_id = i;
                img_buffers_.push_back(image_temp);
                
                glUnmapBufferARB(GL_PIXEL_PACK_BUFFER_ARB);
            }
            
        } else {
            glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB,
                            pixel_buffers_[pixel_buffer_next_index_]);
            GLubyte* ptr = (GLubyte*)glMapBufferARB(
                    GL_PIXEL_PACK_BUFFER_ARB,GL_READ_ONLY_ARB);
            if (ptr) {
                Image image_temp;
                std::vector<unsigned char> temp(
                        ptr, ptr + (int)(width_*height_*4));
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

void Render::Visualization() {
    glDisable(GL_BLEND); 
    glViewport(0, 0, width_, height_);
    glClearColor(
            background_color_.x, background_color_.y, background_color_.z, 1.0f); 
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    shaders_[kColor].use();
    free_camera_framebuffer_->ActivateAsTexture(shaders_[kColor].id(), "tex", 0);
    RenderQuad();     

    for (int i = 0; i < multiplied_framebuffer_list_.size(); ++i) {
        glViewport(width_, i*height_, width_, height_);
        shaders_[kColor].use();
        multiplied_framebuffer_list_[i]->ActivateAsTexture(
                shaders_[kColor].id(), "tex", 0);
        RenderQuad();
    }

    for (int i = 0; i < multiplied_framebuffer_list_.size(); ++i) {
        glViewport(width_*2, i*height_, width_, height_);
        shaders_[kDepth].use();
        camera_framebuffer_list_[i]->ActivateDepthAsTexture(
                shaders_[kDepth].id(), "tex", 0);
        RenderQuad();
    }
}

int Render::StepRender(RenderWorld* world, int pick_camera_id) {
    GetDeltaTime();
    GetFrameRate();

    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glViewport(0, 0, width_, height_);
    glClearColor(
            background_color_.x, background_color_.y, background_color_.z, 1.0f);        

    StepRenderFreeCamera(world);

    int pick_result = -1;
    std::vector<int> picks(camera_framebuffer_list_.size(), -1);
    if(pick_camera_id >= 0) {
        StepRenderAllCameras(world, picks, true);
        pick_result = picks[pick_camera_id];
    } else {
        StepRenderAllCameras(world, picks, false);
    }

    StepRenderGetDepthAllCameras();

    Visualization();
    
    num_frames_++;
    return pick_result;
}

void Render::InitShaders() {
    std::string pwd = get_pwd(__FILE__);
    shaders_.resize(6);
    shaders_[kLambert] = Shader(pwd+"/shaders/lambert.vs",
                                pwd+"/shaders/lambert.fs");
    
    shaders_[kDepth] = Shader(pwd+"/shaders/quad.vs",
                              pwd+"/shaders/depth.fs");
    
    shaders_[kColor] = Shader(pwd+"/shaders/quad.vs",
                              pwd+"/shaders/flat.fs");

    shaders_[kLine] = Shader(pwd+"/shaders/line.vs",
                             pwd+"/shaders/line.fs");

    shaders_[kCrossHair] = Shader(pwd+"/shaders/crosshair.vs",
                                  pwd+"/shaders/line.fs");

    shaders_[kMultiply] = Shader(pwd+"/shaders/quad.vs",
                                 pwd+"/shaders/premult.fs");
}

void Render::GetDeltaTime() {
    TimeFrame current_frame = std::chrono::high_resolution_clock::now();
    delta_time_ = std::chrono::duration_cast<std::chrono::duration<double>>(
            current_frame - last_frame_).count();
    last_frame_ = current_frame;
}

void Render::GetFrameRate() {
    current_framerate_ = 1.0f / delta_time_;
    avg_framerate_ += current_framerate_;
    all_rendered_frames_ += 1;
    max_framerate_ = glm::max(max_framerate_, current_framerate_);

    std::string framerate_str = "Cur: " 
        + std::to_string((int)current_framerate_)
        + " Avg: " + std::to_string((int)(avg_framerate_/all_rendered_frames_))
        + " Max: " + std::to_string((int)max_framerate_);
    ctx_->SetTitle(framerate_str.c_str());
}

} } // xrobot::render_engine
