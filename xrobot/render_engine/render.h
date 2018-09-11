#ifndef RENDER_ENGINE_RENDER_H_
#define RENDER_ENGINE_RENDER_H_

#include <chrono>
#include <ctime>
#include <ratio>
#include <stdio.h>
#include <string>
#include <vector>

#include "fbo.h"
#include "gl_context.h"
#include "shader.h"
#include "render_world.h"

namespace xrobot {
namespace render_engine {

typedef std::chrono::high_resolution_clock::time_point TimeFrame;

struct Image {
    Image() : data(0), camera_id(-1) {}

    std::vector<unsigned char> data;
    int camera_id;
};

class Render {
    enum ShaderTypes {
        kLambert = 0,
        kDepth = 1,
        kColor = 2,
        kLine = 3,
        kCrossHair = 4,
        kMultiply = 5
    };

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
    glm::vec3 background_color_;
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

public: 
    Render(const int width,
           const int height,
           const int num_cameras,
           GLContext * ctx);

    ~Render();

    void ProcessInput();

    void ProcessMouse();

    void RenderMarker();

    void RenderCube();
    
    void RenderQuad();
    
    void InitFramebuffers(const unsigned int num_cameras);
    
    void Draw(RenderWorld* world, const Shader& shader);

    void DrawRootAABB(RenderWorld* world, const Shader& shader);
    //
    //void DrawEmptyAABB(Map* map, const Shader& shader);
    
    //void DrawRoomAABB(Map* map, const Shader& shader); 

    void StepRenderFreeCamera(RenderWorld* world);

    void StepRenderAllCameras(
            RenderWorld* world, std::vector<int>& picks, const bool color_pick);

    void StepRenderGetDepthAllCameras();

    void Visualization();

    int StepRender(RenderWorld* map, int pick_camera_id = -1);
    
    // This Function Has To Be Called After StepRender(...)!
    std::vector<Image> GetRenderedImages() {
        return img_buffers_;
    }
    
    void InitShaders();
 
private:
    void IdToColor(const int id, int& r, int& g, int& b) {
        r = (id & 0x000000FF) >> 0;
        g = (id & 0x0000FF00) >> 8;
        b = (id & 0x00FF0000) >> 16;
    }

    int ColorToId(const int r, const int g, const int b) {
        return r + g * 256 + b * 256 * 256;
    }

    void GetDeltaTime();

    void GetFrameRate();

    std::string get_pwd(const std::string& file) {
        size_t p = file.find_last_of("/");
        return file.substr(0, p);
    }
};

} } // xrobot::render_engine

#endif // RENDER_ENGINE_RENDER_H_
