#define USE_GLFW
#include "render_engine/gl_context.h"
#include "render_engine/render.h"
#include "utils.h"

#include <fstream>
#include <iostream>
#include <chrono>
#include <string>
#include <thread>
#include <unistd.h>
#include <assert.h>
#include <ios>
#include <iomanip>

#include "lidar.h"
#include "map.h"
#include "map_suncg.h"
#include "render_engine/render2d.h"

using namespace xrobot;

static int w = 640;
static int h = 480;
static std::string data_dir = "/home/ziyuli/Desktop/suncg";
static std::string metadata_models = "/home/ziyuli/Desktop/suncg/metadata/ModelCategoryMapping.csv";
static std::string house0 = "/home/ziyuli/Desktop/suncg/house/0a0b9b45a1db29832dd84e80c1347854/house.json";

//0a0b9b45a1db29832dd84e80c1347854

//7c16efebdfe46f3f14fa81abe500589c
// -8 6
// 0.3, 1, 1

int main(int argc, char **argv)
{
    assert(argc < 3);

    render_engine::RenderSettings render_profile(render_engine::kLowQuality);
    if(argc == 2) {
        render_profile = render_engine::RenderSettings(atoi(argv[1]));
    }

    render_engine::GLContext * ctx = render_engine::CreateContext(h, w * 2);
    render_engine::Render * renderer = new render_engine::Render(w, h, 1, render_profile, ctx);
    
    MapSuncg * scene = new MapSuncg();
    scene->LoadCategoryCSV(metadata_models.c_str());
    scene->SetRemoveAll( kRemoveStairs );
    scene->LoadJSON(house0.c_str(), data_dir.c_str(), true);
    scene->SetMapSize(-8, -8, 6, 6);

    Robot * husky;
    render_engine::Camera * c0;

    husky = scene->world_->LoadURDF(
        "husky/robot_kuka.urdf",
        btVector3(-6,0.21,-1),
        btQuaternion(btVector3(-1,0,0),1.57),
        0.6f
    );
    husky->DisableSleeping();

    c0 = scene->world_->add_camera(vec3(0,0,0), vec3(0.3,1.2,0.0), (float) w / h);
    scene->world_->attach_camera(c0, husky);
    renderer->Init(c0);

    scene->world_->BulletStep(); 

    static float pos_0 = 0.0f;
    static float pos_1 = 0.0f;
    static float pos_2 = 0.0f;
    static float pos_3 = 0.0f;
    static float pos_4 = 0.0f;
    static float pos_5 = 0.0f;
    static float pos_6 = 0.0f;
    static float pos_7 = 0.0f;
    static float cam_pitch = 0.0f;
    static float vel_front_left_wheel = 0.0f;
    static float vel_front_right_wheel = 0.0f;
    static float vel_rear_left_wheel = 0.0f;
    static float vel_rear_right_wheel = 0.0f;
    static float speed = 4.5f;

    // Lidar * lidar = new Lidar(scene->world_, 360, 10.0f);
    // renderer->InitDrawBatchRay(360);

    while(!ctx->GetWindowShouldClose())
    {

        if(ctx->GetKeyPressSpace()) {
            scene->world_->LoadURDF (
                "apple/apple.urdf",
                btVector3(-4,1,-1),
                btQuaternion(btVector3(-1,0,0),1.57),
                0.7f
            );
        }

        // std::vector<ObjectAttributes> temp;
        // scene_->world_->QueryObjectByLabel("pan", temp);

        // for(int i = 0; i < temp.size(); ++i)
        // {
        //     glm::vec3 aabb_min = temp[i].aabb_min - glm::vec3(0.2);
        //     glm::vec3 aabb_max = temp[i].aabb_max + glm::vec3(0.2);

        //     glm::vec3 gripper_aabb_min, gripper_aabb_max;
        //     husky->other_parts_[10]->GetAABB(gripper_aabb_min, gripper_aabb_max);

        //     if(aabb_min.x < c0_->Position.x &&
        //        aabb_max.x > c0_->Position.x && 
        //        aabb_min.z < c0_->Position.z &&
        //        aabb_max.z > c0_->Position.z) 
        //     {

        //     }
        // }
        // std::vector<RayTestInfo> batch_raycast_result;
        // lidar->Update(c0->Front, c0->Up, c0->Position - glm::vec3(0, 0.6f, 0));
        // batch_raycast_result = lidar->GetResult();

        // for (int i = 0; i < batch_raycast_result.size(); ++i)
        // {
        //     if(batch_raycast_result[i].bullet_id < 0)
        //     {
        //         renderer->UpdateRay(i, glm::vec3(0), glm::vec3(0));
        //     } else {
        //         renderer->UpdateRay(i, c0->Position - glm::vec3(0,0.5f,0),
        //                 batch_raycast_result[i].pos);
        //     }
        // }

        scene->world_->rotate_camera(c0, cam_pitch);
        Joint * j;


        husky->Freeze();
        if(ctx->GetKeyPressUp())
        {
            husky->MoveForward(0.01f);
        }

        if(ctx->GetKeyPressDown())
        {
            husky->MoveBackward(0.01f);
        }

        if(ctx->GetKeyPressLeft())
        {
            husky->TurnLeft(0.01f);
        }

        if(ctx->GetKeyPressRight())
        {
            husky->TurnRight(0.01f);
        }

        // vel_front_left_wheel = 0.0f;
        // vel_front_right_wheel = 0.0f;
        // vel_rear_left_wheel = 0.0f;
        // vel_rear_right_wheel = 0.0f;

        // if(ctx->GetKeyPressUp())
        // {
        //     vel_front_left_wheel  += speed;
        //     vel_front_right_wheel += speed;
        //     vel_rear_left_wheel   += speed;
        //     vel_rear_right_wheel  += speed;
        // }

        // if(ctx->GetKeyPressDown())
        // {
        //     vel_front_left_wheel  -= speed;
        //     vel_front_right_wheel -= speed;
        //     vel_rear_left_wheel   -= speed;
        //     vel_rear_right_wheel  -= speed;
        // }

        // if(ctx->GetKeyPressLeft())
        // {
        //     vel_front_left_wheel  -= speed;
        //     vel_front_right_wheel += speed;
        //     vel_rear_left_wheel   -= speed;
        //     vel_rear_right_wheel  += speed;
        // }

        // if(ctx->GetKeyPressRight())
        // {
        //     vel_front_left_wheel  += speed;
        //     vel_front_right_wheel -= speed;
        //     vel_rear_left_wheel   += speed;
        //     vel_rear_right_wheel  -= speed;
        // }

        if(ctx->GetKeyPress1())
        {
            pos_0 += 0.0025f;
        }
        if(ctx->GetKeyPress2())
        {
            pos_0 -= 0.0025f;
        }

        if(ctx->GetKeyPress3())
        {
            pos_1 += 0.0025f;
        }
        if(ctx->GetKeyPress4())
        {
            pos_1 -= 0.0025f;
        }

        if(ctx->GetKeyPress5())
        {
            pos_2 += 0.0025f;
        }
        if(ctx->GetKeyPress6())
        {
            pos_2 -= 0.0025f;
        }

        if(ctx->GetKeyPress7())
        {
            pos_3 += 0.0025f;
        }
        if(ctx->GetKeyPress8())
        {
            pos_3 -= 0.0025f;
        }

        if(ctx->GetKeyPress9())
        {
            pos_4 += 0.0025f;
        }
        if(ctx->GetKeyPress0())
        {
            pos_4 -= 0.0025f;
        }

        if(ctx->GetKeyPressKP1())
        {
            pos_5 += 0.0025f;
        }
        if(ctx->GetKeyPressKP2())
        {
            pos_5 -= 0.0025f;
        }

        if(ctx->GetKeyPressKP4())
        {
            pos_6 += 0.0025f;
        }
        if(ctx->GetKeyPressKP5())
        {
            pos_6 -= 0.0025f;
        }

        if(ctx->GetKeyPressKP7())
        {
            pos_7 = 0.05f;
        }
        if(ctx->GetKeyPressKP8())
        {
            pos_7 = 0.005f;
        }

        if(ctx->GetKeyPressKP9())
        {
            cam_pitch += 0.2f;
        }
        if(ctx->GetKeyPressKP6())
        {
            cam_pitch -= 0.2f;
        }

        cam_pitch = glm::clamp(cam_pitch, -45.0f, 45.0f);
        pos_0 = glm::clamp(pos_0, -1.0f, 1.0f);
        pos_1 = glm::clamp(pos_1, -1.0f, 1.0f);
        pos_2 = glm::clamp(pos_2, -1.0f, 1.0f);
        pos_3 = glm::clamp(pos_3, -1.0f, 1.0f);
        pos_4 = glm::clamp(pos_4, -1.0f, 1.0f);
        pos_5 = glm::clamp(pos_5, -1.0f, 1.0f);
        pos_6 = glm::clamp(pos_6, -1.0f, 1.0f);

        // j = husky->joints_list_[15];
        // j->SetJointMotorControlVelocity(vel_front_left_wheel, 1.0, 1000.0);
        // j = husky->joints_list_[16];
        // j->SetJointMotorControlVelocity(vel_front_right_wheel, 1.0, 1000.0);
        // j = husky->joints_list_[17];
        // j->SetJointMotorControlVelocity(vel_rear_left_wheel, 1.0, 1000.0);
        // j = husky->joints_list_[18];
        // j->SetJointMotorControlVelocity(vel_rear_right_wheel, 1.0, 1000.0);
        j = husky->joints_list_[2];
        j->SetJointMotorControlPosition(pos_0, 0.1f, 1.0f,100.0f);
        j = husky->joints_list_[3];
        j->SetJointMotorControlPosition(pos_1, 0.1f, 1.0f,100.0f);
        j = husky->joints_list_[4];
        j->SetJointMotorControlPosition(pos_2, 0.1f, 1.0f,100.0f);
        j = husky->joints_list_[5];
        j->SetJointMotorControlPosition(pos_3, 0.1f, 1.0f,100.0f);
        j = husky->joints_list_[6];
        j->SetJointMotorControlPosition(pos_4, 0.1f, 1.0f,100.0f);
        j = husky->joints_list_[7];
        j->SetJointMotorControlPosition(pos_5, 0.1f, 1.0f,100.0f);
        j = husky->joints_list_[8];
        j->SetJointMotorControlPosition(pos_6, 0.1f, 1.0f,100.0f);
        j = husky->joints_list_[10];
        j->SetJointMotorControlPosition(pos_7, 0.1f, 1.0f,10.0f);
        j = husky->joints_list_[12];
        j->SetJointMotorControlPosition(pos_7, 0.1f, 1.0f,10.0f);

        scene->world_->BulletStep();   
        renderer->StepRender(scene->world_);

        ctx->SwapBuffer();
        ctx->PollEvent();
    }

    // delete lidar;
    delete renderer;
    delete scene;

    CloseContext(ctx);
	
    return 0;
}
