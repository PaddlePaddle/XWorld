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
#include <ios>
#include <iomanip>

#include "map.h"
#include "render_engine/render2d.h"

using namespace xrobot;

static int w = 640;
static int h = 480;
static int frame = 0;

static std::string door0 = "./door0/door.urdf";
static std::string wall = "./wall/floor.urdf";
static std::string floor0 = "./floor0/floor.urdf";
static std::string floor1 = "./floor1/floor.urdf";
static std::string floor2 = "./floor2/floor.urdf";
static std::string crate1 = "./crate_1/crate.urdf";
static std::string crate03 = "./crate_0.3/crate.urdf";
static std::string apple = "./apple/apple.urdf";


int main(int argc, char **argv)
{
    assert(argc < 3);

    render_engine::RenderSettings render_profile(render_engine::kLowQuality);
    if(argc == 2) {
        render_profile = render_engine::RenderSettings(atoi(argv[1]));
    }

    render_engine::GLContext * ctx = render_engine::CreateContext(h, w);
    render_engine::Render * renderer = new render_engine::Render(w, h, 1, render_profile, ctx);
    Map * scene = new Map();

    scene->CreateLabel(crate1, "crate");
    scene->CreateLabel(crate03, "crate");
    scene->CreateLabel(apple, "apple");
    scene->CreateSectionType(floor0, wall, door0);
    scene->CreateSectionType(floor1, wall, door0);
    scene->CreateSectionType(floor2, wall, door0);
    scene->CreateSpawnOnFloor(crate1);
    // scene->CreateSpawnOnFloor(bed1);
    // scene->CreateSpawnOnFloor(chair1);

    // scene->CreateSpawnOnObject(apple);

    // scene->CreateSpawnEither(crate03);
    // scene->CreateSpawnEither(cat);

    // scene->CreateSpawnConstraint(apple);
    // scene->CreateSpawnConstraint(bed1);
    // scene->CreateSpawnConstraint(chair1);

    startBenchmark();
    glm::vec3 startPosition = scene->GenerateFloorPlan(10, 10);
    scene->Spawn(5, 0, 0);
    endBenchmark();

    Robot * husky;
    render_engine::Camera * c0;


    husky = scene->world_->LoadURDF(
        "husky/robot_kuka.urdf",
        btVector3(startPosition.x,0.21,startPosition.z),
        btQuaternion(btVector3(-1,0,0),1.57)
    );
    husky->DisableSleeping();

    c0 = scene->world_->add_camera(vec3(0,0,0), vec3(0.3,1.0,0.0), (float) w / h);
    scene->world_->attach_camera(c0, husky);

    renderer->Init(c0);

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
    static float speed = 8.5f;

    int count = 0;
    bool flag = true; 

    while(!ctx->GetWindowShouldClose())
    {
        if(ctx->GetKeyPressKP0())
        {
            scene->world_->PrintCacheInfo();
        }

        if(ctx->GetKeyPressSpace())
        {
            count++;
            printf("count: %d\n", count);

            if(count % 1000 == 0)
                scene->ResetMap();
            else
                scene->ClearMap();

            startPosition = scene->GenerateFloorPlan(8, 8);
            scene->Spawn(5, 0, 0);

            husky = scene->world_->load_urdf(
                "./husky/robot_kuka.urdf",
                btVector3(startPosition.x,0.21,startPosition.z),
                btQuaternion(btVector3(-1,0,0),1.57)
            );
            husky->DisableSleeping();

            pos_0 = 0.0f;
            pos_1 = 0.0f;
            pos_2 = 0.0f;
            pos_3 = 0.0f;
            pos_4 = 0.0f;
            pos_5 = 0.0f;
            pos_6 = 0.0f;
            pos_7 = 0.005f;

            c0 = scene->world_->add_camera(vec3(0,0,0), vec3(0.3,1.0,0.0), (float) w / h);
            scene->world_->attach_camera(c0, husky);
        }


        // Ray Test
        // glm::vec3 fromPosition = c0->Position;
        // glm::vec3 toPosition = c0->Front * 20.0f + fromPosition;
        // int res = scene->world->rayTest(fromPosition, toPosition);

        scene->world_->rotate_camera(c0, cam_pitch); 

        Joint * j;

        vel_front_left_wheel = 0.0f;
        vel_front_right_wheel = 0.0f;
        vel_rear_left_wheel = 0.0f;
        vel_rear_right_wheel = 0.0f;

        if(ctx->GetKeyPressUp())
        {
            vel_front_left_wheel  += speed;
            vel_front_right_wheel += speed;
            vel_rear_left_wheel   += speed;
            vel_rear_right_wheel  += speed;
        }

        if(ctx->GetKeyPressDown())
        {
            vel_front_left_wheel  -= speed;
            vel_front_right_wheel -= speed;
            vel_rear_left_wheel   -= speed;
            vel_rear_right_wheel  -= speed;
        }

        if(ctx->GetKeyPressLeft())
        {
            vel_front_left_wheel  -= speed;
            vel_front_right_wheel += speed;
            vel_rear_left_wheel   -= speed;
            vel_rear_right_wheel  += speed;
        }

        if(ctx->GetKeyPressRight())
        {
            vel_front_left_wheel  += speed;
            vel_front_right_wheel -= speed;
            vel_rear_left_wheel   += speed;
            vel_rear_right_wheel  -= speed;
        }

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
            cam_pitch += 0.1f;
        }
        if(ctx->GetKeyPressKP6())
        {
            cam_pitch -= 0.1f;
        }

        cam_pitch = glm::clamp(cam_pitch, -45.0f, 45.0f);
        pos_0 = glm::clamp(pos_0, -1.0f, 1.0f);
        pos_1 = glm::clamp(pos_1, -1.0f, 1.0f);
        pos_2 = glm::clamp(pos_2, -1.0f, 1.0f);
        pos_3 = glm::clamp(pos_3, -1.0f, 1.0f);
        pos_4 = glm::clamp(pos_4, -1.0f, 1.0f);
        pos_5 = glm::clamp(pos_5, -1.0f, 1.0f);
        pos_6 = glm::clamp(pos_6, -1.0f, 1.0f);

        j = husky->joints_list_[15];
        j->SetJointMotorControlVelocity(vel_front_left_wheel, 1.0, 1000.0);
        j = husky->joints_list_[16];
        j->SetJointMotorControlVelocity(vel_front_right_wheel, 1.0, 1000.0);
        j = husky->joints_list_[17];
        j->SetJointMotorControlVelocity(vel_rear_left_wheel, 1.0, 1000.0);
        j = husky->joints_list_[18];
        j->SetJointMotorControlVelocity(vel_rear_right_wheel, 1.0, 1000.0);
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


    delete renderer;
    delete scene;

    CloseContext(ctx);
	
    return 0;
}
