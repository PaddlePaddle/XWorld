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

static int w = 800;
static int h = 450;
static int frame = 0;

static std::string door0 = "./door0/door.urdf";
static std::string wall = "./wall/floor.urdf";
static std::string floor0 = "./floor0/floor.urdf";
static std::string floor1 = "./floor1/floor.urdf";
static std::string floor2 = "./floor2/floor.urdf";
static std::string crate1 = "./crate_1/crate.urdf";
static std::string crate03 = "./crate_0.3/crate.urdf";
static std::string apple = "./apple/apple.urdf";
static float cam_pitch = 0.0f;

int main(int argc, char **argv)
{
    assert(argc < 3);

    render_engine::RenderSettings render_profile(render_engine::kLowQuality);
    if(argc == 2) {
        render_profile = render_engine::RenderSettings(atoi(argv[1]));
    }

    render_engine::GLContext * ctx = render_engine::CreateContext(h, w * 2);
    render_engine::Render * renderer = new render_engine::Render(w, h, 1, render_profile, ctx);
    Map * scene = new Map();

    scene->CreateLabel(crate1, "crate");
    scene->CreateLabel(crate03, "crate");
    scene->CreateLabel(apple, "apple");
    scene->CreateSectionType(floor0, wall, door0);
    scene->CreateSectionType(floor1, wall, door0);
    scene->CreateSectionType(floor2, wall, door0);
    scene->CreateSpawnOnFloor(crate1);
    scene->CreateSpawnOnObject(apple);
    scene->CreateSpawnEither(crate03);
    scene->CreateSpawnConstraint(apple);


    startBenchmark();
    glm::vec3 startPosition = scene->GenerateFloorPlan(10, 10);
    scene->Spawn(5, 5, 5);
    endBenchmark();

    Robot * husky;
    render_engine::Camera * c0;

    husky = scene->world_->LoadURDF(
        "husky/husky.urdf",
        btVector3(startPosition.x,0.21,startPosition.z),
        btQuaternion(btVector3(-1,0,0),1.57)
    );
    husky->DisableSleeping();

    c0 = scene->world_->AddCamera(vec3(0,0,0), vec3(0.3,1.3,0.0), (float) w / h);
    scene->world_->AttachCamera(c0, husky);
    renderer->Init(c0);
    scene->world_->BulletStep(); 

    int count = 0;
    bool flag = true; 

    while(!ctx->GetWindowShouldClose())
    {
        if(ctx->GetKeyPress0())
        {
            std::vector<ObjectAttributes> temp;
            scene->world_->QueryObjectByLabel("apple", temp);

            printf("apple: %d\n", temp.size());
            for (int i = 0; i < temp.size(); ++i)
            {
                printf("    id: %d\n", temp[i].bullet_id);
            }
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

            husky = scene->world_->LoadURDF(
                "./husky/husky.urdf",
                btVector3(startPosition.x,0.21,startPosition.z),
                btQuaternion(btVector3(-1,0,0),1.57)
            );
            husky->DisableSleeping();

            c0 = scene->world_->AddCamera(vec3(0,0,0), vec3(0.3,1.3,0.0), (float) w / h);
            scene->world_->AttachCamera(c0, husky);
            scene->world_->BulletStep(); 
        }


        scene->world_->RotateCamera(c0, cam_pitch); 

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

        if(ctx->GetKeyPressKP9())
        {
            cam_pitch += 0.1f;
        }
        if(ctx->GetKeyPressKP6())
        {
            cam_pitch -= 0.1f;
        }

        cam_pitch = glm::clamp(cam_pitch, -45.0f, 45.0f);


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
