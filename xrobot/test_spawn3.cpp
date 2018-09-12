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

#include "task.h"
#include "state_machine.h"
#include "render_engine/render2d.h"

using namespace xrobot;

static int w = 640;
static int h = 480;
static int frame = 0;

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
    MapSuncg * scene_suncg = new MapSuncg();

    NavTask0 nav_task0(ctx, renderer, scene);
    NavTask1 nav_task1(ctx, renderer, scene);
    NavTask2 nav_task2(ctx, renderer, scene);
    NavTask3 nav_task3(ctx, renderer, scene);
    NavTask4 nav_task4(ctx, renderer, scene_suncg);
    NavTask5 nav_task5(ctx, renderer, scene_suncg);

    TaskGroup task_group0("TaskGroup_NavToObject");
    TaskGroup task_group1("TaskGroup_NavToRobot");
    TaskGroup task_group3("TaskGroup_NavToObject_SUNCG");
    TaskGroup task_group5("TaskGroup_TestingGroup");

    task_group0.AddTask("Task_NavToLargeCrate", nav_task1, 0);
    //task_group0.AddTask("Task_NavToSmallCrate", nav_task2, 0);
    task_group1.AddTask("Task_NavToMovingRobot", nav_task0, 0);
    task_group3.AddTask("TaskGroup_NavToObject_SUNCG", nav_task4, 0);
    //task_group3.AddTask("TaskGroup_GrabObject_SUNCG", nav_task5, 0);

    renderer->InitDrawBatchRay(180);

    while(!ctx->GetWindowShouldClose())
    {
        task_group0.RunStage();
    }

    CloseContext(ctx);
    return 0;
}