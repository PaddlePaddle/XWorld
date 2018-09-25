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
#include "task_example.h"
#include "task_example_testing.h"
#include "state_machine.h"
#include "render_engine/render2d.h"

using namespace xrobot;

static int w = 640;
static int h = 480;

int main(int argc, char **argv)
{
    assert(argc < 3);

    render_engine::RenderSettings render_profile(render_engine::kLowQuality);
    if(argc == 2) {
        render_profile = render_engine::RenderSettings(atoi(argv[1]));
    }

    render_engine::Render * renderer = new render_engine::Render(w, h, 1, render_profile, false);
    Map * scene = new Map();
    MapSuncg * scene_suncg = new MapSuncg();


    // Map Generation
    TaskGroup group0("TaskGroup_NavToObject");
    Task_NavToLargeCrate task0(renderer, scene);
    Task_NavToSmallCrate task1(renderer, scene);
    group0.AddTask("NavToLargeCrate", task0);
    group0.AddTask("NavToSmallCrate", task1);

    // Moving Target 
    TaskGroup group1("TaskGroup_NavToMovingRobot");
    Task_FollowRobot task2(renderer, scene);
    group1.AddTask("NavToMovingRobot", task2);

    // Loading SUNCG Scene
    TaskGroup group2("TaskGroup_NavToObjectInSUNCG");
    Task_NavToFruitBowl task3(renderer, scene_suncg);
    Task_TouchPan task4(renderer, scene_suncg);
    group2.AddTask("NavToFruitBowl", task3);
    group2.AddTask("TouchPan", task4);

    // // Loading Customized Scene (ex. obj with mtl)
    // TaskGroup group3("TaskGroup_Testing");
    // Task_LoadScene task5(renderer, scene);
    // group3.AddTask("LoadScene", task5);

    // // Get / Put
    // TaskGroup group4("TaskGroup_Testing");
    // Task_GetAndPut task6(renderer, scene_suncg);
    // group4.AddTask("GetAndPut", task6);

    // Loading Pica Pica Model...
    TaskGroup group5("TaskGroup_Testing2");
    Task_FollowRobot2 task7(renderer, scene);
    group5.AddTask("GetAndPut", task7);

    while(!renderer->ctx_->GetWindowShouldClose())
    {
        group5.RunStage();
    }

    delete renderer;
    delete scene;
    delete scene_suncg;

    return 0;
}