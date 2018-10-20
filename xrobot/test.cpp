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
#include <memory>

#include "task.h"
#include "task_example.h"
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

    std::shared_ptr<MapGrid> scene(new MapGrid());
    std::shared_ptr<MapSuncg> scene_suncg(new MapSuncg());
    std::shared_ptr<render_engine::Render> renderer(
        new render_engine::Render(w, h, 1, render_profile, false)
    );
    
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
    //Task_TouchPan task4(renderer, scene_suncg);
    group2.AddTask("NavToFruitBowl", task3);
    //group2.AddTask("TouchPan", task4);

    // Inventory, Pickup and Putdown, Animation and Convertion
    TaskGroup group3("TaskGroup_New");
    Task_NewFeatures task5(renderer, scene);
    group3.AddTask("GetAndPut", task5);
    
    // Navigation
    TaskGroup group4("TaskGroup_Crowd");
    Task_Crowd task6(renderer, scene);
    group4.AddTask("Crowd", task6);

    while(!renderer->ctx_->GetWindowShouldClose())
    {
        group2.RunStage();
    }

    return 0;
}