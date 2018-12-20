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
#include "navigation_in_suncg.h"
#include "state_machine.h"

using namespace xrobot;

constexpr int w = 640;
constexpr int h = 480;

int main(int argc, char **argv)
{
    assert(argc < 3);

    render_engine::Profile profile = render_engine::kVeryLowQualityVisualize;
    if(argc == 2 && atoi(argv[1]) < 4) 
        profile = render_engine::profiles[atoi(argv[1])];

    profile.shadow = false;

    std::shared_ptr<MapSuncg> scene = std::make_shared<MapSuncg>();
    std::shared_ptr<render_engine::Render> renderer = 
            std::make_shared<render_engine::Render> (w, h, profile, false);
    
    TaskGroup group("TaskGroup_SUNCG");
    Task_SUNCG task_nav(renderer, scene);
    group.AddTask("SUNCG", task_nav);

    while(!renderer->GetContext()->GetWindowShouldClose())
        group.RunStage();

    return 0;
}