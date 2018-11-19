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
#include "task_example_crowd.h"
#include "state_machine.h"
#include "render_engine/render2d.h"

using namespace xrobot;

static int w = 640;
static int h = 480;

int main(int argc, char **argv)
{
    assert(argc < 3);

    render_engine::RenderSettings render_profile(render_engine::kLowQuality);
    if(argc == 2) 
        render_profile = render_engine::RenderSettings(atoi(argv[1]));


    std::shared_ptr<MapGrid> scene(new MapGrid());
    std::shared_ptr<render_engine::Render> renderer(
        new render_engine::Render(w, h, 1, render_profile, false)
    );

    TaskGroup group("TaskGroup_Crowd");
    Task_Crowd task_nav(renderer, scene);
    group.AddTask("Crowd", task_nav);

    while(!renderer->GetContext()->GetWindowShouldClose())
        group.RunStage();

    return 0;
}