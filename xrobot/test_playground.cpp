#include "interface.h"

using namespace xrobot;
using namespace glm;

static int w = 640;
static int h = 480;

int main(int argc, char **argv)
{
	std::unique_ptr<Playground> scene(new Playground(w, h, 0));
	scene->EnableInventory(10);
	scene->Clear();
	scene->CreateAnTestScene();
	scene->SetLighting();

	Thing crate = scene->SpawnAnObject("./crate_1/crate.urdf",
					     vec3(0),
					     vec4(1,0,0,0),
					     1.0f,
					     "Crate",
					     false);

	scene->SpawnAnObject("./crate_0.3/crate.urdf",
					     vec3(5,0,0),
					     vec4(1,0,0,0),
					     1.0f,
					     "SmallCrate",
					     false);

	scene->SpawnAnObject("./crate_0.3/crate.urdf",
					     vec3(0,0,5),
					     vec4(1,0,0,0),
					     1.0f,
					     "SmallCrate",
					     false);

	Thing door = scene->SpawnAnObject("/home/ziyuli/model/door.json",
									   vec3(5,0,5),
									   vec4(-1,0,0,1.57f),
									   1.0f,
									   "Door");

	Thing robot_agent = scene->SpawnAnObject("husky/husky.urdf",
											 vec3(2,0,2),
											 vec4(-1,0,0,1.57f),
											 1.0f,
											 "Agent");
	scene->AttachCameraTo(robot_agent, vec3(0.3,1.3,0.0));
	scene->Initialize();

	while(1) {

		CTX * ctx = scene->GetContext();

		if(ctx->GetKeyPressUp())
	        scene->MoveForward(robot_agent);

	    if(ctx->GetKeyPressDown())
	        scene->MoveBackward(robot_agent);

	    if(ctx->GetKeyPressLeft())
	        scene->TurnLeft(robot_agent);

	    if(ctx->GetKeyPressRight())
	        scene->TurnRight(robot_agent);

	    if(ctx->GetKeyPressKP9())
            scene->LookUp();

        if(ctx->GetKeyPressKP6())
			scene->LookDown();

		if(ctx->GetKeyPress1())
			scene->Grasp();

		if(ctx->GetKeyPress2())
			scene->PutDown();

		if(ctx->GetKeyPress3())
			scene->Rotate(vec3(0.2f,0,0));

		if(ctx->GetKeyPress4())
			scene->TakeAction(0);


		if(ctx->GetKeyPressSpace()) {
			if(scene->QueryObjectWithLabelAtCameraCenter("Crate"))
				printf("Found!\n");
			else
				printf("Not Found!\n");
		}

		scene->Update();	
	}

	return 0;
}