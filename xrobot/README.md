XRobot
============

# Introduction

# Prerequisites

XRobot build-depends on GLX, GLFW, EGL, and Boost. In addition, XRobot requires a graphics card with at least OpenGL 3.3 compatible graphics driver.

On Ubuntu and derivatives, run:

    sudo apt-get install libglfw3-dev libboost-all-dev
    
# Build

Build depends cmake >= 3.0. To build the module:

    mkdir build
    cmake ..
    make -j

# Usage

There are a few good starting points for familiarizing oneself with the code:

For Python users:
- Look at the `python_binding.h` for how to construct the scene
- Look at `python/py_nav_task.py` for how to define the task

For C++ users:
- Still working on it, But...
- Look at the `task_example.h` and `task_example.cpp`
- Follow Task_FollowRobot (task0) to learn how to define the task
- Follow `world.h` for details on APIs

# Code Overview

The code in the `xrobot/directory` is organized as follows:

- data/ contains all necessary models and textures for running basic tests and tutorials
- vendor/ contains code for loading json and images
- python/ contains code for different navigation tasks in python
- render_engine/ contains code for rendering the scene
    - render_engine/EGL/ contains EGL extensions code
    - render_engine/shaders/ contains various types of shaders which are necessary for render engine

# Performance

On Ubuntu and derivatives, use following script to benchmark:

    sh run_benchmark.sh --num-proc 3 --num-gpu 1

The results are approx. framerates in each process (include rendering and simulation). The total framerate should reach approx. 500 - 1500 fps on decent Nvidia GPU with EGL backend.

# Python Tutorial

## Write a Navigation Task

Some examples can be found in `xrobot/python`. Below we show the step-by-step code to create a navigation task. Check out `API.md` for more details.


a) Import different Python modules which all together constitute the overall simulation environment

```python

from teaching_task import *
from libxrobot import *

```

b) Create a task by implementing a base class `TaskInterface` and define the member function for each stage

```python

"""
this tutorial is adapted from the task file below:
xrobot/python/py_nav_task.py
"""

class XRobot3DNavTarget(TaskInterface):

    def __init__(self):

    ## Return all the stages as a python dictionary which constitue the tasks
    def get_stages(self):
        stages = {
            ## 'idle' stage is mandatory, the rest should follow the rule below
            ## stage_name : corresponding member function

            "idle" : self.start, 
            "navigation" : self.navigation
        }
        return stages

    def start(self):

    def navigation(self):

    # def other_stage(self):
    # ...

```


c) Create a simulation environment

```python

def __init__(self):

    self.agent = None

    ## YOU SHOULD CREATE THIS PLAYGROUND OUTSIDE OF TASK. THIS IS ONLY FOR DEMONSTRATION.
    ## PLEASE CHECK OUT 'xrobot/python/py_nav_task.py'

    ## Playground is essential to the simulation environment and rendering engine.
    ## Here we define the width and height of the virtual camera, and also select
    ## the rendering quality and assign it to a dedicated gpu (if you have a multi-gpu
    ## system)

    ## Headless Context
    ## Headless context is used for rendering a scene without associating a window
    ## The default rendering backend is EGL, and GLX Headless is also supported.
    ##
    ## To disable headless context, change the "HEADLESS" to "Debug_Visualization"
    ## for visualizing the scene with a flyover camera.
    ## (This only available when rendering is debugged mode)

    ## Rendering Quality
    ## Rendering Quality can be chosen between low and normal. Low quality does not
    ## have any of post-processing but is quite fast. However, normal quality use SSAO  
    ## to increase some small occlusion-like details and use FXAA to reduce some of 
    ## jagged edges. Exposure control is also available in normal rendering quality.
    ##
    ## To switch to low-quality mode, change the "HEADLESS" to "RENDER_QUALITY_LOW"

    ## Select GPU (Only available in the multi-gpu system)
    ##
    ## Use gpu id to assign the rendering backend to a dedicated gpu
    ## (gpu id can be found in nvidia-smi)

    self.env = Playground(120, # Width \
                          90, # Height \
                          HEADLESS, # Headless Context \
                          RENDER_QUALITY_NORMAL, # Rendering Quality \
                          1) # Select GPU






```

d) Define the stage

First, we define the `start` stage then follow the reset.

```python

def start(self):

    ## Enable inventory feature for grasping object
    self.env.EnableInventory(10) # Maximum capacity

    ## Reset the scene. This will remove any object in the environment including the camera
    self.env.Clear()

    ## Create a square arena with checkerboard-pattern for testing
    self.env.CreateAnTestScene()

    ## Generate a random position (2D) for spawning target object
    x = random.randint(2, 7)
    y = random.randint(2, 7)

    ## Spawn the target object
    
    ## SpawnAnObject
    ## This function can use to generate an object with a certain position, orientation and
    ## uniform scale. Object label has to be assigned for query purposes. Last but not least,
    ## the property of fixed root also need to be assigned. This defines whether or not the
    ## object will be treated as an active rigid body in the simulation. 
    ##
    ## Be aware of orientation representation! It uses valid quaternion for orientation!
    ## [0, 0, 0, 0] is invalid in quaternion!

    self.env.SpawnAnObject("./crate_1/crate.urdf", # Object Path \
                          [x,0,y], # World Position \
                          [1,0,0,0], # World Orientation (Quaternion) \
                          1.0, # Uniform Scale \
                          "Crate", # Unique Label \
                          False) # Fixed Root?


    ## Similar...
    self.agent = self.env.SpawnAnObject("husky/husky.urdf", \
                                        [2,0,2], \
                                        [-1,0,0,1.57], \
                                        1.0, \
                                        "Agent", \
                                        True)

    ## Create and Attach the camera to the agent

    ## AttachCameraTo
    ## This function is to attach the camera to object with slight amount of offset.
    ## This offset will move the camera away from its pivot point 
    self.env.AttachCameraTo(self.agent, [0.3,1.3,0.0])

    ## Initialize the scene. It tells the playground ready for rendering
    self.env.Initialize()

    ## Move to 'navigation' stage
    return "navigation"

```

Then the `navigation` stage

```python

def navigation(self):

    ## Query object with label at forward of robot 
    if self.env.QueryObjectWithLabelAtForward("Crate"):
        # Move to 'idle' stage and wait for next task
        return "idle"

    ## Get agent current position and orientation
    position = self.agent.GetPosition()

    ## GetOrientation
    ## This function returns quaternion!
    orientation = self.agent.GetOrientation()

    ## Fetch Raw Images
    
    ## GetCameraRGBDRaw
    ## This function returns the raw camera capture in unsigned char array which are
    ## 8-bit in each RGBA channel. And alpha channel is actually 8-bit depth channel
    image_str = self.env.GetCameraRGBDRaw()

    ## Convert the color and flip the image
    image_rgbd = np.fromstring(image_str, np.uint8).reshape( 480, 640, 4 )
    image_rgbd = cv2.cvtColor(image_rgbd, cv2.COLOR_BGRA2RGBA)
    image_rgbd = cv2.flip(image_rgbd, 0)
    image_rgb = np.array(image_rgbd[:,:,:3])
    image_depth = np.array(image_rgbd[:,:,3])

    ## Get current framerate and frames already rendered

    ## GetStatus
    ## This function returns a dictionary with playground current status.
    ## Information can be accessed by following keys 'frames', 'framerate' and 'cache'
    frames = "frames: " + str(self.env.GetStatus()["frames"])
    framerate = " | framerate: " + str(self.env.GetStatus()["framerate"])

    ## Put those information on cv mat
    cv2.putText(image_rgb, frames + framerate, (30,30), \
        cv2.FONT_HERSHEY_PLAIN, 1, (200,250,250), 1);

    ## Display result
    cv2.imshow("RGB", image_rgb)

    ## Task not completed! move to 'navigation' stage
    return "navigation"

```

d) Run the task you just created

You can use `xrobot\python\py_taskground_navigation.py` for testing

```python

"""
this tutorial is adapted from the task file below:
xrobot/python/py_taskground_navigation.py
"""
class XRobotEnv(object):
    def __init__(self):

        ## Create playground here
        self.env = Playground(120,90,HEADLESS,RENDER_QUALITY_LOW,0)

        ## Create a task group. A task group is a container for the same type of tasks.
        ## By calling the function 'run_stage' to randomly select a task in the task group.
        self.task_group = TaskGroup("TaskGroup")

        ## Add the task which just created
        self.task_group.add_task("Navigation_SUNCG", XRobot3DNavTarget(self.env))

           ## ...

```

On Ubuntu and derivatives, run:

    PYTHONPATH=..:$PYTHONPATH __GL_SYNC_TO_VBLANK=0 python py_taskground_navigation.py

Do not forget to use `__GL_SYNC_TO_VBLANK=0` to disable V-Sync.
