XWorld3D
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
- Look at `API.md` for the usage of creating an environment and controlling the robot
- Look at `python_example/` for defining tasks


# Code Overview

The code in the `xrobot/directory` is organized as follows:

- data/ contains all necessary models and textures for running basic tests and tutorials
- vendor/ contains code for loading json and images
- cpp_example/ contains c++ example
- python_example/ contains python example for creating tasks
- render_engine/ contains code for rendering the scene
    - render_engine/EGL/ contains EGL extensions
    - render_engine/shaders/ contains various shaders which are necessary for render engine

# Performance

On Ubuntu and derivatives, use following script to benchmark:

    sh run_benchmark.sh --num-proc 3 --num-gpu 1

The results are approximate framerates in each process (include rendering and simulation). The total framerate should reach approx. 500 - 1500 fps on decent Nvidia GPU with EGL backend.

# Python Tutorial

## Write a Navigation Task

Some examples can be found in `xrobot/python_example`. Below we show the step-by-step code to create a basic navigation task. Check out `API.md` for more details.


a) Import Python modules

```python

from teaching_task import *
from libxrobot import *

```

b) Create a task by implementing base class `XWorld3DTask` and define the member function for each stage

```python
"""
this tutorial is adapted from the task file below:
xrobot/python_example/XWorld3DNavTarget.py
"""

class XWorld3DNavTarget(XWorld3DTask):
    def __init__(self, env):
        super(XWorld3DNavTarget, self).__init__(env)

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

    def _define_grammar(self):

```

c) Implement each stage we previously defined

```python
def start(self):
    ## Reset the entire scene
    self.reset()
    ## Create a square arena (build-in testing environment)
    self.env.CreateArena(3, 3)


    ## Generate random position and orientation for the goal
    pos_x = random.uniform(2, 4)
    pos_z = random.uniform(2, 4)
    orn_y = random.uniform(0, 2)


    ## SpawnAnObject(path, position, orientation, scale, label, fixed)
    ##
    ## This function can populate an object with user defined position, orientation and
    ## uniform scale. The label parameter needs to be assigned by a string for quering purposes. 
    ## Last but not least, the fixed parameter defines whether or not the object
    ## will be treated as an active rigid body in the simulation. 
    ##
    ## The orientation uses axis-angle representation. 
    ## For exmaple, [1, 0, 0, 1.57] means object rotate around axis [1, 0, 0] 90 degrees.

    ## Populate a static goal in the scene.
    self.env.SpawnAnObject(goal.path, [pos_x, 0, pos_z], \
        [0, 1, 0, orn], 1.0, goal.label, True)

    ## Populate the agent in the scene.
    self.agent = self.env.SpawnAnObject(agent.path, [0, 0, 0], \
        [-1, 0, 0, 1.57], 1.0, agent.label, True)


    ## AttachCameraTo(object, position)
    ##
    ## This function can attach the camera to object with slight amount of offset.

    ## Attach the virtual camera to the agent
    self.env.AttachCameraTo(self.agent, [0.0,1.0,0.0])

    ## Initialize the scene. It tells the playground ready for rendering
    self.env.Initialize()

    ## Move to 'navigation' stage
    return ["navigation", 0.0, ""]
```


```python
def navigation(self):
    next_stage = "navigation"

    ## QueryObjectWithLabelAtCameraCenter(label)
    ##
    ## Return true if a object with label is at the center of camera and 
    ## within a certain distance.

    ## Reset the game if agent find goal
    if self.env.QueryObjectWithLabelAtCameraCenter(goal.label):
        next_stage = "idle"

    ## GetCameraRGBDRaw()
    ##
    ## This function returns the raw camera capture in unsigned char array which are
    ## 8-bit in each RGBA channel. And alpha channel is actually 8-bit depth channel

    ## Fetch RGBD data from virtual camera
    image_str = self.env.GetCameraRGBDRaw()
    image_rgbd = np.fromstring(image_str, np.uint8).reshape(480, 640, 4)

    return [next_stage, 0.0, ""]
```

d) Define the grammar

```python
 def _define_grammar(self):
    all_goals_names = \
        self._get_all_goal_names_as_rhs([goal.label])
    grammar_str = """
    S --> navigation | correct | wrong | timeup
    navigation -> I0
    correct -> 'Well' 'done' '!'
    wrong -> 'Wrong' '!'
    timeup -> 'Time' 'up' '!'
    I0 -> 'find' G | 'navigate' 'to' G
    G --> %s
    """ % all_goals_names
    return grammar_str, "S"
```

e) Complete the grammar defination in all stages

```python
    def start(self):
        ## ...
        
        ## Generate sentence
        self._bind("S -> navigation")
        self.sentence = self._generate()
        
        return ["navigation", 0.0, self.sentence]


    def navigation(self):
        next_stage = "navigation"
        reward, time_out = self._time_reward()
        
        if not time_out:
            if self.env.QueryObjectWithLabelAtCameraCenter( \
                goal.label):
                next_stage = "idle"
                reward = self._successful_goal(reward)
            
        ## ...

        return [next_stage, reward, self.sentence]
```

f) Run the task

You also can use `xrobot\python_example\XWorld3DNavTarget_TaskGroup.py` for testing

```python
class XWorld3DEnv(object):
    def __init__(self):

        ## Create playground here
        self.env = Playground(640,480,HEADLESS,RENDER_QUALITY_LOW,0)

        ## Create a task group. A task group is a container for the same type of tasks.
        ## By calling the function 'run_stage' to randomly run a task in the task group.
        self.task_group = TaskGroup("TaskGroup")
        self.task_group.add_task("Navigation_Task", XWorld3DNavTarget(self.env))

    def reset(self):
        self.env.Clear()

    def step(self, action):
        self.task_group.run_stage()
        self.env.UpdateSimulationWithAction(action)

    def render(self):
        self.env.UpdateRenderer()

env = XWorld3DEnv()
env.reset();

while True:
    action = NO_ACTION 

    # Action inputs from keyboard
    key = cv2.waitKey(1)
    if key == 119:   # W
        action = 0
    elif key == 97:  # A
        action = 2
    elif key == 115: # S
        action = 1
    elif key == 100: # D
        action = 3
    elif key == 27: # ESC
        break

    # Update
    env.step(action)
    env.render()

```

On Ubuntu and derivatives, run:

    PYTHONPATH=..:$PYTHONPATH __GL_SYNC_TO_VBLANK=0 python py_taskground_navigation.py

Do not forget to use `__GL_SYNC_TO_VBLANK=0` to disable V-Sync.