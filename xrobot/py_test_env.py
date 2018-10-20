import cv2
import numpy as np
from libxrobot import *
from teaching_task import *
from py_nav_task import *
from py_suncg_task import *
from py_random_task import *
from py_nav_agent_task import *

class XRobotEnv(object):
    def __init__(self):
        self.env = libxrobot.Playground(640, \
                                        480, \
                                        # DEBUG_VISUALIZATION, \
					                    HEADLESS, \
                                        RENDER_QUALITY_LOW)

        self.task_group = TaskGroup("TaskGroup")
        self.task_group.add_task("Navigation_1", XRobot3DRandom(self.env))
        # self.task_group.add_task("Navigation_2", XRobot3DNavAgentTarget(self.env))

    def reset(self):
        self.env.Clear()

    def step(self, action, actrep=1):
        self.task_group.run_stage()
        return self.env.UpdateSimulationWithAction(action)

    def render(self):
        self.env.UpdateRenderer()

    def preprocess_observation(self, ob):
        return ob.astype("float32")

    def observation_dims(self):
        return self.env.GetObservationSpace();

    def action_dims(self):
        return self.env.GetActionSpace();

    def game_over(self):
        return False


env = XRobotEnv()
env.reset();

while (not env.game_over()):

    action = 13; # Do Nothing

    # action inputs from keyboard
    key = cv2.waitKey(1)
    if key == 119:   # W
        action = 0
    elif key == 97:  # A
        action = 2
    elif key == 115: # S
        action = 1
    elif key == 100: # D
        action = 3
    elif key == 48: # 9
        action = 4
    elif key == 57: # 0
        action = 5
    elif key == 54: # kp6
        action = 11
    elif key == 51: # kp3
        action = 12
    elif key == 27: # ESC
        break;

    # update
    env.step(action)
    env.render()
