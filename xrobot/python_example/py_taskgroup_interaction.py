import cv2
import numpy as np
import time
from libxrobot import *
from teaching_task import *
from py_interaction_task import *

class XRobotEnv(object):
    def __init__(self):
        self.env = Playground(640, \
                              360, \
		                      DEBUG_VISUALIZATION, \
                              RENDER_QUALITY_NORMAL, \
                              1)

        #DEBUG_VISUALIZATION

        self.task_group = TaskGroup("TaskGroup")
        self.task_group.add_task("Navigation_1", XRobot3DInteractions(self.env))

    def reset(self):
        self.env.Clear()

    def step(self, action):
        self.task_group.run_stage()
        self.env.UpdateSimulationWithAction(action)

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

start = time.time()
while (not env.game_over()):

    action = NO_ACTION # Do Nothing

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
    elif key == 49:  # 1 Pick
        action = 8
    elif key == 50:  # 2 Drop
        action = 9
    elif key == 48:  # kp9 Up
        action = 4
    elif key == 57: # kp0 Down
        action = 5
    elif key == 54: # kp6 Open
        action = 12
    elif key == 51: # kp3 Close
        action = 13
    elif key == 55: # kp7 Enable Interact
        action = 11
    elif key == 56: # kp8 Disable Interact
        action = 13
    elif key == 27: # ESC
        break

    # update
    env.step(action)
    env.render()