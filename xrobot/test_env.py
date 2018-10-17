import cv2
import numpy as np
import libxrobot
from teaching_task import *
from test_task import *

class XRobotEnv(object):
    def __init__(self, contexts = 1):
        self.env = libxrobot.Playground(640, 480, libxrobot.HEADLESS)

        self.task_group = TaskGroup("TaskGroup")
        task_nav   = XRobot3DNavTarget(self.env)
        self.task_group.add_task("Navigation", task_nav)

    def reset(self):
        self.env.Clear()

    def get_max_steps(self):
        return 100 # self.gym_env._max_episode_steps

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
        action = 8
    elif key == 51: # kp3
        action = 9

    env.step(action)
    env.render()
