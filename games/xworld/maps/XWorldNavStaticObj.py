from xworld_env import XWorldEnv
from py_gflags import get_flag
import os
import random

class XWorldNavStaticObj(XWorldEnv):
    """
    This example map config illustrates how to set some static objects with fixed locations,
    while allowing the co-eixstence of objects with flexible locations. This is achieved by
    setting maze_generation flag to False and specifying loc value for those static objects.
    Note that setting maze_generating flag to True, a maze generation algorithm will be used
    and all the pre-specified locations will be over-written.
    """
    def __init__(self, item_path, start_level=0):
        super(XWorldNavStaticObj, self).__init__(
            item_path=item_path,
            max_height=7,
            max_width=7,
            maze_generation=False)

    def _configure(self):
        self.set_goal_subtrees(["animal", "fruit", "shape"])
        self.set_entity(type="agent", loc=(0, 0))
        self.set_entity(type="goal", loc=(5, 5))

        ## set blocks  with fixed locations
        for i in range(2):
            self.set_entity(type="block", loc=(3, i))
        for i in range(5, 7):
            self.set_entity(type="block", loc=(3, i))
        for i in range(2):
            self.set_entity(type="block", loc=(i, 3))
        for i in range(5, 7):
            self.set_entity(type="block", loc=(i, 3))

        ## set some random blocks
        for i in range(2):
            self.set_entity(type="block")

        ## set goals
        for i in range(2):
            self.set_entity(type="goal")
