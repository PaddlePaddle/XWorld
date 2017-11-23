from xworld3d_env import XWorld3DEnv
from py_gflags import get_flag
import os
import random

class XWorld3DNav(XWorld3DEnv):
    def __init__(self, item_path):
        super(XWorld3DNav, self).__init__(
            item_path=item_path,
            max_height=7,
            max_width=7)
        self.curriculum = get_flag("curriculum")

    def _configure(self):
        self.set_goal_subtrees(["animal", "others"])
        self.set_entity(type="agent")

        num_games = self.get_num_games()

        ## compute world dims
        min_dim = 3
        max_h, max_w = self.get_dims()

        if self.curriculum == 0:
            progress = 1.0
            delta_dim = max_h - min_dim
        else:
            progress = min(float(num_games) / self.curriculum, 1.0)
            delta_dim = int(progress * (max_h - min_dim))
        self.set_dims(min_dim + delta_dim, min_dim + delta_dim)

        ## set goals
        min_num_goal = 1
        num_goals = int(progress * 2) + min_num_goal
        for i in range(num_goals):
            self.set_entity(type="goal")

        ## set blocks
        num_blocks = int(progress * 10)
        for i in range(num_blocks):
            self.set_entity(type="block")
