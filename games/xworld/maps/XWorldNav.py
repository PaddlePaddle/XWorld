from xworld_env import XWorldEnv
from py_gflags import get_flag
import os
import random

class XWorldNav(XWorldEnv):
    def __init__(self, item_path):
        super(XWorldNav, self).__init__(
            item_path=item_path,
            max_height=5,
            max_width=5)
        self.curriculum = get_flag("curriculum")
        self.current_level = 0

    def _configure(self):
        self.set_goal_subtrees(["animal", "fruit", "shape"])
        self.set_entity(type="agent")

        ## compute world dims
        min_dim = 2
        max_h, max_w = self.get_max_dims()
        n_levels = max_h - min_dim + 1
        max_num_goals, min_num_goals = 1, 1
        max_num_blocks, min_num_blocks = 2, 0

        if self.curriculum == 0:
            progress = 1.0
            current_dim = max_h
            num_goals = max_num_goals
            num_blocks = max_num_blocks
        else:
            if self.current_usage >= self.curriculum:
                ## move to the next stage
                self.current_level = min(self.current_level + 1, n_levels - 1)
            current_dim = min_dim + self.current_level
            rate = 0
            if n_levels - 1 > 0:
                rate = float(self.current_level) / (n_levels - 1)
            num_goals = int(rate * (max_num_goals - min_num_goals) + min_num_goals)
            num_blocks = int(rate * (max_num_blocks - min_num_blocks) + min_num_blocks)

        self.set_dims(current_dim, current_dim)
        ## set goals
        for i in range(num_goals):
            self.set_entity(type="goal")
        ## set blocks
        for i in range(num_blocks):
            self.set_entity(type="block")
