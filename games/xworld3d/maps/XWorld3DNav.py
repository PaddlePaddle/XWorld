from xworld3d_env import XWorld3DEnv
from py_gflags import get_flag
from py_gflags import log_info
import os
import random

class XWorld3DNav(XWorld3DEnv):
    def __init__(self, asset_path):
        super(XWorld3DNav, self).__init__(
            asset_path=asset_path,
            max_height=7,
            max_width=7)
        self.curriculum = get_flag("curriculum")
        self.current_level = 0

    def _configure(self):
        self.set_goal_subtrees(["animal", "others"])
        self.set_entity(type="agent")

        ## compute world dims
        min_dim = 3
        max_h, max_w = self.get_max_dims()
        n_levels = max_h - min_dim + 1
        max_num_goals, min_num_goals = 1, 1
        max_num_blocks, min_num_blocks = 10, 0

        def compute(current_level):
            rate = 0
            if n_levels - 1 > 0:
                rate = float(current_level) / (n_levels - 1)
            return min_dim + current_level, \
                int(rate * (max_num_goals - min_num_goals) + min_num_goals), \
                int(rate * (max_num_blocks - min_num_blocks) + min_num_blocks)

        if self.curriculum == 0:
            progress = 1.0
            current_dim = max_h
            num_goals = max_num_goals
            num_blocks = max_num_blocks
        else:
            flag = False
            if self.current_usage >= self.curriculum \
               and self.current_level < n_levels - 1:
                log_info("~~~~~~ Entering the next level: ~~~~~~~")
                current_dim, num_goals, num_blocks = compute(self.current_level)
                msg = "%dx%d map, %d goals, %d blocks -> " \
                      % (current_dim, current_dim, num_goals, num_blocks)
                ## move to the next stage
                self.current_level += 1
                flag = True

            current_dim, num_goals, num_blocks = compute(self.current_level)
            if flag:
                msg += "%dx%d map, %d goals, %d blocks" \
                       % (current_dim, current_dim, num_goals, num_blocks)
                log_info(msg)

        self.set_dims(current_dim, current_dim)
        ## set goals
        for i in range(num_goals):
            self.set_entity(type="goal")
        ## set blocks
        for i in range(num_blocks):
            self.set_entity(type="block")
