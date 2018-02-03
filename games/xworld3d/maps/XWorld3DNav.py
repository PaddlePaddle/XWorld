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
        self.set_goal_subtrees(["animal", "others", "furniture"])
        self.set_entity(type="agent")

        goal_names = self.get_all_possible_names("goal")

        ## compute world dims
        min_dim = 3
        max_h, max_w = self.get_max_dims()
        n_levels = max_h - min_dim + 1
        max_num_goals, min_num_goals = 2, 2
        max_goal_classes, min_goal_classes = len(goal_names), len(goal_names)
        max_num_blocks, min_num_blocks = 10, 1

        def compute(current_level):
            def interpolate(min_, max_, rate):
                return int(rate * (max_ - min_) + min_)

            rate = 0
            if n_levels - 1 > 0:
                rate = float(current_level) / (n_levels - 1)
            return min_dim + current_level, \
                interpolate(min_num_goals, max_num_goals, rate), \
                interpolate(min_num_blocks, max_num_blocks, rate), \
                interpolate(min_goal_classes, max_goal_classes, rate)

        if self.curriculum == 0:
            progress = 1.0
            current_dim = max_h
            num_goals = max_num_goals
            num_blocks = max_num_blocks
            num_goal_classes = max_goal_classes
        else:
            flag = False
            if self.get_current_usage() >= self.curriculum \
               and self.current_level < n_levels - 1:
                log_info("~~~~~~ Entering the next level: ~~~~~~~")
                current_dim, num_goals, num_blocks, num_goal_classes = compute(self.current_level)
                msg = "%dx%d map, %d goals, %d goal classes, %d blocks -> " \
                      % (current_dim, current_dim, num_goals, num_goal_classes, num_blocks)
                ## move to the next stage
                self.current_level += 1
                flag = True

            current_dim, num_goals, num_blocks, num_goal_classes = compute(self.current_level)
            if flag:
                msg += "%dx%d map, %d goals, %d goal classes, %d blocks" \
                       % (current_dim, current_dim, num_goals, num_goal_classes, num_blocks)
                log_info(msg)

        self.set_dims(current_dim, current_dim)
        ## set goals
        goal_names = sorted(goal_names)[:num_goal_classes]
        random.shuffle(goal_names)
        assert num_goal_classes >= num_goals
        for i in range(num_goals):
            self.set_entity(type="goal", name=goal_names.pop())
        ## set blocks
        for i in range(num_blocks):
            self.set_entity(type="block", name=random.choice(["brick"]))
