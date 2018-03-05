from xworld3d_env import XWorld3DEnv
from py_gflags import get_flag
import os
import random

class XWorld3DNav(XWorld3DEnv):
    def __init__(self, asset_path, start_level=0):
        super(XWorld3DNav, self).__init__(
            asset_path=asset_path,
            max_height=7,
            max_width=7)
        self.curriculum = get_flag("curriculum")
        self.current_level = start_level

    def _configure(self):
        self.set_goal_subtrees(["animal", "others", "furniture"])

        ## these are all the object classes
        goal_names = self.get_all_possible_names("goal")

        ## compute world dims
        min_dim = 3
        max_h, max_w = self.get_max_dims()
        n_levels = max_h - min_dim + 1

        #### define curriculum #########
        num_goals_seq = [3, 3, 3, 3, 3]
        goal_classes_seq = [len(goal_names)] * n_levels
        num_blocks_seq = [0, 2, 4, 7, 10]
        assert len(num_goals_seq) == n_levels \
            and len(goal_classes_seq) == n_levels \
            and len(num_blocks_seq) == n_levels
        ################################

        def compute(current_level):
            return min_dim + current_level, \
                num_goals_seq[current_level], \
                num_blocks_seq[current_level], \
                goal_classes_seq[current_level]

        if self.curriculum == 0:
            current_dim = max_h
            num_goals = num_goals_seq[-1]
            num_blocks = num_blocks_seq[-1]
            num_goal_classes = goal_classes_seq[-1]
        else:
            flag = False
            if self.get_current_usage() >= self.curriculum \
               and self.current_level < n_levels - 1:
                print("~~~~~~ Entering the next level: ~~~~~~~")
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
                print(msg)


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
        self.set_entity(type="agent")
