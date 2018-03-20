from xworld_env import XWorldEnv
import os
import random

class XWorldWalls(XWorldEnv):
    def __init__(self, item_path, start_level=0):
        super(XWorldWalls, self).__init__(
            item_path=item_path,
            max_height=7,
            max_width=7,
            start_level=start_level,
            maze_generation=False)

    def _configure(self):
        self.set_dims(7, 7)
        self.set_goal_subtrees(["animal", "fruit", "shape"])
        self.set_entity(type="agent")
        for i in range(12):
            self.set_entity(type="goal")

        ## make a vertical wall and a horizontal wall
        n_blocks = 12
        # pick a random row
        row = random.randint(0, self.height - 1)
        for i in range(min(n_blocks, self.width)):
            self.set_entity(loc=(i, row, 0), type="block")
        n_blocks -= min(n_blocks, self.width)
        # pick a random column
        column = random.randint(0, self.width - 1)

        i, j = 0, 0
        while j < min(n_blocks, self.height - 1):
            if i != row:
                self.set_entity(loc=(column, i, 0), type="block")
                j += 1
            i += 1
