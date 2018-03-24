from xworld_env import XWorldEnv

class XWorldLanguage(XWorldEnv):
    def __init__(self, item_path, start_level=0):
        super(XWorldLanguage, self).__init__(
            item_path=item_path,
            max_height=5,
            max_width=5,
            maze_generation=False)

    def _configure(self):

        ## one agent in the center of the map
        self.set_entity(type="agent", loc=(2, 2))
        ## the agent is surrounded by four abitary goals
        g_locs = [(3, 2), (1, 2), (2, 1), (2, 3)]
        for i in range(4):
            self.set_entity(type="goal", loc=g_locs[i])
