from xworld_env import XWorldEnv

class XWorldLanguage(XWorldEnv):
    def __init__(self, item_path, start_level=0):
        super(XWorldLanguage, self).__init__(
            item_path=item_path)

    def _configure(self):
        self.set_dims(5, 5)
        ## one agent in the center of the map
        self.set_entity(type="agent")
        ## the agent is surrounded by four abitary goals
        for i in range(4):
            self.set_entity(type="goal")
