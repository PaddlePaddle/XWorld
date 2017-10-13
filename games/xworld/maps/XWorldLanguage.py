from xworld_env import XWorldEnv

class XWorldLanguage(XWorldEnv):
    def __init__(self, item_path):
        super(XWorldLanguage, self).__init__(
            item_path=item_path)

    def _configure(self):
        self.set_dims(5, 5)
        ## one agent in the center of the map
        self.set_entity(type="agent", loc=(2, 2))
        ## the agent is surrounded by four abitary goals
        self.set_entity(type="goal", loc=(3, 2))
        self.set_entity(type="goal", loc=(1, 2))
        self.set_entity(type="goal", loc=(2, 1))
        self.set_entity(type="goal", loc=(2, 3))
