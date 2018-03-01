from xworld_env import XWorldEnv

class XWorldDialogMap(XWorldEnv):
    def __init__(self, item_path, start_level=0):
        super(XWorldDialogMap, self).__init__(
            item_path=item_path, max_height=3, max_width=3, class_per_session=2)

    def _configure(self):
        self.set_dims(3, 3)
        self.set_entity(type="goal", loc=(1, 0))
        self.set_entity(type="agent", loc=(1, 1))
