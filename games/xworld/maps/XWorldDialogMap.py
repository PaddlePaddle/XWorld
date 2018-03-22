import random
from xworld_env import XWorldEnv
from py_util import overrides

class XWorldDialogMap(XWorldEnv):
    def __init__(self, item_path, start_level=0):
        super(XWorldDialogMap, self).__init__(
            item_path=item_path, max_height=1, max_width=1)
        self.class_per_session = 2 # max number of classes in a session
                                   # value < 1 denotes all classes are used
        self.sel_classes = {} # selected classes for a session

    def _configure(self, select_class=True):
        self.set_dims(1, 1)
        self.set_entity(type="agent", loc=(0, 0), force_occupy=True)
        self.set_entity(type="goal", loc=(0, 0), force_occupy=True)
        if select_class:
            self.select_goal_classes() # re-select goal class for a new session

    @overrides(XWorldEnv)
    def get_all_possible_names(self, type):
        """
        Return all possible names for type
        'goal'  - all unique object names
        'block' - all block names
        'agent' - all agent names
        """
        if type == "goal":
            return self.get_selected_goal_classes()
        else:
            return self.items[type].keys()

    def select_goal_classes(self):
        """
        Sample a number of classes (class_per_session) for interaction within a session
        """
        if self.class_per_session > 1:
            self.sel_classes = random.sample(self.items["goal"].keys(), self.class_per_session)
        else:
            self.sel_classes = self.items["goal"].keys()

    def get_selected_goal_classes(self):
        """
        Get the selected classes for a session
        """
        if not self.sel_classes:
            self.sel_classes = self.select_goal_classes()
        return self.sel_classes

    def within_session_reinstantiation(self):
        # re-instantiate within the same session
        # re-load from map config with the same set of sampled classes
        for e in self.get_goals():
            self.set_property(e, property_value_dict={"name" : True})
