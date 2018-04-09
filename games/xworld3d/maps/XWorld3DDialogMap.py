from xworld3d_env import XWorld3DEnv
from py_gflags import get_flag
import os
import random
from py_util import overrides

class XWorld3DDialogMap(XWorld3DEnv):
    def __init__(self, asset_path, start_level=0):
        super(XWorld3DDialogMap, self).__init__(
            asset_path=asset_path,
            max_height=3,
            max_width=3,
            maze_generation=False)
        self.class_per_session = 2 # max number of classes in a session
                                   # value < 1 denotes all classes are used
        self.sel_classes = {} # selected classes for a session
        self.shuffle = True # shuffle classes

    def _configure(self, select_class=True):
        self.set_goal_subtrees([ "others", "furniture"])
        ## these are all the object classes
        self.set_dims(3, 3)

        if select_class:
            self.select_goal_classes() # re-select goal class for a new session

        if self.shuffle:
            self.shuffle_classes("goal")

        self.set_entity(type="agent", loc=(2, 1, 0))
        self.set_entity(type="goal", loc=(2, 2, 0))

        for e in self.get_goals():
            self.set_property(e, property_value_dict={"name" : None, "yaw" : None})
        a, _, _ = self.get_agent()
        self.set_property(a, property_value_dict={"yaw" : 3.14/2})

    @overrides(XWorld3DEnv)
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

    def shuffle_classes(self, type):
        K = self.items[type].keys()
        V = self.items[type].values()
        random.shuffle(V)
        self.items[type].update(dict(zip(K, V)))

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
            self.select_goal_classes()
        return self.sel_classes

    def within_session_reinstantiation(self):
        # re-instantiate within the same session
        # re-load from map config with the same set of sampled classes
        for e in self.get_goals():
            self.set_property(e, property_value_dict={"asset_path" : None, "yaw" : None})
