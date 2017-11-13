"""
Copyright (c) 2017 Baidu Inc. All Rights Reserved.

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
"""

import random
import os
import copy
from itertools import groupby
import itertools
import pprint

"""
Entity:
  id         - unique str for this entity
  type       - "agent", "goal", "block"
  location   - (x, y)
  asset_path - the model path
  name       - name of the entity
  color      - color of the entity
"""
class Entity:
    def __init__(self, type, id=None, loc=None, name=None, asset_path=None, color=None):
        if not loc is None:
            assert isinstance(loc, tuple) and len(loc) == 3
        self.type = type
        self.id = id
        self.loc = loc
        self.name = name
        self.asset_path = asset_path
        self.color = color

class XWorld3DEnv(object):
    def __init__(self, item_path, max_height=10, max_width=10):
        self.num_games = -1
        ## load all items from item_path
        self.grid_types = ["goal", "block", "agent"]
        self.item_path = item_path
        self.max_height = max_height
        self.height = max_height
        self.max_width = max_width
        self.width = max_width
        self.all_object_paths = []
        for dirpath, _, files in os.walk(item_path):
            for f in files:
                if f.endswith(".urdf"):
                    self.all_object_paths.append(os.path.join(dirpath, f))
        self.set_goal_subtrees([])
        ## init dimensions
        self.__clean_env()
        ## read item colors
        color_file = os.path.join(item_path, "properties.txt")
        assert os.path.exists(color_file)
        with open(color_file, "r") as f:
            lines = f.read().splitlines()
        self.color_table = {os.path.join(item_path, l.split()[0]) : l.split()[1]\
                            for l in lines if not l.startswith("//") and not l == ""}

    ############################ interface with Python tasks ############################
    def reset(self):
        """
        Reset the map.
        """
        self.__clean_env()
        self._configure()
        self.__instantiate_entities()

    def get_num_games(self):
        """
        How many sessions the agent has played
        """
        return self.num_games

    def set_dims(self, h, w):
        """
        Set the dimensions of the map. If h or w is less than self.max_height or
        self.max_width, then walls will be automatically padded. The python user should
        use coordinates in [0, h) and [0, w).
        """
        assert h > 1 and w > 1
        assert h <= self.max_height and w <= self.max_width
        self.height = h
        self.width = w
        self.boundaries = self.__add_boundaries()
        self.available_grids = list(set(itertools.product(range(w), range(h))))
        random.shuffle(self.available_grids)
        self.changed = True

    def set_entity(self, type, loc=None, name=None):
        """
        Add an entity of type to loc which must be currently empty
        """
        if not loc is None:
            assert loc in self.available_grids, \
                "set_dims correctly before setting a location"
            self.available_grids.remove(loc)
        self.entity_nums[type] += 1
        self.entities.append(Entity(type=type, loc=loc, name=name))
        self.changed = True

    def delete_entity(self, loc=None, id=None):
        """
        Delete an entity on the current map either by its location or id
        """
        assert not (loc is None and id is None), "must specify what to delete"
        xe = [e for e in self.entities if e.loc == loc or e.id == id]
        assert len(xe) == 1
        x = xe[0]
        self.entities.remove(x)
        self.entity_nums[x.type] -= 1
        self.available_grids.append(x.loc)
        self.changed = True

    def set_goal_subtrees(self, subtrees):
        """
        Set goal directory substrees so that only goals in the selected subtrees
        will be sampled when generating the map. The user can use this function to
        control the number of goal classes.
        The change of goal subtrees will only be reflected for the next game, after
        reset() is called. The current game still uses old goal subtrees.
        """
        goal_path = os.path.join(self.item_path, "goal")
        self.object_paths = copy.deepcopy(self.all_object_paths)
        if len(subtrees) > 0:
            self.object_paths \
                = [p for p in self.object_paths \
                   if not p.startswith(goal_path) or p.split("/")[-3] in subtrees]
        ## get a hierarchy of all possible objects
        key = lambda p: '_'.join(p.split('_')[:-1])
        objects = groupby(sorted(self.object_paths, key=key), key=key)

        self.items = {t : {} for t in self.grid_types}
        for k, g in objects:
            type = [t for t in k.split("/") if t in self.grid_types][0]
            assert type in self.items
            self.items[type][os.path.basename(k)] = list(g)

    def get_dims(self):
        """
        Get the height and width of the map
        """
        return (self.height, self.width)

    def get_n(self, type):
        """
        Get the current number of entities on the map for type
        """
        assert type in self.entity_nums
        return self.entity_nums[type]

    def get_all_possible_names(self, type):
        """
        Return all possible names for type
        'goal'  - all unique object names
        'block' - all block names
        'agent' - all agent names
        """
        return self.items[type].keys()

    def get_all_colors(self):
        """
        Return all possible colors in xworld
        """
        return list(set(self.color_table.values()))

    def get_agent(self):
        """
        Get the agent information: (entity, agent sentence, action success)
        """
        agent = [e for e in self.entities if e.type == "agent"][0]
        return (agent, self.agent_sent, self.action_successful)

    def get_goals(self):
        """
        Return all the goals on the current map
        """
        return [e for e in self.entities if e.type == "goal"]

    def get_blocks(self):
        """
        Return all the blocks on the current map
        """
        return [e for e in self.entities if e.type == "block"]

    def get_entities(self):
        """
        Return all the entities on the current map
        """
        return self.entities

    ######################## interface with C++ #############################
    def env_changed(self):
        """
        Whether the environment has been changed by the teacher during the current
        stage of the task. If yes, then teaching_task.cpp will notify the simulator to update
        the game environment.
        """
        ret = self.changed
        self.changed = False
        return ret

    def cpp_get_entities(self):
        """
        C++ code gets entities information. Used by the underlying simulator.
        C++ entity is in 3D so we need to add an additional 0.
        A 3D entity is compatible with different games.
        """
        actual_entities = [e.__dict__ for e in self.entities]
        boundary_entities = [e.__dict__ for e in self.boundaries]
        return actual_entities + boundary_entities

    def update_entities_from_cpp(self, entities):
        """
        Update the environment from C++. The changes might be due to
        the environment dynamics or the agent's actions.
        Entities is a list of python dicts.
        We only keep the first two dimensions of a C++ entity.
        """
        self.entity_nums = {t : 0 for t in self.grid_types}
        self.entities = [Entity(**i) for i in entities if not self.__is_boundary(i["loc"])]
        for e in self.entities:
            self.entity_nums[e.type] += 1

    def update_agent_sentence_from_cpp(self, sent):
        """
        Update the agent sentence from the CPP simulator
        """
        self.agent_sent = sent

    def update_agent_action_success_from_cpp(self, successful):
        """
        Update the agent action success from the CPP simulator
        """
        self.action_successful = successful

    ######################## private or protected #########################
    def _configure(self):
        """
        The user has to override this function to define how the map
        will be generated after each session resetting
        """
        raise NotImplementedError()

    def __instantiate_entities(self):
        """
        For each entity, select an instance from the object class it belongs to,
        after which its properties are set.
        The entities should have been set in _configure()
        """
        ## select a random object path for each entity
        for i, e in enumerate(self.entities):
            if e.name is None:
                e.name = random.choice(self.get_all_possible_names(e.type))
            icons = self.items[e.type][e.name]
            e.asset_path = random.choice(icons)
            e.id = "%s_%d" % (e.name, i)
            e.color = self.color_table[e.asset_path]
            if e.loc is None:
                assert len(self.available_grids) > 0
                e.loc = self.available_grids[0] + (0, )
                self.available_grids = self.available_grids[1:]

    def __add_boundaries(self):
        """
        Given the max height and width of a map, return a list of surrounding
        wall blocks.
        """
        wall_blocks = []
        def add_blocks(range1, range2, id):
            for loc in itertools.product(range1, range2):
                print(loc+(0,))
                wall_blocks.append(Entity(type="block", loc=loc+(0,), id="boundary_%d" % id,
                                          name="boundary", color="na",
                                          asset_path=self.items["block"]["boundary"][0]))
                id += 1
            return id
        id = add_blocks(range(-1, self.max_width+1), (-1, self.max_height),
                        self.max_height * self.max_width)
        id = add_blocks((-1, self.max_width), range(0, self.max_height), id);
        return wall_blocks

    def __is_boundary(self, loc):
        """
        Given a location, determine whether it's a padding block or not
        """
        x, y, _ = loc
        return not (x >= 0 and x < self.width and \
                    y >= 0 and y < self.height)

    def __clean_env(self):
        """
        Reset members; preparing for the next session
        """
        self.num_games += 1
        self.agent_sent = ""
        self.changed = False
        self.entities = []
        self.boundaries = []
        self.entity_nums = {t : 0 for t in self.grid_types}
        self.available_grids = list(itertools.product(range(self.max_width), range(self.max_height)))
