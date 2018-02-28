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

import copy
import itertools
import numbers
import os
import random
from maze2d import spanning_tree_maze_generator

"""
Entity:
  id            - unique str for this entity
  grid_types    - "agent", "goal", "block", "boundary"
  location      - (x, y, z)
  yaw           - in radian
  scale         - (0, 1.0]
  offset        - [0, 1-scale]
  name          - name of the entity
  asset_path    - the model path
  color         - color of the entity
"""
class Entity:
    def __init__(self, type, id=None, loc=None, yaw=0.0,
                 scale=1.0, offset=0.0, name=None, asset_path=None, color=None):
        if not loc is None:
            assert isinstance(loc, tuple) and len(loc) == 3
        self.type = type
        self.id = id
        self.loc = loc
        self.yaw = yaw
        self.scale = scale
        self.offset = offset
        self.name = name
        self.asset_path = asset_path
        self.color = color

class XWorld3DEnv(object):
    PI_2 = 1.5707963
    PI = 3.1415926

    curriculum_check_period = 100

    def __init__(self, asset_path, max_height=10, max_width=10):
        self.current_usage = {}
        self.action_successful = False
        self.grid_types = ["goal", "block", "agent", "boundary"]
        ## init dimensions
        self.max_height = max_height
        self.max_width = max_width
        self.__clean_env()
        ## event messages
        self.action_successful = False
        self.agent_sent = ""
        self.game_event = ""

        self.curriculum_check_counter = 0

        ## load all items from asset_path
        self.asset_path = asset_path
        self.all_object_paths = []
        for dirpath, _, files in os.walk(asset_path):
            for f in files:
                if f.endswith(".urdf"):
                    self.all_object_paths.append(os.path.join(dirpath, f))
        self.set_goal_subtrees([])
        ## read item properties
        color_file = os.path.join(asset_path, "properties.txt")
        assert os.path.exists(color_file)
        with open(color_file, "r") as f:
            lines = f.read().splitlines()
        self.color_table = {os.path.join(asset_path, l.split()[0]) : l.split()[1]\
                           for l in lines if not l.startswith("//") and not l == ""}

    ############################ interface with Python tasks ############################
    def reset(self):
        """
        Reset the map.
        """
        self.__clean_env()
        self._configure()
        self.__instantiate_entities()

    def get_current_usage(self):
        self.curriculum_check_counter += 1
        if self.curriculum_check_counter < XWorld3DEnv.curriculum_check_period \
           or not self.current_usage:
            return 0
        ## we take the min usage across all the tasks
        usage = min([sum(l) / float(len(l)) for l in self.current_usage.values()])
        self.curriculum_check_counter = 0
        return usage

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
        self.available_grids = list(set(itertools.product(range(w), range(h), (0,))))
        random.shuffle(self.available_grids)
        self.changed = True

    def set_entity(self, type, loc=None, name=None):
        """
        Add an entity of type to loc which must be currently empty
        """
        self.__set_entity(Entity(type=type, loc=loc, name=name))

    def __set_entity(self, e):
        ## agent has continuous loc, so we don't check
        if not e.loc is None and e.type != "agent":
            assert e.loc in self.available_grids, \
                "set_dims correctly before setting a location"
            self.available_grids.remove(e.loc)
        self.entity_nums[e.type] += 1
        self.entities.append(e)
        self.changed = True

    def delete_entity(self, id):
        """
        Delete an entity on the current map either by its id
        """
        xe = [e for e in self.entities if e.id == id]
        assert len(xe) == 1
        x = xe[0]
        self.entities.remove(x)
        self.entity_nums[x.type] -= 1
        ## agent has continuous loc, so we don't restore
        if x.type != "agent":
            self.available_grids.append(x.loc)
        self.changed = True

    def move_entity(self, e, loc):
        """
        Move entity e from its current location to loc
        """
        self.delete_entity(id=e.id)
        e.loc = loc
        self.__set_entity(e)

    def set_goal_subtrees(self, subtrees):
        """
        Set goal directory substrees so that only goals in the selected subtrees
        will be sampled when generating the map. The user can use this function to
        control the number of goal classes.
        The change of goal subtrees will only be reflected for the next game, after
        reset() is called. The current game still uses old goal subtrees.
        """
        goal_path = os.path.join(self.asset_path, "goal")
        self.object_paths = copy.deepcopy(self.all_object_paths)
        if len(subtrees) > 0:
            self.object_paths \
                = [p for p in self.object_paths \
                   if not p.startswith(goal_path) or p.split("/")[-3] in subtrees]
        ## get a hierarchy of all possible objects
        key = lambda p: '_'.join(p.split('_')[:-1])
        objects = itertools.groupby(sorted(self.object_paths, key=key), key=key)

        self.items = {t : {} for t in self.grid_types}
        for k, g in objects:
            type = [t for t in k.split("/") if t in self.grid_types][0]
            assert type in self.items
            self.items[type][os.path.basename(k)] = list(g)

    def get_max_dims(self):
        """
        Get the max height and max width of the map
        """
        return (self.max_height, self.max_width)

    def get_dims(self):
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

    def get_available_grids(self):
        """
        Return all the available grids on the current map
        """
        return self.available_grids

    def get_entities(self):
        """
        Return all the entities on the current map
        """
        return self.entities

    def record_environment_usage(self, task_name, x):
        """
        Update the current environment usage
        The higher the usage is, the better the agent handles the environment (so
        it might be a good time now to move to more difficult scenarios)
        This quantity can be used to generate a curriculum of the world
        """
        self.current_usage[task_name] = x

    ######################## interface with C++ #############################
    def dump_curriculum_progress(self):
        return self.current_level

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
        """
        actual_entities = [e.__dict__ for e in self.entities]
        boundary_entities = [e.__dict__ for e in self.boundaries]
        return actual_entities + boundary_entities

    def update_entities_from_cpp(self, entities):
        """
        Update the environment from C++. The changes might be due to
        the environment dynamics or the agent's actions.
        Entities is a list of python dicts.
        """
        self.entity_nums = {t : 0 for t in self.grid_types}
        self.entities = [Entity(**i) for i in entities if not self.__is_boundary(i["id"])]
        for e in self.entities:
            self.entity_nums[e.type] += 1
        # update available grids
        self.available_grids = set(itertools.product(range(self.width), range(self.height), (0,)))
        occupied = set([e.loc for e in self.entities])
        self.available_grids -= occupied
        self.available_grids = list(self.available_grids)
        random.shuffle(self.available_grids)

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

    def update_game_event_from_cpp(self, event):
        """
        Update the game event from CPP simulator
        """
        self.game_event = event

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
        Y, X = self.get_dims()

        maze = spanning_tree_maze_generator(X, Y)
        blocks = [(j, i, 0) for i,m in enumerate(maze) for j,b in enumerate(m) if b == '#']

        ## maybe not all blocks of the maze will be used later
        random.shuffle(blocks)

        ## first remove all maze blocks from the available set
        for b in blocks:
            if b in self.available_grids:
                self.available_grids.remove(b)

        ## select a random object path for each non-block entity
        for i, e in enumerate(self.entities):
            if e.name is None:
                e.name = random.choice(self.get_all_possible_names(e.type))
            e.id = "%s_%d" % (e.name, i)
            if e.asset_path is None:
                icons = self.items[e.type][e.name]
                e.asset_path = random.choice(icons)
            e.color = self.color_table[e.asset_path]
            if e.loc is None and e.type != "block":
                assert len(self.available_grids) > 0
                e.loc = self.available_grids.pop()
            if e.type == "agent":
                e.yaw = random.uniform(-self.PI, self.PI)
                self.init_agent_loc = e.loc
            elif e.type == "goal":
                e.yaw = random.randint(-1,2) * self.PI_2

        ## add back some empty grids
        self.available_grids += blocks[len(self.get_blocks()):]

        ## use the remaining blocks
        for e in self.get_blocks():
            assert blocks, "too many blocks for a valid maze"
            e.loc = blocks.pop()

    def __add_boundaries(self):
        """
        Given the max height and width of a map, return a list of surrounding
        wall blocks.
        """
        wall_blocks = []
        wall_height = 3
        def add_blocks(range1, range2, id):
            for loc in itertools.product(range1, range2):
                for k in range(wall_height):
                    wall_blocks.append(Entity(type="boundary", loc=loc+(k,), id="wall_%d" % id,
                                              name="wall", color="na",
                                              asset_path=random.choice(self.items["boundary"]["wall"])))
                    id += 1
            return id
        id = add_blocks(range(-1, self.width+1), (-1, self.height),
                        self.height * self.width)
        id = add_blocks((-1, self.width), range(0, self.height), id);
        return wall_blocks

    def __is_boundary(self, id):
        """
        Given a location, determine whether it's a padding block or not
        """
        return (id in [b.id for b in self.boundaries])

    def __clean_env(self):
        """
        Reset members; preparing for the next session
        """
        self.agent_sent = ""
        self.changed = False
        self.entities = []
        self.boundaries = []
        self.entity_nums = {t : 0 for t in self.grid_types}
        self.available_grids = []
