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
import warnings
from itertools import groupby
import itertools
from collections import OrderedDict
from py_gflags import get_flag
from maze2d import spanning_tree_maze_generator
from py_util import check_or_get_value

"""
Entity:
  id         - unique str for this entity
  type       - "agent", "goal", "block"
  location   - (x, y)
  yaw        - in radian
  scale      - (0, 1.0]
  offset     - [0, 1-scale]
  asset_path - the icon image path
  name       - name of the entity
  color      - color of the entity
"""
class Entity:
    def __init__(self, type, id=None, loc=None, name=None,
                 asset_path=None, color=None, yaw=0.0, scale=1.0, offset=0.0):
        if not loc is None:
            assert isinstance(loc, tuple) and len(loc) == 2
        self.type = type
        self.id = id
        self.loc = loc
        self.yaw = yaw
        self.scale = scale
        self.offset = offset
        self.name = name
        self.asset_path = asset_path
        self.color = color

class XWorldEnv(object):
    PI_2 = 1.5707963

    curriculum_check_period = 100

    def __init__(self, item_path, max_height=7, max_width=7, maze_generation=True):
        """
        item_path: path to the item images
        max_height/max_width: maximum height of the world; if a smaller world size
        is specified using set_dims, the extra space will be padded with wall blocks
        maze_generattion: True: use maze generator for placing the blocks
                          False: randomly assign available locations to blocks
        """
        self.num_games = -1
        ## load all items from item_path
        self.grid_types = ["goal", "block", "agent"]
        self.item_path = item_path
        self.max_height = max_height
        self.max_width = max_width
        self.maze_generation = maze_generation
        self.current_usage = {}
        self.curriculum_check_counter = 0
        self.all_icon_paths = []
        for dirpath, _, files in os.walk(item_path):
            for f in files:
                if f.endswith(".jpg") or f.endswith(".png"):
                    self.all_icon_paths.append(os.path.join(dirpath, f))
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

    def get_current_usage(self):
        self.curriculum_check_counter += 1
        if self.curriculum_check_counter < XWorldEnv.curriculum_check_period \
           or not self.current_usage:
            return 0
        ## we take the average usage across all the tasks
        usage = sum([sum(l) / float(len(l)) for l in self.current_usage.values()]) \
                / len(self.current_usage)
        self.curriculum_check_counter = 0
        return usage

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
        assert h >= 1 and w >= 1
        assert h <= self.max_height and w <= self.max_width
        self.height = h
        self.width = w
        ## if the actual size is smaller than the max, then we pad walls
        self.offset_h = (self.max_height - h) / 2
        self.offset_w = (self.max_width - w) / 2
        self.pad_blocks = self.__padding_walls()
        existing_entities = [e.loc for e in self.entities]
        self.available_grids = list(set(self.__generate_all_grids(h, w))-set(existing_entities))
        self.changed = True

    def set_entity(self, type, loc=None, name=None, force_occupy=False):
        """
        Add an entity instance of type to loc which must be currently empty if
        force_occupy is False. If force_occupy is True, then omit the location
        based availability check.
        """
        if not loc is None:
            if not force_occupy:
                assert loc in self.available_grids, \
                    "invalid or unavailable location %s; available grids %s" % (loc, self.available_grids)
            if loc in self.available_grids:
                self.available_grids.remove(loc)
        self.entity_nums[type] += 1
        self.entities.append(Entity(type=type, loc=loc, name=name))
        self.changed = True

    def set_property(self, entity, property_value_dict={}):
        """
        Reinstantiate the specified properties of an existing entity.
        Properties and corresponding values are specified by the property_value_dict in the form
        of {property : value, ...}, e.g. {"name" : "apple", "loc" : (0, 0)}. value could be None
        (force reinstantiation) or a valid value for that property.
        1) If None value is provided for a specified property (e.g. {"name" : None}), entity
           property will be reinstantiated regardless of its original value.
        2) otherwise, the value will be assigned to that property of the entity.
        3) all unset entity properties will be instantiated.
        """
        default_dict = OrderedDict.fromkeys(["name", \
                                             "loc", \
                                             "asset_path", \
                                             "yaw", \
                                             "scale", \
                                             "offset"])
        pv_dict = default_dict.copy()
        pv_dict.update(property_value_dict)

        # pre-processing for name and asset_path due to their dependency
        path_value = property_value_dict.get("asset_path", "empty")
        name_value = property_value_dict.get("name", "empty")

        # if name is specified by user but asset_path is unspecified
        if name_value != "empty" and path_value == "empty":
            property_value_dict["asset_path"] = None

        # if asset_path is specified
        if path_value != "empty":
            # if name is not specified then derive name from asset_path
            if name_value == "empty":
                names = [n for n in self.items[entity.type] if path_value in set(self.items[entity.type][n])]
                assert len(names) == 1, "there should be only one name corresponding to the asset_path: %s" % (path_value)
                property_value_dict["name"] = names[0]
                pv_dict.update(property_value_dict)
            else:
                # if both name and asset_path are specified, check consistency
                assert path_value in self.items[entity.type][name_value], \
                    "specified name: %s and asset_path: %s mis-match" % (name_value, path_value)

        for property in pv_dict:
            assert property in entity.__dict__.keys() and property in default_dict.keys(), \
                "invalid property name: %s is provided" % property
            value = pv_dict[property]

            # skip unspecified and non-empty entity properties
            if property not in property_value_dict.keys() and entity.__dict__[property] is not None:
                continue

            if property == "loc":
                if entity.loc is not None:
                    self.available_grids.append(entity.loc)
                entity.loc = check_or_get_value(value, self.available_grids)
                self.available_grids.remove(entity.loc)
            if property == "name":
                entity.name = check_or_get_value(value, self.get_all_possible_names(entity.type))
                # update id once name is changed
                entity.id = "%s_%s" % (entity.name, self.entity_nums[entity.type])
            if property == "asset_path":
                entity.asset_path = check_or_get_value(value, self.items[entity.type][entity.name])
                # color is coupled with asset_path
                if entity.asset_path in self.color_table.keys():
                    entity.color = self.color_table[entity.asset_path]
                else:
                    entity.color = "na"
            if property == "yaw" and get_flag("visible_radius") and entity.type != "block":
                ## if partially observed, perturb the objects
                yaw_range = range(-1, 3)
                entity.yaw = check_or_get_value(value, yaw_range) * self.PI_2
            if property == "scale" and get_flag("visible_radius") and entity.type == "goal":
                scale_range = [0.5, 1]
                entity.scale = check_or_get_value(value, scale_range, is_continuous=True)
            if property == "offset" and get_flag("visible_radius") and entity.type == "goal":
                offset_range = [0, 1 - (entity.scale if hasattr(entity, 'scale') else 0.5)]
                entity.offset = check_or_get_value(value, offset_range, is_continuous=True)

        self.changed = True

    def set_entity_inst(self, e):
        if not e.loc is None:
            assert e.loc in self.available_grids
            self.available_grids.remove(e.loc)
        self.entity_nums[e.type] += 1
        self.entities.append(e)
        self.changed = True

    def delete_entity(self, x):
        """
        Delete an entity on the current map either by its location or id
        """
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
        self.icon_paths = copy.deepcopy(self.all_icon_paths)
        if len(subtrees) > 0:
            self.icon_paths \
                = [p for p in self.icon_paths \
                   if not p.startswith(goal_path) or p.split("/")[-2] in subtrees]
        ## get a hierarchy of all possible objects
        key = lambda p: '_'.join(p.split('_')[:-1])
        objects = groupby(sorted(self.icon_paths, key=key), key=key)

        self.items = {t : {} for t in self.grid_types}
        for k, g in objects:
            type = [t for t in k.split("/") if t in self.grid_types][0]
            assert type in self.items
            self.items[type][os.path.basename(k)] = list(g)

    def get_max_dims(self):
        """
        Get the max height and width of the map
        We return the max height and width because C++ will render the padding walls
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
        stage of the task. If yes, then py_task.cpp will notify the simulator to update
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
        for e in actual_entities:
            e["loc"] = (e["loc"][0] + self.offset_w, e["loc"][1] + self.offset_h, 0)
        pad_entities = [e.__dict__ for e in self.pad_blocks]
        for e in pad_entities:
            e["loc"] = (e["loc"][0], e["loc"][1], 0)
        ## pad with walls
        return actual_entities + pad_entities

    def update_entities_from_cpp(self, entities):
        """
        Update the environment from C++. The changes might be due to
        the environment dynamics or the agent's actions.
        Entities is a list of python dicts.
        We only keep the first two dimensions of a C++ entity.
        """
        self.entity_nums = {t : 0 for t in self.grid_types}
        for e in entities:
            e["loc"] = e["loc"][:2]  ## remove the third dimension 'z'
        self.entities = [Entity(**i) for i in entities if not self.__is_padding_block(i["loc"])]
        for e in self.entities:
            e.loc = (e.loc[0] - self.offset_w, e.loc[1] - self.offset_h)
            self.entity_nums[e.type] += 1
        # update available grids
        self.available_grids = set(itertools.product(range(self.width), range(self.height)))
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
        pass

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
        if self.maze_generation:
            Y, X = self.get_dims()
            maze = spanning_tree_maze_generator(X, Y)
            blocks = [(j, i) for i,m in enumerate(maze) for j,b in enumerate(m) if b == '#']

            ## maybe not all blocks of the maze will be used later
            random.shuffle(blocks)

            ## first remove all maze blocks from the available set
            for b in blocks:
                if b in self.available_grids:
                    self.available_grids.remove(b)
            ## instantiate properties for each entity
            for i, e in enumerate(self.entities):
                if e.loc is not None:
                    warnings.warn("Maze generation is on! Overwriting pre-specified location %s!" % (e.loc,))
                    e.loc = None # remove the pre-set location when maze_generation is on
                ## skip setting loc for block here and set it later
                if e.type != "block":
                    ## if non-block, randomize the yaw, scale, and offset
                    self.set_property(e, {"yaw": None, "scale": None, "offset": None})
            ## add back some empty grids
            self.available_grids += blocks[len(self.get_blocks()):]

            ## use the remaining blocks
            for e in self.get_blocks():
                assert blocks, "too many blocks for a valid maze"
                e.loc = blocks.pop()
                self.set_property(e) ## still need to set other properties
        else:
            ## instantiate properties for each entity
            for i, e in enumerate(self.entities):
                self.set_property(e)

    def __padding_walls(self):
        """
        Given the max height and width of a map and (offset_w, offset_h),
        return a list of padding wall blocks. The actual space for the agent is
        (offset_w, offset_h, offset_w + self.width, offset_h + self.height)
        """
        wall_blocks = []
        def add_blocks(range1, range2, id):
            for loc in itertools.product(range1, range2):
                wall_blocks.append(Entity(type="block", loc=loc, id="block_%d" % id,
                                          name="brick", color="na",
                                          asset_path=self.items["block"]["brick"][0]))
                id += 1
            return id
        id = add_blocks(range(0, self.offset_w), range(0, self.height + self.offset_h),
                        self.max_height * self.max_width)
        id = add_blocks(range(self.offset_w, self.max_width), range(0, self.offset_h), id);
        id = add_blocks(range(self.offset_w + self.width, self.max_width), range(self.offset_h, self.max_height), id)
        id = add_blocks(range(0, self.offset_w + self.width), range(self.offset_h + self.height, self.max_height), id)
        return wall_blocks

    def __is_padding_block(self, loc):
        """
        Given a location, determine whether it's a padding block or not
        """
        x, y = loc
        return not (x >= self.offset_w and x < self.offset_w + self.width and \
                    y >= self.offset_h and y < self.offset_h + self.height)

    def __generate_all_grids(self, height, width, shuffle=True):
        """
        Given height and width, generate all the grids as the outer product of [0, h) x [0, w)]
        Randomly shuffle all grids if shuffle is True
        Return list of all the grids
        """
        assert height >= 1 and width >= 1
        all_grids = list(itertools.product(range(width), range(height)))
        if shuffle:
            random.shuffle(all_grids)
        return all_grids

    def __clean_env(self):
        """
        Reset members; preparing for the next session
        """
        self.num_games += 1
        self.agent_sent = ""
        self.changed = False
        self.entities = []
        self.entity_nums = {t : 0 for t in self.grid_types}
        self.available_grids = []
        self.set_dims(self.max_height, self.max_width)
