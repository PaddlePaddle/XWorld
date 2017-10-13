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
  asset_path - the icon image path
  name       - name of the entity
  color      - color of the entity
"""
class Entity:
    def __init__(self, type, id=None, loc=None, name=None, asset_path=None, color=None):
        if not loc is None:
            assert isinstance(loc, tuple) and len(loc) == 2
        self.type = type
        self.id = id
        self.loc = loc
        self.name = name
        self.asset_path = asset_path
        self.color = color

class XWorldEnv(object):
    def __init__(self, item_path, max_height=7, max_width=7):
        self.num_games = -1
        ## load all items from item_path
        self.grid_types = ["goal", "block", "agent"]
        self.item_path = item_path
        self.max_height = max_height
        self.max_width = max_width
        self.all_icon_paths = []
        for dirpath, dirs, files in os.walk(item_path):
            for f in files:
                if f.endswith(".jpg") or f.endswith(".png"):
                    self.all_icon_paths.append(os.path.join(dirpath, f))
        self.set_goal_subtrees([])
        ## init dimensions
        self.__clean_env()
        self.set_dims(max_height, max_width)
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
        return self.num_games

    def draw_map(self):
        map_ = [["." for j in range(self.max_width)] for i in range(self.max_height)]
        for e in self.pad_blocks:
            l = e.loc
            map_[l[1]][l[0]] = 'b'
        for e in self.entities:
            l = e.loc
            map_[l[1] + self.offset_h][l[0] + self.offset_w] = e.type[0]
        pprint.pprint(map_)

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
        ## if the actual size is smaller than the max, then we pad walls
        self.offset_h = (self.max_height - h) / 2
        self.offset_w = (self.max_width - w) / 2
        self.pad_blocks = self.__padding_walls()
        self.available_grids = list(set(itertools.product(range(w), range(h))) & set(self.available_grids))
        random.shuffle(self.available_grids)
        self.changed = True

    def set_entity(self, type, loc=None, name=None):
        """
        Add an entity of type to loc which must be empty
        """
        self.entity_nums[type] += 1
        self.entities.append(Entity(type=type, loc=loc, name=name))
        if not loc is None:
            assert loc in self.available_grids, \
                "set_dims correctly before setting a location"
            self.available_grids.remove(loc)
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

    def get_dims(self):
        """
        Get the height and width of the map
        We return the max height and width because C++ will render the padding walls
        """
        return (self.max_height, self.max_width)

    def get_n(self, type):
        """
        Get the current entity number on the map for type
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
        agent = [e for e in self.entities if e.type == "agent"][0]
        return (agent, self.agent_sent, self.action_successful)

    def get_goals(self):
        return [e for e in self.entities if e.type == "goal"]

    def get_blocks(self):
        return [e for e in self.entities if e.type == "block"]

    def get_entities(self):
        return self.entities

    ######################## interface with C++ #############################
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
        Update the environment from C++. The chagnes might be due to
        the environment dynamics or the agent's actions.
        Entities is a list of python dicts.
        """
        self.entity_nums = {t : 0 for t in self.grid_types}
        for e in entities:
            e["loc"] = e["loc"][:2]  ## remove the third dimension 'z'
        self.entities = [Entity(**i) for i in entities if not self.__is_padding_block(i["loc"])]
        for e in self.entities:
            e.loc = (e.loc[0] - self.offset_w, e.loc[1] - self.offset_h)
            self.entity_nums[e.type] += 1

    def update_agent_sentence_from_cpp(self, sent):
        self.agent_sent = sent

    def update_agent_action_success_from_cpp(self, successful):
        self.action_successful = successful

    ######################## private or protected #########################
    def _configure(self):
        """
        The user has to override this function
        """
        raise NotImplementedError()

    def __instantiate_entities(self):
        """
        For each entity, select an instance from the object class it belongs to,
        after which its properties are set
        """
        ## select a random img path for each entity
        for i, e in enumerate(self.entities):
            if e.name is None:
                e.name = random.choice(self.get_all_possible_names(e.type))
            icons = self.items[e.type][e.name]
            e.asset_path = random.choice(icons)
            e.id = "%s_%d" % (e.name, i)
            e.color = self.color_table[e.asset_path]
            if e.loc is None:
                assert len(self.available_grids) > 0
                e.loc = self.available_grids[0]
                self.available_grids = self.available_grids[1:]

    def __padding_walls(self):
        """
        Given the max height and width of a map and (offset_w, offset_h),
        return a list of padding wall blocks
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

    def __clean_env(self):
        """
        Clean up some members
        """
        self.num_games += 1
        self.agent_sent = ""
        self.changed = False
        self.entities = []
        self.entity_nums = {t : 0 for t in self.grid_types}
        self.available_grids = list(itertools.product(range(self.max_width), range(self.max_height)))
