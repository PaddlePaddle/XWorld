#!/usr/bin/python

from py_simulator import Simulator
from random import randint
import os

if __name__ == "__main__":
    options = {
        "runfiles_path": os.environ["DEEPMIND_RUNFILES"],
        "level_script": "./test_map.lua",
        "context": 1
    }
    dm = Simulator.create("deepmind_lab", options)

    num_actions = dm.get_num_actions()
    act_rep = options["context"]

    reward = 0
    for i in range(100):
        game_over_str = dm.game_over()
        if game_over_str != "alive" and game_over_str != "lost_life":
            print "game over because of ", game_over_str
            dm.reset_game()
            continue

        dm.show_screen()

        states = dm.get_state()
        action = randint(0, num_actions - 1)
        r = dm.take_actions({"action": action}, act_rep)
        print r
        reward += r

    print "total reward ", reward
