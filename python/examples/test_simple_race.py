#!/usr/bin/python

from py_simulator import Simulator
from random import randint

if __name__ == "__main__":
    options = {
        "pause_screen": False,
        "window_width": 480,
        "window_height": 480,
        "track_type": "straight",
        "track_width": 20.0,
        "track_length": 100.0,
        "track_radius": 30.0,
        "race_full_manouver": False,
        "random": False,
        "difficulty": "easy",
        "context": 1
    }
    sr = Simulator.create("simple_race", options)
    sr.reset_game()

    num_actions = sr.get_num_actions()
    act_rep = options["context"]

    reward = 0
    print("\033[93mUse show_screen() to see the game. " \
          + "However, X server must be used; otherwise the code crashes\033[0m")
    for i in range(100):
        game_over_str = sr.game_over()
        if game_over_str != "alive":
            print "game over because of ", game_over_str
            sr.reset_game()
            continue

#        sr.show_screen()

        states = sr.get_state()
        action = randint(0, num_actions - 1)
        r = sr.take_actions({"action": action}, act_rep)
        print r
        reward += r

    print "total reward ", reward
