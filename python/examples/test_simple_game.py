#!/usr/bin/python

from py_simulator import Simulator
from random import randint

if __name__ == "__main__":
    options = {
        "array_size": 7
    }
    simple = Simulator.create("simple_game", options)
    simple.reset_game()

    num_actions = simple.get_num_actions()

    reward = 0
    for i in range(100):
        game_over_str = simple.game_over()
        states = simple.get_state()
        action = randint(0, num_actions - 1)
        r = simple.take_actions({"action": action}, 1, False)

        if game_over_str != "alive":
            print "game over because of ", game_over_str
            simple.reset_game()
            continue

        print r
        reward += r

    print "total reward ", reward
