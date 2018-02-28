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
    print("\033[93mUse show_screen() to see the game. " \
          + "This game does not need X server.\033[0m")
    for i in range(100):
        game_over_str = simple.game_over()
        if game_over_str != "alive":
            print "game over because of ", game_over_str
            simple.reset_game()
            continue

#        simple.show_screen()

        states = simple.get_state()
        action = randint(0, num_actions - 1)
        r = simple.take_actions({"action": action}, 1)
        print r
        reward += r

    print "total reward ", reward
