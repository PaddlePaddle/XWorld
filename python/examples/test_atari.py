#!/usr/bin/python

from py_simulator import Simulator
from random import randint

if __name__ == "__main__":
    options = {
        "ale_rom": "/tmp/breakout.bin",
        "pause_screen": True,
        "context": 4
    }
    atari = Simulator.create("atari", options)
    atari.reset_game()

    num_actions = atari.get_num_actions()
    act_rep = options["context"]

    reward = 0
    for i in range(100):
        game_over_str = atari.game_over()
        states = atari.get_state()
        action = randint(0, num_actions - 1)
        r = atari.take_actions({"action": action}, act_rep, False)

        if game_over_str != "alive" and game_over_str != "lost_life":
            print "game over because of ", game_over_str
            atari.reset_game()
            continue

        print r
        reward += r

    print "total reward ", reward
