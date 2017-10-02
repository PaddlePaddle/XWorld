#!/usr/bin/python

from py_simulator import Simulator
import sys
sys.path.append("../../games/xworld/tasks")


def compute_action(states, num_actions):
    """
    A bad action computation method
    """
    screen = states["screen"]
    sentence = states["sentence"]
    action = (abs(hash(sentence)) + int(sum(screen))) % num_actions
    return action


def compute_speak_action(states, num_actions):
    """
    A bad speak action computation method
    """
    screen = states["screen"]
    sentence = states["sentence"]
    action = sentence # repeat teacher's sentence
    return action


if __name__ == "__main__":

    print "Example 1: Navigation with language instruction"
    options = {
        "conf_path": "../../confs/empty_ground.json",
        "curriculum": 0,
        "task_mode": "arxiv_lang_acquisition",
        "context": 1,
        "pause_screen": True,
        "task_groups_exclusive": True
    }
    xworld = Simulator.create("xworld", options)

    num_actions = xworld.get_num_actions()

    reward = 0
    for i in range(100):
        game_over_str = xworld.game_over()
        if game_over_str != "alive":
            print "game over because of ", game_over_str
            xworld.reset_game()
            continue

        xworld.show_screen()

        states = xworld.get_state()
        action = compute_action(states, num_actions)
        r = xworld.take_actions({"action": action})
        print r
        reward += r

    print "total reward ", reward

    print "Example 2: Interactive language learning"
    options = {
        "conf_path": "../../confs/lang.json",
        "curriculum": 0,
        "task_mode": "arxiv_interactive",
        "context": 1,
        "pause_screen": True,
        "task_groups_exclusive": True
    }
    xworld = Simulator.create("xworld", options)

    num_actions = xworld.get_num_actions()

    reward = 0
    for i in range(100):
        game_over_str = xworld.game_over()
        if game_over_str != "alive":
            print "game over because of ", game_over_str
            xworld.reset_game()
            continue

        xworld.show_screen()

        states = xworld.get_state()
        action = compute_speak_action(states, num_actions)
        r = xworld.take_actions({"pred_sentence": action})
        print r
        reward += r

    print "total reward ", reward
