#!/usr/bin/python

from py_simulator import Simulator


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
        "xwd_conf_path": "../../confs/walls.json",
        "curriculum": 0,
        "task_mode": "arxiv_lang_acquisition",
        "context": 1,
        "pause_screen": True,
        "task_groups_exclusive": False,
        "ego_centric" : True
    }
    xworld = Simulator.create("xworld", options)

    num_actions = xworld.get_num_actions()

    reward = 0
    print("\033[93mUse show_screen() to see the game. " \
          + "However, X server must be used; otherwise the code crashes\033[0m")
    for i in range(100):
#        xworld.show_screen()

        game_over_str = xworld.game_over()
        if game_over_str != "alive":
            print "game over because of ", game_over_str
            xworld.reset_game()
            continue

        states = xworld.get_state()
        action = compute_action(states, num_actions)
        r = xworld.take_actions({"action": action, "pred_sentence" : ""})
        print r
        reward += r

    print "total reward ", reward

    print "Example 2: Interactive language learning"
    options = {
        "xwd_conf_path": "../../confs/lang.json",
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
#        xworld.show_screen()

        game_over_str = xworld.game_over()
        print(game_over_str)
        if game_over_str != "alive":
            print "game over because of ", game_over_str
            xworld.reset_game()
            continue

        states = xworld.get_state()
        action = compute_speak_action(states, num_actions)
        r = xworld.take_actions({"pred_sentence": action})
        print r
        reward += r

    print "total reward ", reward
