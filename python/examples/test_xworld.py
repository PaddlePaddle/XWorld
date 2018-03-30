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


def compute_speak_action(states, num_actions, keep_silent = False):
    """
    A bad speak action computation method
    """
    screen = states["screen"]
    sentence = states["sentence"]
    if keep_silent:
        action = "what"
    else:
        action = sentence # repeat teacher's sentence
    return action


if __name__ == "__main__":

    print "Example 1: Navigation with language instruction (with curriculum)"
    options = {
        "xwd_conf_path": "../../confs/walls.json",
        "curriculum": 0.1,
        "task_mode": "arxiv_lang_acquisition",
        "context": 1,
        "pause_screen": True,
        "task_groups_exclusive": False,
        "visible_radius" : 0
    }
    xworld = Simulator.create("xworld", options)
    xworld.reset_game()

    num_actions = xworld.get_num_actions()

    reward = 0
    for i in range(100):
        game_over_str = xworld.game_over()
        if game_over_str != "alive":
            print "game over because of ", game_over_str
            xworld.reset_game()
            continue

        states = xworld.get_state()
        action = compute_action(states, num_actions)
        r = xworld.take_actions({"action": action, "pred_sentence" : ""}, 1, False)
        print r
        reward += r

    print "total reward ", reward

    print "Example 2: Interactive language learning"
    options = {
        "xwd_conf_path": "../../confs/lang.json",
        "task_mode": "arxiv_interactive",
        "context": 1,
        "pause_screen": True,
        "task_groups_exclusive": True
    }
    xworld = Simulator.create("xworld", options)
    xworld.reset_game()

    num_actions = xworld.get_num_actions()

    reward = 0
    for i in range(100):
        game_over_str = xworld.game_over()
        states = xworld.get_state()
        action = compute_speak_action(states, num_actions)
        r = xworld.take_actions({"pred_sentence": action}, 1, False)

        if game_over_str != "alive":
            print "game over because of ", game_over_str
            xworld.reset_game()
            continue

        print r
        reward += r

    print "total reward ", reward


    print "Example 3: Dialogue-based language and concept learning"
    options = {
        "xwd_conf_path": "../../confs/dialog.json",
        "task_mode": "arxiv_interactive",
        "context": 1,
        "pause_screen": True,
        "task_groups_exclusive": True
    }
    xworld = Simulator.create("xworld", options)
    xworld.reset_game()

    num_actions = xworld.get_num_actions()

    reward = 0
    for i in range(100):
        game_over_str = xworld.game_over()
        states = xworld.get_state()
        action = compute_speak_action(states, num_actions, True)
        r = xworld.take_actions({"pred_sentence": action}, 1, False)

        if game_over_str != "alive":
            print "game over because of ", game_over_str
            xworld.reset_game()
            continue

        print r
        reward += r

    print "total reward ", reward
