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

    print "Example: Language Learning in 3D"
    options = {
        "x3_conf": "../../confs/dialog3d.json",
        "context": 1,
        "pause_screen": True,
        "x3_training_img_width" : 64,
        "x3_training_img_height" : 64,
        "x3_big_screen" : True,
        "x3_task_mode" : "arxiv_interactive"
    }
    x3d = Simulator.create("xworld3d", options)
    x3d.reset_game()

    num_actions = x3d.get_num_actions()

    reward = 0
    for i in range(1000):
        game_over_str = x3d.game_over()
        states = x3d.get_state()
        action = compute_speak_action(states, num_actions, True)
        r = x3d.take_actions({"pred_sentence" : action}, 1, True)

        if game_over_str != "alive":
            print "game over because of ", game_over_str
            x3d.reset_game()
            continue

        print r
        reward += r

    print "total reward ", reward
