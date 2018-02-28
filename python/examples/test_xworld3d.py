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

    print "Example: Navigation in 3D"
    options = {
        "x3_conf": "../../confs/walls3d.json",
        "context": 1,
        "pause_screen": True,
        "x3_training_img_width" : 64,
        "x3_training_img_height" : 64,
        "x3_big_screen" : True,
    }
    x3d = Simulator.create("xworld3d", options)
    x3d.reset_game()

    num_actions = x3d.get_num_actions()

    reward = 0
    print("\033[93mUse show_screen() to see the game. " \
          + "However, X server must be used; otherwise the code crashes\033[0m")
    for i in range(100):
#        x3d.show_screen()

        game_over_str = x3d.game_over()
        if game_over_str != "alive":
            print "game over because of ", game_over_str
            x3d.reset_game()
            continue

        states = x3d.get_state()
        action = compute_action(states, num_actions)
        r = x3d.take_actions({"action": action, "pred_sentence" : ""})
        print r
        reward += r

    print "total reward ", reward
