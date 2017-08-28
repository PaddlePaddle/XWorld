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


if __name__ == "__main__":
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
