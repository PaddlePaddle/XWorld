#!/usr/bin/python

from py_simulator import Simulator
from random import randint
import os
import subprocess
from subprocess import PIPE
import time

client_port = 10000


def launch_client():
    try:
        malmo_root = os.environ["MALMO_ROOT"]
    except:
        print("Please set environment variable MALMO_ROOT")
        exit()

    p = subprocess.Popen(
        'cd ' + malmo_root + '/Minecraft; ./launchClient.sh -port %d' %
        client_port,
        shell=True,
        stdout=PIPE,
        stderr=PIPE)

    while True:
        for stdout_line in iter(p.stdout.readline, ""):
            if "Listening for messages on port" in stdout_line:
                return p
        time.sleep(3)


if __name__ == "__main__":

    client = launch_client()

    options = {
        "conf_path": "./demo_conf.xml",
        "mission": "demo",
        "client_ip": "127.0.0.1",
        "client_port": client_port,
        "ms_per_tick": 10,
        "context": 1,
        "pause_screen": False
    }
    minecraft = Simulator.create("minecraft", options)

    num_actions = minecraft.get_num_actions()

    reward = 0
    for i in range(100):
        game_over_str = minecraft.game_over()
        if game_over_str != "alive":
            print "game over because of ", game_over_str
            minecraft.reset_game()
            continue

        minecraft.show_screen()

        minecraft.get_state()
        action = randint(0, num_actions - 1)
        r = minecraft.take_actions({"action": action})
        print r
        reward += r

    print "total reward ", reward

    client.kill()
