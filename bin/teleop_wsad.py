#!/usr/bin/env python
from __future__ import annotations

import sys
import termios
import tty

from cortex.cortical_agent import CorticalAgent
from envs.base_env import BaseEnv


KEY_ACTIONS = {
    'w': [1.0, 0.0],
    's': [-1.0, 0.0],
    'a': [0.0, -1.0],
    'd': [0.0, 1.0],
}


def get_key() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def main() -> None:
    env = BaseEnv()
    agent = CorticalAgent(env.observation_space, env.action_space)
    obs, _ = env.reset()
    done = False
    while not done:
        print("w/s/a/d to move, q to quit")
        k = get_key()
        if k == 'q':
            break
        action = KEY_ACTIONS.get(k, [0.0, 0.0])
        obs, _, done, _, _ = env.step(action)
        print("state", obs)


if __name__ == "__main__":
    main()
