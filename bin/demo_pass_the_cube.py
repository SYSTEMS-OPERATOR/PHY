#!/usr/bin/env python
from tasks.cooperative_tasks import PassTheCubeEnv


def main() -> None:
    env = PassTheCubeEnv()
    env.reset()
    obs, reward, done, _, _ = env.step({"agent_0": [0.5], "agent_1": [0.5]})
    print("done", done, "reward", reward)


if __name__ == "__main__":
    main()
