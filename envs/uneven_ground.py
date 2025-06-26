from .base_env import BaseEnv
import numpy as np


class UnevenGroundEnv(BaseEnv):
    def step(self, action):
        self.state += 0.1 * np.array(action) + np.random.uniform(-0.05, 0.05, size=self.state.shape)
        self.t += 1
        reward = float(-np.sum(self.state ** 2))
        done = self.t >= 100
        return self.state, reward, done, False, {}
