#!/usr/bin/env python3
"""Headless evaluation — fast summary, no window."""

import sys, os
sys.path.insert(0, os.path.dirname(__file__))

import numpy as np
from stable_baselines3 import SAC
from envs.ur5_env import UR5GraspEnv

model_path = sys.argv[1] if len(sys.argv) > 1 else "ur5_grasp_sac"
model = SAC.load(model_path)
env = UR5GraspEnv()

N = 50
successes = [] ; rewards = []

for i in range(N):
    obs, _ = env.reset()
    ep_r = 0
    for _ in range(200):
        action, _ = model.predict(obs, deterministic=True)
        obs, r, terminated, truncated, info = env.step(action)
        ep_r += r
        if terminated or truncated:
            break
    successes.append(info['success'])
    rewards.append(ep_r)

env.close()

print(f"\n--- {N} episodes (headless) ---")
print(f"Success rate: {sum(successes)}/{N} = {100*sum(successes)/N:.0f}%")
print(f"Mean reward:  {np.mean(rewards):.1f} ± {np.std(rewards):.1f}")
print(f"Mean cube_z:  {info.get('cube_z', 0):.3f} (last episode)")
