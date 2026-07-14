#!/usr/bin/env python3
"""SAC on IK-assisted reach: position fixed, RL closes gripper."""

import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from envs.ur5_env import UR5GraspEnv
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.monitor import Monitor
import torch

device = "mps" if torch.backends.mps.is_available() else "cpu"
print(f"Device: {device}")

env = Monitor(UR5GraspEnv(max_steps=30))
eval_env = Monitor(UR5GraspEnv(max_steps=30))

model = SAC("MlpPolicy", env,
    learning_rate=3e-4, buffer_size=5000, batch_size=64,
    tau=0.005, gamma=0.95, device=device, verbose=1)

model.learn(total_timesteps=10000, callback=EvalCallback(
    eval_env, best_model_save_path="./logs/best_model",
    log_path="./logs/", eval_freq=2000, n_eval_episodes=10, deterministic=True))
model.save("ur5_ik_sac")
print("\nSaved ur5_ik_sac.zip")
env.close(); eval_env.close()
