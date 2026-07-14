#!/usr/bin/env python3
"""Native collision detection + collision-free IK + RL."""

import gymnasium as gym
import numpy as np
import mujoco
import os, math, random

MODEL_PATH = os.path.join(os.path.dirname(__file__), "..", "ur5_2f85.xml")
TARGET = np.array([0.55, 0.0, 0.14], dtype=np.float32)
PEDESTAL = np.array([0.55, 0.0, 0.0])

GRIPPER_BODIES = ['ee_link','grp_base','grp_right_driver','grp_right_coupler',
    'grp_right_follower','grp_right_pad','grp_right_spring_link',
    'grp_left_driver','grp_left_coupler','grp_left_follower',
    'grp_left_pad','grp_left_spring_link','wrist_3_link','wrist_2_link']

ARM_BODIES = {'shoulder_link','elbow_link','wrist_1_link','wrist_2_link',
              'wrist_3_link','ee_link','base_link','base'}


def ik_pinch(model, data, target, init_qpos=None, max_iter=150, tol=0.003):
    if init_qpos is not None: data.qpos[10:16] = init_qpos
    for _ in range(max_iter):
        mujoco.mj_forward(model, data)
        p = data.site("grp_pinch").xpos.copy()
        e = target - p
        if np.linalg.norm(e) < tol: return True
        j = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, j, None, data.site("grp_pinch").id)
        j = j[:, 9:15]
        data.qpos[10:16] = np.clip(
            data.qpos[10:16] + np.clip(j.T @ np.linalg.solve(j@j.T + 0.01*np.eye(3), e), -0.1, 0.1),
            -math.pi, math.pi)
    return False


def in_pedestal(pos):
    return np.linalg.norm(pos[:2] - PEDESTAL[:2]) < 0.04 and pos[2] < 0.12


def count_penetrations(model, data):
    return sum(1 for b in GRIPPER_BODIES if in_pedestal(data.body(b).xpos))


def count_arm_contacts(model, data):
    """Use native collision detection for arm self-collision + base contact."""
    mujoco.mj_collision(model, data)
    n = 0
    for i in range(data.ncon):
        c = data.contact[i]
        b1 = model.body(model.geom_bodyid[c.geom1]).name
        b2 = model.body(model.geom_bodyid[c.geom2]).name
        if b1 in ARM_BODIES and b2 in ARM_BODIES and b1 != b2:
            n += 1
    return n


def ik_collision_free(model, data, target, max_tries=40):
    best, best_pen = None, 999; save = data.qpos.copy()
    for _ in range(max_tries):
        data.qpos[:] = save
        data.qpos[10:16] = np.array([random.uniform(-np.pi, np.pi) for _ in range(6)])
        if ik_pinch(model, data, target):
            p_ped = count_penetrations(model, data)
            p_arm = count_arm_contacts(model, data)
            pen = p_ped*3 + p_arm*2  # weight pedestal higher
            if pen == 0: return data.qpos[10:16].copy()
            if pen < best_pen: best_pen = pen; best = data.qpos[10:16].copy()
    if best is not None: data.qpos[10:16] = best; return best
    data.qpos[10:16] = np.zeros(6); ik_pinch(model, data, target)
    return data.qpos[10:16].copy()


class UR5GraspEnv(gym.Env):
    def __init__(self, render_mode=None, max_steps=40):
        super().__init__()
        self.model = mujoco.MjModel.from_xml_path(MODEL_PATH)
        self.data = mujoco.MjData(self.model)
        self.max_steps = max_steps
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (15,), np.float32)
        self.action_space = gym.spaces.Box(
            np.array([-0.08]*6 + [-0.05], np.float32),
            np.array([0.08]*6 + [0.05], np.float32), dtype=np.float32)
        self._step_count = 0; self._ik_pose = None

    def _get_obs(self):
        pinch = self.data.site("grp_pinch").xpos.copy()
        jpos = self.data.qpos[10:16].copy()
        ped = float(count_penetrations(self.model, self.data) > 0)
        arm = float(count_arm_contacts(self.model, self.data) > 0)
        return np.concatenate([pinch, TARGET, [self.data.qpos[2]], jpos, [ped], [arm]],
                              dtype=np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.data.qpos[:]=0; self.data.qvel[:]=0; self.data.ctrl[:]=0
        self.data.qpos[0:7] = [0.55, 0.0, 0.18, 1, 0, 0, 0]
        mujoco.mj_forward(self.model, self.data)
        for _ in range(100): self.data.qpos[7:10]=0; mujoco.mj_step(self.model, self.data)
        self._ik_pose = ik_collision_free(self.model, self.data, TARGET)
        self.data.ctrl[9] = 0; self._step_count = 0
        return self._get_obs(), {}

    def step(self, action):
        self.data.qpos[7:10] = 0
        self.data.qpos[10:16] = np.clip(self._ik_pose + action[:6], -math.pi, math.pi)
        self.data.ctrl[3:9] = self.data.qpos[10:16].copy()
        self.data.ctrl[9] = np.clip(self.data.ctrl[9] + action[6], 0, 0.8)
        mujoco.mj_step(self.model, self.data)
        self._step_count += 1
        d = np.linalg.norm(self.data.site("grp_pinch").xpos - TARGET)
        z = self.data.qpos[2]
        n_ped = count_penetrations(self.model, self.data)
        n_arm = count_arm_contacts(self.model, self.data)
        r = -d*0.5 - n_ped*8.0 - n_arm*8.0 + (self.data.ctrl[9]/0.8)*2.0
        if z > 0.16: r += 5.0
        if z > 0.22: r += 30.0
        return self._get_obs(), r, z > 0.22, self._step_count >= self.max_steps, {
            "dist": d, "cube_z": z, "success": z > 0.22,
            "ped": n_ped, "arm_col": n_arm}

    def render(self): return None
    def close(self): super().close()


if __name__ == "__main__":
    env = UR5GraspEnv()
    obs, _ = env.reset()
    print(f"ped={obs[13]:.0f} arm={obs[14]:.0f} z={obs[6]:.3f}")
    for _ in range(20):
        obs, r, t, tr, info = env.step(np.array([0]*6+[0.05], dtype=np.float32))
    print(f"After close: ped={info['ped']} arm={info['arm_col']} z={info['cube_z']:.3f}")
    env.close()
