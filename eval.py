#!/usr/bin/env python3
"""Visual eval — uses env, renders every frame."""

import sys, os; sys.path.insert(0, os.path.dirname(__file__))
import mujoco, glfw, time, numpy as np
from stable_baselines3 import SAC
from envs.ur5_env import UR5GraspEnv, TARGET

model = SAC.load(sys.argv[1] if len(sys.argv) > 1 else "ur5_ik_sac")
env = UR5GraspEnv()

if not glfw.init(): sys.exit(1)
w = glfw.create_window(1200, 800, "UR5 Grasp", None, None)
if not w: glfw.terminate(); sys.exit(1)
glfw.make_context_current(w)

vcam = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(vcam)
vcam.azimuth=135; vcam.elevation=-25; vcam.distance=2.0; vcam.lookat[:]=[0.6,0,0.15]
scene = mujoco.MjvScene(env.model, 2000)
opt = mujoco.MjvOption(); mujoco.mjv_defaultOption(opt)
pert = mujoco.MjvPerturb()
ctx = mujoco.MjrContext(env.model, mujoco.mjtFontScale.mjFONTSCALE_150)

def rf():
    vp = mujoco.MjrRect(0,0,*glfw.get_framebuffer_size(w))
    mujoco.mjv_updateScene(env.model,env.data,opt,pert,vcam,mujoco.mjtCatBit.mjCAT_ALL,scene)
    mujoco.mjr_render(vp,scene,ctx); glfw.swap_buffers(w); glfw.poll_events()
    if glfw.get_key(w, glfw.KEY_LEFT)==glfw.PRESS: vcam.azimuth-=3
    if glfw.get_key(w, glfw.KEY_RIGHT)==glfw.PRESS: vcam.azimuth+=3
    if glfw.get_key(w, glfw.KEY_UP)==glfw.PRESS: vcam.elevation=min(89,vcam.elevation+3)
    if glfw.get_key(w, glfw.KEY_DOWN)==glfw.PRESS: vcam.elevation=max(-89,vcam.elevation-3)
glfw.set_scroll_callback(w, lambda wv,x,y: setattr(vcam,'distance',max(0.3,min(10,vcam.distance*(1-y*0.08)))))

succ = 0
for ep in range(30):
    if glfw.window_should_close(w): break
    print(f"[{ep+1}/30]", flush=True)
    obs, _ = env.reset()  # env handles everything: drop cube, IK, animate arm
    rf(); time.sleep(0.3)

    for s in range(30):
        action, _ = model.predict(obs, deterministic=True)
        obs, r, t, tr, info = env.step(action)
        rf(); time.sleep(0.03)
        if t or tr: break

    ok = info['success']
    if ok: succ += 1
    print(f" {'✓' if ok else '✗'} z={info['cube_z']:.3f}\n")
    for _ in range(30): rf(); time.sleep(0.02)

glfw.terminate(); env.close()
print(f"\n{succ}/30 successful")
