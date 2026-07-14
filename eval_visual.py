#!/usr/bin/env python3
"""Visual eval — DQN grasp at cube (0.97, 0.01, -0.28)."""

import sys, os
sys.path.insert(0, os.path.dirname(__file__))
import mujoco, glfw, time, numpy as np, math
from stable_baselines3 import DQN

MODEL_PATH = "ur5_2f85.xml"; H, W, GRID = 84, 84, 10
CUBE = [0.97, 0.01, -0.28]

model = mujoco.MjModel.from_xml_path(MODEL_PATH); data = mujoco.MjData(model)
policy = DQN.load(sys.argv[1] if len(sys.argv) > 1 else "ur5_dqn_visual")

if not glfw.init(): sys.exit(1)
win = glfw.create_window(1200, 800, "Visual Grasp", None, None)
if not win: glfw.terminate(); sys.exit(1)
glfw.make_context_current(win)

vcam = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(vcam)
vcam.azimuth=135; vcam.elevation=-25; vcam.distance=2.0; vcam.lookat[:]=CUBE
scene = mujoco.MjvScene(model, 2000)
opt = mujoco.MjvOption(); mujoco.mjv_defaultOption(opt)
pert = mujoco.MjvPerturb()
ctx = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
rgb_rend = mujoco.Renderer(model, H, W)

cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "top_down")
cam_obj = model.cam(cam_id); fov = float(cam_obj.fovy)
cam_pos = np.array(cam_obj.pos0); cam_mat = np.array(cam_obj.mat0).reshape(3,3)
fl = H/(2*math.tan(math.radians(fov)/2))

def ik(model, data, tgt):
    for _ in range(80):
        mujoco.mj_forward(model, data); ee = data.body("ee_link").xpos.copy()
        if np.linalg.norm(tgt-ee) < 0.01: return
        jac = np.zeros((3, model.nv))
        mujoco.mj_jacBody(model, data, jac, None, data.body("ee_link").id)
        jac = jac[:,3:9]
        data.qpos[3:9] += np.clip(jac.T @ np.linalg.solve(jac@jac.T+0.01*np.eye(3), tgt-ee), -0.1, 0.1)

def p2w(px, py):
    dx, dy = (px-W/2)/fl, (py-H/2)/fl
    ray = cam_mat.T @ (np.array([dx,dy,1.0])/np.linalg.norm([dx,dy,1.0]))
    t = (CUBE[2] - cam_pos[2]) / ray[2]
    return cam_pos + ray*t

def rf():
    glfw.make_context_current(win)
    vp = mujoco.MjrRect(0,0,*glfw.get_framebuffer_size(win))
    mujoco.mjv_updateScene(model,data,opt,pert,vcam,mujoco.mjtCatBit.mjCAT_ALL,scene)
    mujoco.mjr_render(vp,scene,ctx); glfw.swap_buffers(win); glfw.poll_events()
    if glfw.get_key(win, glfw.KEY_LEFT)==glfw.PRESS: vcam.azimuth-=3
    if glfw.get_key(win, glfw.KEY_RIGHT)==glfw.PRESS: vcam.azimuth+=3
    if glfw.get_key(win, glfw.KEY_UP)==glfw.PRESS: vcam.elevation=min(89,vcam.elevation+3)
    if glfw.get_key(win, glfw.KEY_DOWN)==glfw.PRESS: vcam.elevation=max(-89,vcam.elevation-3)
glfw.set_scroll_callback(win, lambda w,x,y: setattr(vcam,'distance',max(0.3,min(10,vcam.distance*(1-y*0.08)))))

succ = 0
for ep in range(30):
    if glfw.window_should_close(win): break
    data.qpos[:]=0; data.qvel[:]=0; data.ctrl[:]=0
    model.body("target_marker").pos = CUBE
    mujoco.mj_forward(model, data)

    rgb_rend.update_scene(data, camera="top_down"); rgb = rgb_rend.render()
    glfw.make_context_current(win)
    obs = np.transpose(rgb, (2,0,1)).astype(np.float32)/255.0

    action, _ = policy.predict(obs, deterministic=True)
    row, col = action//GRID, action%GRID
    target = p2w((col+0.5)/GRID*W, (row+0.5)/GRID*H)
    print(f"[{ep+1}/30] Cell {action} → ({target[0]:.2f},{target[1]:.2f})", end=" ")

    for tgt in [target+[0,0,0.15], target, target]:
        ik(model, data, tgt)
        for i in range(40):
            if tgt is target: data.ctrl[9] = 255
            mujoco.mj_step(model, data)
            if i%4==0: rf(); time.sleep(0.02)
    ik(model, data, target+[0,0,0.2])
    for i in range(40):
        mujoco.mj_step(model, data)
        if i%4==0: rf(); time.sleep(0.02)

    ok = data.qpos[13] > 0.3 and data.body("ee_link").xpos[2] > target[2]+0.05
    if ok: succ += 1
    print(f"{'OK' if ok else '--'}")
    for _ in range(10): rf(); time.sleep(0.05)

glfw.terminate(); print(f"\n{succ}/30 success")
