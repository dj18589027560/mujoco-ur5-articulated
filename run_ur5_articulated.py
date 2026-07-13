#!/usr/bin/env python3
"""MuJoCo UR5 articulated — stable version (no gripper)."""

import mujoco, glfw, sys, os, threading, time, math

os.chdir(os.path.dirname(os.path.abspath(__file__)))

model = mujoco.MjModel.from_xml_path("ur5_articulated.xml")
data = mujoco.MjData(model)

# Actuators: [0..2]=base(vel)  [3..8]=UR5(pos)
B_X, B_Y, B_T = 0, 1, 2
J1, J2, J3, J4, J5, J6 = 3, 4, 5, 6, 7, 8
GL, GR = 9, 10

u_init = [-math.pi/2, -3.054, -0.0873, -math.pi, -math.pi/2, -math.pi]
data.qpos[3:9] = u_init
data.ctrl[J1:J6+1] = u_init
data.ctrl[GL] = data.ctrl[GR] = 0.0
data.ctrl[B_X:B_T+1] = 0.0
mujoco.mj_forward(model, data)

print(f"MuJoCo {mujoco.__version__}  nq={model.nq}")

if not glfw.init(): sys.exit(1)
w = glfw.create_window(1200, 800, "MuJoCo UR5", None, None)
if not w: glfw.terminate(); sys.exit(1)
glfw.make_context_current(w)

cam = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(cam)
cam.azimuth, cam.elevation, cam.distance = 135, -20, 3.0
cam.lookat[:] = [0.3, 0.0, 0.3]

scene = mujoco.MjvScene(model, 1000)
opt = mujoco.MjvOption(); mujoco.mjv_defaultOption(opt)
pert = mujoco.MjvPerturb()
ctx = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)

JSTEP = 0.15
GSTEP = 0.003
BSPD = 0.2

def key_cb(win, key, sc, act, mods):
    if act == glfw.RELEASE:
        data.ctrl[B_X:B_T+1] = 0.0; return
    if act != glfw.PRESS: return

    if   key == glfw.KEY_LEFT:    cam.azimuth -= 5
    elif key == glfw.KEY_RIGHT:   cam.azimuth += 5
    elif key == glfw.KEY_UP:      cam.elevation = min(90, cam.elevation+5)
    elif key == glfw.KEY_DOWN:    cam.elevation = max(-90, cam.elevation-5)
    elif key == glfw.KEY_ESCAPE:  glfw.set_window_should_close(w, True)
    elif key == glfw.KEY_G:       data.ctrl[B_X] = BSPD
    elif key == glfw.KEY_H:       data.ctrl[B_X] = -BSPD
    elif key == glfw.KEY_T:       data.ctrl[B_Y] = BSPD
    elif key == glfw.KEY_Y:       data.ctrl[B_Y] = -BSPD
    elif key == glfw.KEY_R:       data.ctrl[B_T] = BSPD
    elif key == glfw.KEY_F:       data.ctrl[B_T] = -BSPD
    elif key == glfw.KEY_1:       data.ctrl[J1] -= JSTEP
    elif key == glfw.KEY_2:       data.ctrl[J1] += JSTEP
    elif key == glfw.KEY_W:       data.ctrl[J2] -= JSTEP
    elif key == glfw.KEY_S:       data.ctrl[J2] += JSTEP
    elif key == glfw.KEY_5:       data.ctrl[J3] -= JSTEP
    elif key == glfw.KEY_6:       data.ctrl[J3] += JSTEP
    elif key == glfw.KEY_7:       data.ctrl[J4] -= JSTEP
    elif key == glfw.KEY_8:       data.ctrl[J4] += JSTEP
    elif key == glfw.KEY_9:       data.ctrl[J5] -= JSTEP
    elif key == glfw.KEY_0:       data.ctrl[J5] += JSTEP
    elif key == glfw.KEY_MINUS:   data.ctrl[J6] -= JSTEP
    elif key == glfw.KEY_EQUAL:   data.ctrl[J6] += JSTEP

    elif key == glfw.KEY_Z:       data.ctrl[GL] += GSTEP; data.ctrl[GR] -= GSTEP
    elif key == glfw.KEY_X:       data.ctrl[GL] -= GSTEP; data.ctrl[GR] += GSTEP

glfw.set_key_callback(w, key_cb)
glfw.set_scroll_callback(w, lambda wv,x,y: setattr(cam, 'distance', max(0.3, min(30, cam.distance*(1-y*0.08)))))

run = True
def phys():
    while run: mujoco.mj_step(model, data); time.sleep(0.002)
threading.Thread(target=phys, daemon=True).start()

print("G/H/T/Y | R/F | W/S 1/2 5/6 7/8 9/0 -/= | arrows | ESC")

while not glfw.window_should_close(w):
    glfw.poll_events()
    vp = mujoco.MjrRect(0, 0, *glfw.get_framebuffer_size(w))
    mujoco.mjv_updateScene(model, data, opt, pert, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)
    mujoco.mjr_render(vp, scene, ctx)
    glfw.swap_buffers(w)

run = False; glfw.terminate()
