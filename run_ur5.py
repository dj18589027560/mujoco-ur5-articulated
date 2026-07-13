#!/usr/bin/env python3
"""MuJoCo UR5 viewer using GLFW directly.

Controls:
  Left/Right   - rotate camera azimuth
  Up/Down      - tilt camera elevation  
  Scroll       - zoom in/out
  ESC/Close    - exit
"""

import mujoco
import glfw
import sys
import os
import threading
import time

os.chdir(os.path.dirname(os.path.abspath(__file__)))

model = mujoco.MjModel.from_xml_path("example.xml")
data = mujoco.MjData(model)
print(f"MuJoCo {mujoco.__version__}  nq={model.nq}  nu={model.nu}")

if not glfw.init():
    sys.exit(1)

window = glfw.create_window(1200, 800, "MuJoCo UR5", None, None)
if not window:
    glfw.terminate()
    sys.exit(1)
glfw.make_context_current(window)

# Camera
cam = mujoco.MjvCamera()
mujoco.mjv_defaultCamera(cam)
cam.azimuth = 135
cam.elevation = -20
cam.distance = 2.5
cam.lookat[:] = [0.3, 0.0, 0.3]

scene = mujoco.MjvScene(model, maxgeom=1000)
opt = mujoco.MjvOption()
mujoco.mjv_defaultOption(opt)
pert = mujoco.MjvPerturb()
ctx = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)

# Keyboard & scroll
def key_cb(w, key, scancode, action, mods):
    if action == glfw.PRESS or action == glfw.REPEAT:
        step = 5.0
        if key == glfw.KEY_LEFT:
            cam.azimuth -= step
        elif key == glfw.KEY_RIGHT:
            cam.azimuth += step
        elif key == glfw.KEY_UP:
            cam.elevation = min(90, cam.elevation + step)
        elif key == glfw.KEY_DOWN:
            cam.elevation = max(-90, cam.elevation - step)
        elif key == glfw.KEY_ESCAPE:
            glfw.set_window_should_close(window, True)

def scroll_cb(w, xoff, yoff):
    cam.distance *= (1.0 - yoff * 0.08)
    cam.distance = max(0.5, min(20, cam.distance))

glfw.set_key_callback(window, key_cb)
glfw.set_scroll_callback(window, scroll_cb)

# Physics thread
running = True
def physics_loop():
    while running:
        mujoco.mj_step(model, data)
        time.sleep(0.002)

t = threading.Thread(target=physics_loop, daemon=True)
t.start()

print("Viewer started. Arrow keys rotate, scroll to zoom. ESC to exit.")

while not glfw.window_should_close(window):
    glfw.poll_events()
    
    w2, h2 = glfw.get_framebuffer_size(window)
    viewport = mujoco.MjrRect(0, 0, w2, h2)
    
    mujoco.mjv_updateScene(model, data, opt, pert, cam,
                           mujoco.mjtCatBit.mjCAT_ALL, scene)
    mujoco.mjr_render(viewport, scene, ctx)
    
    glfw.swap_buffers(window)

running = False
glfw.terminate()
