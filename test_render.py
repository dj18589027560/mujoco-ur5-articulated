#!/usr/bin/env python3
"""Minimal MuJoCo render test — just a plane + box."""

import glfw, mujoco, time

# Minimal XML
xml = '''
<mujoco>
  <worldbody>
    <geom type="plane" size="2 2 0.1" rgba="1 1 1 1"/>
    <body pos="0 0 0.3">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" rgba="0.9 0.2 0.2 1"/>
    </body>
    <light pos="0 0 3"/>
  </worldbody>
</mujoco>'''

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

if not glfw.init():
    exit(1)
w = glfw.create_window(800, 600, "Test Render", None, None)
if not w:
    glfw.terminate()
    exit(1)
glfw.make_context_current(w)

cam = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(cam)
cam.azimuth = 135; cam.elevation = -20; cam.distance = 3
scene = mujoco.MjvScene(model, 100)
opt = mujoco.MjvOption(); mujoco.mjv_defaultOption(opt)
pert = mujoco.MjvPerturb()
ctx = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_100)

while not glfw.window_should_close(w):
    glfw.poll_events()
    mujoco.mj_step(model, data)
    viewport = mujoco.MjrRect(0, 0, 800, 600)
    mujoco.mjv_updateScene(model, data, opt, pert, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)
    mujoco.mjr_render(viewport, scene, ctx)
    glfw.swap_buffers(w)
    time.sleep(0.01)

glfw.terminate()
print("Window closed — rendering works!")
