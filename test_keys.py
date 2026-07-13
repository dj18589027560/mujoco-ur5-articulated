#!/usr/bin/env python3
import glfw

if not glfw.init():
    exit(1)

window = glfw.create_window(400, 200, "Test Keys", None, None)
if not window:
    glfw.terminate()
    exit(1)

glfw.make_context_current(window)
print("Press keys in the window. Press ESC to exit.")

def key_cb(w, key, scancode, action, mods):
    if action == glfw.PRESS:
        print(f"PRESS: key={key} scancode={scancode} name={getattr(glfw, 'KEY_'+chr(key) if 32<=key<127 else '', 'N/A')}")
        if key == glfw.KEY_ESCAPE:
            glfw.set_window_should_close(window, True)

glfw.set_key_callback(window, key_cb)

while not glfw.window_should_close(window):
    glfw.poll_events()

glfw.terminate()
