"""窗口 GL 上下文（GLFW/EGL 之外的第三条路径）里的渲染探针。

用来回答：`mujoco.viewer` 开窗时那条 warning（OpenGL error 0x502
in or before mjr_makeContext）是不是真的让画面空掉了。
这里在 GLFW 窗口上下文里渲染一帧并数颜色，画面好坏靠数字判断。
"""

import os
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
xml = sys.argv[1] if len(sys.argv) > 1 else str(ROOT / "model/universal_robots_ur5e/scene.xml")

model = mujoco.MjModel.from_xml_path(str(xml))
data = mujoco.MjData(model)
for _ in range(500):
    mujoco.mj_step(model, data)

print("XDG_SESSION_TYPE =", os.environ.get("XDG_SESSION_TYPE"), "| MUJOCO_GL =", os.environ.get("MUJOCO_GL", "(default)"))
ctx = mujoco.GLContext(320, 240)
try:
    ctx.make_current()
    renderer = mujoco.Renderer(model, height=240, width=320)
    renderer.update_scene(data)
    pixels = renderer.render()
    renderer.close()
finally:
    ctx.free()

colors = len(np.unique(pixels.reshape(-1, 3), axis=0))
print(f"windowed render: {pixels.shape} colors={colors} std={pixels.std():.1f} max={pixels.max()}")
if pixels.std() < 1.0 or colors < 10:
    print("FAIL: GLFW 窗口上下文里画面是空白")
    sys.exit(1)
print("OK: GLFW 窗口上下文渲染有效")
