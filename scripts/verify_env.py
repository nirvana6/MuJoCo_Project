"""三段式环境探针：物理仿真 / 离屏渲染 / GLFW 建窗。

任何一段失败退出码非 0。

    uv run python scripts/verify_env.py                    # 默认用 model/universal_robots_ur5e/scene.xml
    uv run python scripts/verify_env.py model/car.xml      # 指定其它 MJCF
    MUJOCO_GL=egl uv run python scripts/verify_env.py      # 显式走无头 EGL
"""

import os
import sys
from pathlib import Path

import numpy as np

import mujoco

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_XML = "model/universal_robots_ur5e/scene.xml"

failures = []


def check_physics(xml_path):
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    for _ in range(500):
        mujoco.mj_step(model, data)
    if not np.isfinite(data.qpos).all():
        failures.append("physics: 500 步后 qpos 出现非有限值")
    print(
        f"physics : {Path(xml_path).name}  nq={model.nq} nv={model.nv} nu={model.nu} "
        f"nbody={model.nbody} timestep={model.opt.timestep} "
        f"qpos_range=[{data.qpos.min():.4f}, {data.qpos.max():.4f}]"
    )
    return model, data


def check_scene_assets(model):
    """项目脚本依赖的 site 是否真的存在（缺了会在 while 循环里才炸）。"""
    try:
        site_id = model.site("attachment_site").id
    except KeyError:
        failures.append("assets : 模型里没有 attachment_site（MuJoCo_project.py 依赖它）")
        return
    print(f"assets  : site 'attachment_site' id={site_id} OK")


def check_render(model, data):
    renderer = mujoco.Renderer(model, height=240, width=320)
    try:
        renderer.update_scene(data)
        pixels = renderer.render()
    finally:
        renderer.close()
    colors = len(np.unique(pixels.reshape(-1, 3), axis=0))
    print(f"render  : {pixels.shape} colors={colors} std={pixels.std():.1f}")
    # 死掉的 GL 上下文返回全零/单色帧，不会抛异常 —— 必须靠数字判断
    if pixels.std() < 1.0 or colors < 10:
        failures.append(f"render: 画面疑似空白 (std={pixels.std():.2f}, colors={colors})")


def check_window():
    import glfw

    if not glfw.init():
        failures.append("window : glfw.init() 失败（没有可用显示服务）")
        return
    try:
        glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
        win = glfw.create_window(320, 240, "probe", None, None)
        if not win:
            failures.append("window : glfw.create_window() 失败")
            return
        glfw.make_context_current(win)
        major = glfw.get_window_attrib(win, glfw.CONTEXT_VERSION_MAJOR)
        minor = glfw.get_window_attrib(win, glfw.CONTEXT_VERSION_MINOR)
        print(f"window  : 隐藏 GLFW 窗口 OK, GL {major}.{minor}")
        glfw.destroy_window(win)
    finally:
        glfw.terminate()


def main():
    xml = sys.argv[1] if len(sys.argv) > 1 else str(ROOT / DEFAULT_XML)
    if not Path(xml).is_absolute():
        xml = str(ROOT / xml)
    print(
        f"mujoco {mujoco.__version__} | numpy {np.__version__} | "
        f"MUJOCO_GL={os.environ.get('MUJOCO_GL', '(default)')} | "
        f"XDG_SESSION_TYPE={os.environ.get('XDG_SESSION_TYPE', '(unset)')}"
    )
    model, data = check_physics(xml)
    check_scene_assets(model)
    check_render(model, data)
    check_window()
    if failures:
        for f in failures:
            print("FAIL:", f)
        sys.exit(1)
    print("OK: 物理 + 离屏渲染 + 建窗 三段全部通过")


if __name__ == "__main__":
    main()
