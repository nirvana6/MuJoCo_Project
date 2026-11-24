# MuJoCo_Project

UR5e 机械臂在 MuJoCo 里的运动控制实验（关节空间 / 笛卡尔空间 / 椭圆轨迹 / 视觉轨迹记录），
环境由 [uv](https://docs.astral.sh/uv/) 管理。

- 主脚本：`MuJoCo_project.py`
- 强化学习脚本：`PPO_DQN.py`（依赖是可选的 `rl` extra，见文末）
- 模型：`model/universal_robots_ur5e/scene.xml`（MuJoCo 场景）、`model/ur5e.urdf`（ikpy 用）

## 1. 环境事实（实测）

| 项目                       | 值                                                                   |
| -------------------------- | -------------------------------------------------------------------- |
| Python                     | 3.12.13（uv 托管解释器，`.python-version` 钉在 `3.12`）              |
| MuJoCo                     | 4.0.1（元包：`mujoco-core` + `mujoco-studio` + `glfw` + `pyopengl`） |
| numpy / scipy / matplotlib | 2.5.5 / 1.18.1 / 3.11.2                                              |
| ikpy                       | **4.2.1（钉死，不能升到 4.3+）**                                     |
| transforms3d               | 0.4.2                                                                |
| GPU / 显示                 | RTX 3070 Laptop（GA104M）、GNOME on Wayland、Xwayland `:0`           |
| GL                         | GLFW 隐藏窗返回 GL 4.6，离屏 EGL 渲染正常                            |

> MuJoCo 4.0 起 `pip install mujoco` 装的是**一组包**而不是单个 wheel：物理在 `mujoco-core`，
> 交互查看器在 `mujoco-studio`，开窗靠 `glfw` + `pyopengl`。所以「`uv add` 没报错」不代表
> 能出画面，必须靠下面的探针验证。

## 2. 装依赖 / 复现环境

```bash
cd /mnt/data/Documents/Github_Official_Repo/MuJoCo_Project
uv sync                      # 按 uv.lock 还原全部默认依赖（不需要先 activate）
```

`uv run <cmd>` 会自动先做一次同步再执行，所以脚本、CI、IDE 里都不依赖 shell 有没有 activate。
`.venv/` 在项目内，系统 python 一点没动。

## 3. 跑仿真

```bash
uv run python MuJoCo_project.py
```

`MuJoCo_project.py` 末尾的 `__main__` 默认只调 `ur5e()`，其余实验按需取消注释：

| 函数                  | 内容                                               |
| --------------------- | -------------------------------------------------- |
| `car()`               | `model/car.xml` 的小车                             |
| `ur5e()`              | UR5e 位姿保持，窗口 + 实时 `mj_step`               |
| `ur5e_vision()`       | 在 viewer 里逐点画末端轨迹球 + matplotlib 实时曲线 |
| `point()`             | ikpy 解单个目标位姿后发关节角指令                  |
| `JointSpaceControl()` | 关节空间直线插值（MoveJ）                          |
| `MoveCurve()`         | 笛卡尔直线 + 姿态 slerp 插值（MoveL）              |
| `MoveEllipse()`       | 水平面上的椭圆轨迹跟踪                             |

`launch_passive` 是**非阻塞**查看器，循环里要自己 `viewer.sync()`；脚本里用
`time.sleep(0.002)` 做粗配速（标准做法是按 `model.opt.timestep` 配速）。

## 4. 三段验证（环境是否真的能跑）

```bash
MUJOCO_GL=egl uv run python scripts/verify_env.py      # 物理 + 离屏渲染 + 建窗
uv run python scripts/verify_ik.py                     # ikpy / transforms3d / scipy 链路
uv run python scripts/verify_window_render.py          # GLFW 窗口上下文里渲染一帧
```

判据不是「没抛异常」，是三组数字：

| 探针                     | 实测结果                                                                      | 含义                                          |
| ------------------------ | ----------------------------------------------------------------------------- | --------------------------------------------- |
| 物理                     | `nq=6 nv=6 nu=6 nbody=10 timestep=0.002`，500 步后 `qpos ∈ [-0.0000, 0.0250]` | UR5e 模型加载、动力学推进正常                 |
| 资产                     | `site 'attachment_site' id=0`                                                 | 主脚本依赖的末端 site 存在                    |
| 离屏渲染 `MUJOCO_GL=egl` | `(240, 320, 3)`，`colors=6700`，`std=42.1`                                    | 出了真画面（死上下文会给全零/单色帧且不报错） |
| 建窗                     | 隐藏 GLFW 窗口 GL 4.6                                                         | `mujoco.viewer` 能开的前提                    |
| 窗口上下文渲染           | `colors=6700`，`std=42.1`                                                     | 开窗那条 0x502 警告没有让画面空掉             |
| IK                       | 最优一组 6 关节解 FK 回代误差 `0.00 mm`                                       | ikpy 链路可用                                 |

交互查看器用 `timeout` 验（正常时窗口永不退出，跑满即 124）：

```bash
timeout 8 uv run python MuJoCo_project.py; echo "exit=$? (124 = 窗口正常)"
```

7 个入口函数都按这个口径跑过，全部 124，无 traceback。

## 5. 可选：强化学习依赖（PPO_DQN.py）

`PPO_DQN.py` 额外要 gymnasium / stable-baselines3 / torch / tensorboard。它们被声明为
可选 extra，默认不装（torch 的 CUDA wheel 有好几个 G）：

```bash
uv sync --extra rl     # 需要时再装
```

## 6. 踩坑表

| 现象                                                                                 | 原因                                                                                                                                                                           | 处理                                                                                                                      |
| ------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------- |
| `ValueError: Several links end the tree below 'world': ['ee_link', 'tool0', 'base']` | ikpy **4.3.0 起**要求 URDF 有多个叶子时显式传 `tip=`，而 `model/ur5e.urdf` 正好有三个叶子                                                                                      | 已把 ikpy 钉在 `4.2.1`（`MuJoCo_project.py` 的 `from_urdf_file(...)` 调用不用改）；要用新版就得给每处调用加 `tip="tool0"` |
| `WARNING: OpenGL error 0x502 in or before mjr_makeContext`                           | MuJoCo 4.0.1 在 Wayland + glfw 下创建 UI 上下文时的无害告警                                                                                                                    | 忽略；已在窗口上下文里渲染一帧验证画面正常（colors=6700）                                                                 |
| `Failed to load plugin 'libdecor-gtk.so': failed to init`                            | Wayland 客户端装饰插件缺失，窗口没有标题栏圆角                                                                                                                                 | 忽略，功能不受影响；想要干净输出用 `PYGLFW_LIBRARY_VARIANT=x11`                                                           |
| `uv` 报 `Failed to hardlink files; falling back to full copy`                        | 缓存目录和项目目录不在同一个文件系统（项目在 `/mnt/data`）                                                                                                                     | 只影响安装速度；想消音用 `export UV_LINK_MODE=copy`                                                                       |
| ikpy 解出偏差几百毫米的关节角                                                        | `inverse_kinematics` 是局部优化，`initial_position` 落进坏盆地就收敛到错解（脚本里 `point()` 用的那组 `ref` 对目标 `[-0.13, 0.6, 0.1]` 实测误差 288.8 mm，全零初值则 0.00 mm） | 换初值 / 用上一帧解做 `initial_position`；这是调用参数问题，不是环境问题                                                  |
