"""项目依赖的 IK 链路探针：ikpy + transforms3d + scipy 能否真的解出关节角。

MuJoCo_project.py 里 point() / JointSpaceControl() / MoveCurve() / MoveEllipse()
都依赖这条链路，缺依赖时只在跑起来那一刻才炸，所以单独验一遍。

注意：这里验的是「依赖装好了、能解」，不是「用户的初始猜测合适」。
ikpy 的 inverse_kinematics 是局部优化，initial_position 落在坏盆地时会给出
几百毫米误差的解 —— 那是调用参数问题，不是环境问题，所以下面测多组猜测，
只要有一组收敛到毫米级就算链路可用。
"""

import sys
from pathlib import Path

import numpy as np
import transforms3d as tf
import ikpy.chain

ROOT = Path(__file__).resolve().parent.parent
chain = ikpy.chain.Chain.from_urdf_file(
    str(ROOT / "model/ur5e.urdf"),
    active_links_mask=[False] + [True] * 6 + [False],
)

target_pos = np.array([-0.13, 0.6, 0.1])
target_rot = tf.euler.euler2mat(3.14, 0, 1.57)

guesses = {
    "MuJoCo_project.py 里的 ref": [0, 0, -1.5708, -1.34, 2.65, -1.3, 1.55, 0],
    "全零": [0.0] * 8,
}

print(f"chain   : {len(chain.links)} links (4 个假 link + 6 关节 + 末端), URDF 零位 FK 位置 "
      f"{np.round(chain.forward_kinematics([0.0] * 8)[:3, 3], 4)}")

best = None
for name, ref in guesses.items():
    joints = chain.inverse_kinematics(target_pos, target_rot, "all", initial_position=ref)
    fk = chain.forward_kinematics([float(x) for x in joints])
    err = float(np.linalg.norm(fk[:3, 3] - target_pos))
    print(f"  {name:24s} q={np.round(np.asarray(joints)[1:-1], 3)}  位置误差={err * 1000:7.2f} mm")
    if np.isfinite(joints).all() and err < 0.01:
        best = err

if best is None:
    print("FAIL: 所有初始猜测都解不出 1 cm 以内的解 —— 链路或 URDF 有问题")
    sys.exit(1)
print(f"OK: ikpy 逆运动学可用（最好一组误差 {best * 1000:.2f} mm）")
