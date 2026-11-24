import time

import ikpy.chain
import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer
import numpy as np
import transforms3d as tf
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


# 小车
def car():
    model = mujoco.MjModel.from_xml_path("./model/car.xml")
    data = mujoco.MjData(model)

    # data.ctrl[0] = 0.0
    # data.ctrl[1] = 0.001
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 主仿真循环
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


# 机械臂测试
def ur5e():
    model = mujoco.MjModel.from_xml_path("model/universal_robots_ur5e/scene.xml")
    data = mujoco.MjData(model)

    start_joints = np.array([0, 0, 2.65, -1.3, 1.55, 0])
    data.qpos[:6] = start_joints
    data.ctrl[:6] = [-1.57, -1.34, 2.65, -1.3, 1.55, 0]

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.002)


def ur5e_vision():
    model = mujoco.MjModel.from_xml_path("model/universal_robots_ur5e/scene.xml")
    data = mujoco.MjData(model)

    start_joints = np.array([0, 0, 2.65, -1.3, 1.55, 0])
    data.qpos[:6] = start_joints
    data.ctrl[:6] = [-1.57, -1.34, 2.65, -1.3, 1.55, 0]

    # ---------- 末端轨迹记录 ----------
    site_id = model.site("attachment_site").id  # 获取末端 site 的 ID
    history_positions = []  # 存储历史位置

    # ---------- matplotlib 实时绘图准备 ----------
    plt.ion()  # 交互模式
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("UR5e End‑Effector Trajectory (real‑time)")
    (line,) = ax.plot([], [], [], "b-", linewidth=1)  # 空曲线，后续更新
    plt.show(block=False)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 初始化用户几何体计数器（清空原有内容）
        viewer.user_scn.ngeom = 0
        step_count = 0
        max_spheres = 5000  # 球体数量上限，防止无限增长

        while viewer.is_running():
            # 执行仿真步
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.002)

            # 获取当前末端位置
            current_pos = data.site_xpos[site_id].copy()
            history_positions.append(current_pos)

            # ---------- 向 MuJoCo viewer 中添加轨迹球（每 10 步添加一个）----------
            step_count += 1
            if step_count % 10 == 0 and viewer.user_scn.ngeom < max_spheres:
                mujoco.mjv_initGeom(
                    viewer.user_scn.geoms[viewer.user_scn.ngeom],
                    type=mujoco.mjtGeom.mjGEOM_SPHERE,
                    size=[0.003, 0, 0],  # 小球半径 3 mm
                    pos=np.array(current_pos, dtype=np.float64),
                    mat=np.eye(3).flatten(),
                    rgba=np.array([1.0, 0.0, 0.0, 1.0]),  # 红色
                )
                viewer.user_scn.ngeom += 1

            # ---------- 更新 matplotlib 3D 图 ----------
            if len(history_positions) > 1:
                arr = np.array(history_positions)
                line.set_data(arr[:, 0], arr[:, 1])  # X, Y
                line.set_3d_properties(arr[:, 2])  # Z
                # 动态调整坐标轴范围（可选）
                ax.relim()
                ax.autoscale_view()
                fig.canvas.draw_idle()
                plt.pause(0.001)  # 允许 GUI 更新

        # 退出后关闭 matplotlib 图窗
        plt.close(fig)


def point():
    model = mujoco.MjModel.from_xml_path("model/universal_robots_ur5e/scene.xml")
    data = mujoco.MjData(model)

    my_chain = ikpy.chain.Chain.from_urdf_file(
        "model/ur5e.urdf", active_links_mask=[False] + [True] * 6 + [False]
    )

    start_joints = np.array(
        [-1.57, -1.34, 2.65, -1.3, 1.55, 0]
    )  # 对应机械臂初始位姿[-0.14, 0.3, 0.1, 3.14, 0, 1.57]
    data.qpos[:6] = (
        start_joints  # 确保渲染一开始机械臂便处于起始位置，而非MJCF中的默认位置
    )

    # 设置目标点
    ee_pos = [-0.13, 0.6, 0.1]
    ee_euler = [3.14, 0, 1.57]
    ref_pos = [0, 0, -1.57, -1.34, 2.65, -1.3, 1.55, 0]
    ee_orientation = tf.euler.euler2mat(*ee_euler)

    # 由目标位姿求解关节角
    joint_angles = my_chain.inverse_kinematics(
        ee_pos, ee_orientation, "all", initial_position=ref_pos
    )

    end_joints = joint_angles[1:-1]

    data.ctrl[:6] = end_joints

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.002)


def viewer_init(viewer):
    """渲染器的摄像头视角初始化"""
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    viewer.cam.lookat[:] = [0, 0.5, 0.5]
    viewer.cam.distance = 2.5
    viewer.cam.azimuth = 180
    viewer.cam.elevation = -30


# 关节空间控制(MoveJ)
class JointSpaceTrajectory:
    def __init__(self, start_joints, end_joints, steps):
        self.start_joints = np.array(start_joints)
        self.end_joints = np.array(end_joints)
        self.steps = steps
        self.step = (self.end_joints - self.start_joints) / self.steps
        self.trajectory = self._generate_trajectory()
        self.waypoint = self.start_joints

    def _generate_trajectory(self):
        for i in range(self.steps + 1):
            yield self.start_joints + self.step * i
        # 确保最后精确到达目标关节值
        yield self.end_joints

    def get_next_waypoint(self, qpos):
        # 检查当前的关节值是否已经接近目标路径点。若是，则更新下一个目标路径点；若否，则保持当前目标路径点不变。
        if np.allclose(qpos, self.waypoint, atol=0.02):
            try:
                self.waypoint = next(self.trajectory)
                return self.waypoint
            except StopIteration:
                pass
        return self.waypoint


def JointSpaceControl():
    model = mujoco.MjModel.from_xml_path("model/universal_robots_ur5e/scene.xml")
    data = mujoco.MjData(model)

    my_chain = ikpy.chain.Chain.from_urdf_file(
        "model/ur5e.urdf", active_links_mask=[False] + [True] * 6 + [False]
    )

    start_joints = np.array([-1.57, -1.34, 2.65, -1.3, 1.55, 0])
    data.qpos[:6] = start_joints

    # 目标位姿
    ee_pos = [-0.13, 0.6, 0.1]
    ee_euler = [3.14, 0, 1.57]
    ref_pos = [0, -1.57, -1.34, 2.65, -1.3, 1.55, 0, 0]  # 长度 8，与链匹配
    ee_orientation = tf.euler.euler2mat(*ee_euler)

    joint_angles = my_chain.inverse_kinematics(
        ee_pos, ee_orientation, "all", initial_position=ref_pos
    )
    end_joints = joint_angles[1:-1]  # 提取活动关节角

    joint_trajectory = JointSpaceTrajectory(start_joints, end_joints, steps=100)

    # ---------- 末端轨迹记录与可视化准备 ----------
    site_id = model.site("attachment_site").id
    history_positions = []  # 存储末端位置

    # Matplotlib 实时图初始化
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("UR5e End‑Effector Trajectory (Joint Space)")
    (line,) = ax.plot([], [], [], "b-", linewidth=1.5)
    plt.show(block=False)

    # MuJoCo viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer_init(viewer)
        viewer.user_scn.ngeom = 0  # 清空用户几何体
        step_count = 0
        max_spheres = 2000  # 球体上限

        while viewer.is_running():
            # 获取关节空间路径点并控制
            waypoint = joint_trajectory.get_next_waypoint(data.qpos[:6])
            data.ctrl[:6] = waypoint

            mujoco.mj_step(model, data)
            viewer.sync()

            # 获取末端位置
            current_pos = data.site_xpos[site_id].copy()
            history_positions.append(current_pos)

            # --- 在 MuJoCo viewer 中添加轨迹球（每 5 步加一个）---
            step_count += 1
            if step_count % 5 == 0 and viewer.user_scn.ngeom < max_spheres:
                mujoco.mjv_initGeom(
                    viewer.user_scn.geoms[viewer.user_scn.ngeom],
                    type=mujoco.mjtGeom.mjGEOM_SPHERE,
                    size=[0.003, 0, 0],
                    pos=np.array(current_pos, dtype=np.float64),
                    mat=np.eye(3).flatten(),
                    rgba=np.array([1.0, 0.0, 0.0, 1.0]),
                )
                viewer.user_scn.ngeom += 1

            # --- 更新 matplotlib 3D 曲线 ---
            if len(history_positions) > 1:
                arr = np.array(history_positions)
                line.set_data(arr[:, 0], arr[:, 1])
                line.set_3d_properties(arr[:, 2])
                ax.relim()
                ax.autoscale_view()
                fig.canvas.draw_idle()
                plt.pause(0.001)

        plt.close(fig)


# 笛卡尔空间控制(MoveL)
# 位置线性插值，姿态slerp插值
class CartesianTrajectory:
    def __init__(self, p_start, euler_start, p_end, euler_end, num_points=100):
        self.num_points = num_points
        # 位置插值
        self.positions = np.linspace(p_start, p_end, num_points)
        # 姿态插值，Slerp插值
        rot_start = R.from_euler("xyz", euler_start)
        rot_end = R.from_euler("xyz", euler_end)
        self.rotations = Slerp([0, 1], R.concatenate([rot_start, rot_end]))(
            np.linspace(0, 1, num_points)
        )
        self.index = 0

    def get_next_pose(self):
        if self.index < self.num_points:
            pos = self.positions[self.index]
            rot_mat = self.rotations[self.index].as_matrix()
            self.index += 1
            return pos, rot_mat
        else:
            return self.positions[-1], self.rotations[-1].as_matrix()


def MoveCurve():
    model = mujoco.MjModel.from_xml_path("model/universal_robots_ur5e/scene.xml")
    data = mujoco.MjData(model)
    my_chain = ikpy.chain.Chain.from_urdf_file(
        "model/ur5e.urdf", active_links_mask=[False] + [True] * 6 + [False]
    )

    # 起始位姿
    p_start = [0.4, -0.15, 0.45]
    euler_start = [3.14, 0.0, 0.0]
    # 目标位姿
    p_end = [0.1, -0.15, 0.45]
    euler_end = [3.14, 0.0, 0.0]

    # 计算起始位姿
    initial_guess = [0.0] * 8
    start_rot = R.from_euler("xyz", euler_start).as_matrix()
    start_joints = my_chain.inverse_kinematics(
        p_start, start_rot, "all", initial_position=initial_guess
    )

    # 设置初始位姿
    q_start = start_joints[1:-1]  # 6个关节角
    data.qpos[:6] = q_start
    mujoco.mj_forward(model, data)

    # 得到轨迹
    traj = CartesianTrajectory(p_start, euler_start, p_end, euler_end, num_points=100)

    # ---------- 可视化准备：预先收集所有轨迹点（末端位置）----------
    # 为保持一致性，直接利用 CartesianTrajectory 内部数据
    all_positions = traj.positions  # (100, 3)
    all_rotations = traj.rotations  # 100 个 scipy Rotation 对象

    # 用于 matplotlib 的姿态坐标轴绘制
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    # 绘制位置连线
    ax.plot(
        all_positions[:, 0],
        all_positions[:, 1],
        all_positions[:, 2],
        marker="o",
        markersize=2,
    )
    # 每隔一定间隔绘制姿态坐标轴
    step = max(1, all_positions.shape[0] // 20)  # 约20组轴
    for i in range(0, all_positions.shape[0], step):
        pos = all_positions[i]
        rot_mat = all_rotations[i].as_matrix()
        for axis, color in zip([0, 1, 2], ["r", "g", "b"]):
            ax.quiver(
                pos[0],
                pos[1],
                pos[2],
                rot_mat[0, axis],
                rot_mat[1, axis],
                rot_mat[2, axis],
                color=color,
                length=0.02,
                normalize=True,
            )
    plt.ion()  # 交互模式，不阻塞
    plt.show(block=False)
    # ---------- 可视化准备：预先收集所有轨迹点（末端位置）----------

    # 参数设置
    ik_guess = list(start_joints)
    pos_tolerance = 0.01
    site_id = model.site("attachment_site").id

    # 初始化第一个目标点
    target_pos, target_rot = traj.get_next_pose()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer_init(viewer)

        # ---------- 向 viewer.user_scn 添加末端轨迹球 ----------
        # 清空原有用户几何体（如果有）
        viewer.user_scn.ngeom = 0
        # 每隔一个点添加一个红色小球（与参考代码类似）
        for i, pos in enumerate(all_positions):
            if i % 2 != 0:  # 只添加索引为奇数的点，避免过密
                continue
            # 初始化一个几何体（球体）
            mujoco.mjv_initGeom(
                viewer.user_scn.geoms[viewer.user_scn.ngeom],
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=[0.005, 0, 0],  # 半径 0.005 m
                pos=np.array(pos, dtype=np.float64),
                mat=np.eye(3).flatten(),
                rgba=np.array([1.0, 0.0, 0.0, 1.0]),  # 红色
            )
            viewer.user_scn.ngeom += 1
        print(f"Added {viewer.user_scn.ngeom} trajectory spheres to MuJoCo viewer.")
        # --------------------------------------------------------

        while viewer.is_running():
            # 获取当前末端位姿
            current_pos = data.site_xpos[site_id].copy()

            # 判断是否足够接近当前目标点，若到达，则更新目标点
            distance = np.linalg.norm(current_pos - target_pos)
            if distance < pos_tolerance:
                # 切换到下一个路点
                new_pos, new_rot = traj.get_next_pose()
                target_pos, target_rot = new_pos, new_rot

            joint_angles = my_chain.inverse_kinematics(
                target_pos, target_rot, "all", initial_position=ik_guess
            )

            data.ctrl[:6] = joint_angles[1:-1]
            ik_guess = list(joint_angles)  # 更新猜测，加速下一次 IK

            mujoco.mj_step(model, data)
            viewer.sync()

    plt.close(fig)


# 椭圆轨迹
def generate_ellipse_points(
    center,
    a,
    b,
    num_points=200,
    plane_normal=(0, 0, 1),
    start_angle=0,
    end_angle=2 * np.pi,
):

    t = np.linspace(start_angle, end_angle, num_points)
    # 二维椭圆坐标（u-v 平面）
    uv = np.zeros((num_points, 3))
    uv[:, 0] = a * np.cos(t)  # u 轴分量
    uv[:, 1] = b * np.sin(t)  # v 轴分量

    # 构造局部坐标系 u, v, n
    n = np.array(plane_normal, dtype=float)
    n /= np.linalg.norm(n)
    # 选一个不与 n 共线的参考向量
    if np.abs(n[0]) < 0.9:
        ref = np.array([1, 0, 0])
    else:
        ref = np.array([0, 1, 0])
    u = np.cross(n, ref)
    u /= np.linalg.norm(u)
    v = np.cross(n, u)  # 右手系

    # 旋转并平移到中心
    rot = np.column_stack((u, v, n))  # 3x3 旋转矩阵
    points = center + uv @ rot.T
    return points


def MoveEllipse():
    model = mujoco.MjModel.from_xml_path("model/universal_robots_ur5e/scene.xml")
    data = mujoco.MjData(model)
    my_chain = ikpy.chain.Chain.from_urdf_file(
        "model/ur5e.urdf", active_links_mask=[False] + [True] * 6 + [False]
    )

    # 定义椭圆参数
    center = [0.0, -0.2, 0.5]  # 椭圆中心
    a, b = 0.3, 0.15  # 半长轴、半短轴（米）
    plane_normal = (0, 0, 1)  # 椭圆平面为水平面（XY 平面）
    num_points = 200  # 轨迹点数

    # 生成椭圆轨迹点
    ellipse_points = generate_ellipse_points(
        center, a, b, num_points=num_points, plane_normal=plane_normal
    )

    # 设定固定的末端姿态（例如工具轴垂直向下）
    euler_fixed = [3.14, 0.0, 0.0]  # roll=π, pitch=0, yaw=0
    fixed_rot = R.from_euler("xyz", euler_fixed).as_matrix()

    # --- 1. 计算椭圆起点对应的关节角，初始化机械臂 ---
    p_start = ellipse_points[0]  # 第一个点
    initial_guess = [0.0] * 8
    start_joints = my_chain.inverse_kinematics(
        p_start, fixed_rot, "all", initial_position=initial_guess
    )
    q_start = start_joints[1:-1]  # 提取活动关节角
    data.qpos[:6] = q_start
    mujoco.mj_forward(model, data)  # 立即更新仿真状态

    # ---------- matplotlib 可视化：提前绘制椭圆轨迹 ----------
    plt.ion()  # 开启交互模式，不阻塞主循环
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("Ellipse Trajectory Preview")
    # 绘制椭圆曲线
    ax.plot(
        ellipse_points[:, 0],
        ellipse_points[:, 1],
        ellipse_points[:, 2],
        color="blue",
        linewidth=2,
        label="Desired path",
    )
    # 标记起点（绿色）和终点（红色）——椭圆是闭合的，这里终点就是起点，但仍可标注
    ax.scatter(*ellipse_points[0], color="green", s=50, label="Start")
    ax.scatter(*ellipse_points[-1], color="red", s=50, label="End")
    ax.legend()
    plt.show(block=False)
    # ---------- matplotlib 可视化：提前绘制椭圆轨迹 ----------

    # --- 2. 准备主循环 ---
    ik_guess = list(start_joints)  # 动态初始猜测
    pos_tolerance = 0.01  # 切换阈值（1 厘米）
    site_id = model.site("attachment_site").id

    # 轨迹索引
    traj_idx = 0
    target_pos = ellipse_points[traj_idx]

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer_init(viewer)

        # ---------- 在 MuJoCo viewer 中添加轨迹球 ----------
        viewer.user_scn.ngeom = 0
        # 每隔一个点添加一个红色小球，避免过于密集
        for i, pt in enumerate(ellipse_points):
            if i % 2 == 1:  # 只添加奇数索引点，可根据需要调整
                continue
            mujoco.mjv_initGeom(
                viewer.user_scn.geoms[viewer.user_scn.ngeom],
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=[0.005, 0, 0],
                pos=np.array(pt, dtype=np.float64),
                mat=np.eye(3).flatten(),
                rgba=np.array([1.0, 0.0, 0.0, 1.0]),
            )
            viewer.user_scn.ngeom += 1
        print(f"Added {viewer.user_scn.ngeom} trajectory spheres to MuJoCo viewer.")

        while viewer.is_running():
            # 获取当前末端位置
            current_pos = data.site_xpos[site_id].copy()

            # 判断是否接近当前目标点，是则切换到下一个点
            distance = np.linalg.norm(current_pos - target_pos)
            if distance < pos_tolerance and traj_idx < num_points - 1:
                traj_idx += 1
                target_pos = ellipse_points[traj_idx]

            # 逆运动学求解（姿态保持固定）
            joint_angles = my_chain.inverse_kinematics(
                target_pos, fixed_rot, "all", initial_position=ik_guess
            )
            data.ctrl[:6] = joint_angles[1:-1]
            ik_guess = list(joint_angles)  # 更新猜测，加速下一次 IK

            mujoco.mj_step(model, data)
            viewer.sync()
        plt.close(fig)


if __name__ == "__main__":
    # car()
    ur5e()
    # ur5e_vision()
    # point()
    # JointSpaceControl()
    # MoveCurve()
    # MoveEllipse()
