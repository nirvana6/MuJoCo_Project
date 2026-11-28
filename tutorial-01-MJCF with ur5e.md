本教程用本仓库的 UR5e 说明 MJCF（MuJoCo XML 模型格式）编译进运行时之后的结构。实例是 `model/universal_robots_ur5e/scene.xml` 以及它引入的 `model/universal_robots_ur5e/ur5e.xml`，对照 `model/ur5e.urdf` 和 `MuJoCo_project.py`。下文的维度、力矩、位姿差都是用本环境 MuJoCo 4.0.1（`README.md:15`）执行 `MjModel.from_xml_path("model/universal_robots_ur5e/scene.xml")` 后，从 `MjModel` / `MjData` 读出的。

- [MJCF 的执行链](#sec-chain)
- [逐层读 scene.xml 与 ur5e.xml](#sec-scene-include)
- [default class 继承](#sec-default-class)
- [body/joint 树](#sec-body-joint)
- [执行器是 PD 位置伺服](#sec-actuator-pd)
- [数值契约表](#sec-numeric-contract)
- [MJCF 与 URDF 的对应关系](#sec-mjcf-vs-urdf)
- [限位差异表](#sec-joint-limits)
- [三个最小实验](#sec-minimal-experiments)
- [常见坑](#sec-pitfalls)
- [下一步](#sec-next-steps)

<a id="sec-chain"></a>

## MJCF 的执行链

文件经 `mujoco.MjModel.from_xml_path` 编译成 `MjModel`（模型常量：身体、关节、执行器参数、时间步长）。`MjData` 是状态：广义坐标 `qpos`、广义速度 `qvel`、执行器输入 `ctrl`，以及由它们算出的派生量。本仓库的加载入口是 `MuJoCo_project.py:29-30`。

```python
import mujoco

model = mujoco.MjModel.from_xml_path("model/universal_robots_ur5e/scene.xml")
data = mujoco.MjData(model)

mujoco.mj_forward(model, data)  # 填派生量，不推进时间
mujoco.mj_step(model, data)     # 推进一个 model.opt.timestep
```

`mj_forward` 与 `mj_step` 的分工，以本环境 4.0.1 的文档字符串和一次加载为准。`mj_forward` 的文档字符串是 “Forward dynamics: same as mj_step but do not integrate in time.” `mj_step` 的文档字符串是 “Advance simulation ...”。实测：

- 刚构造的 `MjData` 里 `data.site_xpos` 是 0。把 `qpos` 写成 home 关键帧之后、调用 `mj_forward` 之前，`site_xpos` 仍是 0。`mj_forward` 之后 `data.time` 仍是 0，`qpos` / `qvel` 不变，`site_xpos` 变成该关节角对应的位置。`MuJoCo_project.py:316` 和 `:481` 在写完 `qpos` 后立刻 `mj_forward`，就是为了让随后读取的 `data.site_xpos`（`:364`、`:510`）对应该关节角。
- `mj_step` 把 `data.time` 从 0 推进到 `model.opt.timestep`（本模型是 0.002），并更新 `qpos`、`qvel`。仿真循环里的调用点是 `MuJoCo_project.py:38`。

派生量里本仓库直接用到的是运动学位置 `data.site_xpos` / `data.xpos`，以及执行器输出 `data.actuator_force`（gear 为 1 时与 `data.qfrc_actuator` 逐元素相同）。`mj_forward` 负责把这些量从当前状态算出来；`mj_step` 在此之外还把状态推进一个时间步。

<a id="sec-scene-include"></a>

## 逐层读 scene.xml 与 ur5e.xml

**model/universal_robots_ur5e/scene.xml:1-38**

```xml
   1 │ <mujoco model="ur5e scene">
   2 │   <include file="ur5e.xml"/>
   3 │
   4 │   <statistic center="0.3 0 0.4" extent="0.8"/>
   5 │
   6 │   <visual>
   7 │     <headlight diffuse="0.6 0.6 0.6" ambient="0.1 0.1 0.1" specular="0 0 0"/>
   8 │     <rgba haze="0.15 0.25 0.35 1"/>
   9 │     <global azimuth="120" elevation="-20"/>
  10 │   </visual>
  11 │
  12 │   <asset>
  13 │     <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
  14 │     <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
  15 │       markrgb="0.8 0.8 0.8" width="300" height="300"/>
  16 │     <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  17 │   </asset>
  18 │
  19 │   <worldbody>
  20 │     <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
  21 │     <!--<geom
  22 │     name="ball1" type="sphere" size="0.03" pos="0.30 0.30 0.30" rgba="1 0 0 1"/>-->
  23 │     <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
  24 │     <body name="world_frame" pos="0 0 0">
  25 │       <geom name="X" type="cylinder" size="0.005" fromto="0 0 0 0.3 0 0" rgba="1 0 0 1" contype="0"
  26 │         conaffinity="0"/>
  27 │       <geom name="Y" type="cylinder" size="0.005" fromto="0 0 0 0 0.3 0" rgba="0 1 0 1" contype="0"
  28 │         conaffinity="0"/>
  29 │       <geom name="Z" type="cylinder" size="0.005" fromto="0 0 0 0 0 0.3" rgba="0 0 1 1" contype="0"
  30 │         conaffinity="0"/>
  31 │     </body>
  32 │
  33 │     <body name="target" pos="-0.13 0.5 0.1">
  34 │       <geom size="0.02" rgba="0 1 0 0.5" contype="0" conaffinity="0"/>
  35 │     </body>
  36 │
  37 │   </worldbody>
  38 │ </mujoco>
```

### include 的路径

`scene.xml:2` 是 `<include file="ur5e.xml"/>`。这条相对路径相对于写出 include 的文件所在目录，也就是 `model/universal_robots_ur5e/ur5e.xml`，与进程的当前工作目录无关。在 `/tmp` 下用绝对路径加载 `scene.xml`，仍然得到 `nq=6`、`nmesh=20`。

传给 `from_xml_path` 的字符串是普通文件系统路径。当前工作目录不在仓库根时，相对路径 `model/universal_robots_ur5e/scene.xml` 会打不开。include 与这个参数不是同一条规则。

`compiler meshdir="assets"`（`ur5e.xml:3`）相对于传给 `from_xml_path` 的顶层文件所在目录，不相对于当前工作目录。本仓库的顶层文件是 `scene.xml`，与 `ur5e.xml` 同目录，因此 20 个 obj（`ur5e.xml:40-59`，加载后 `nmesh=20`）来自 `model/universal_robots_ur5e/assets/`。把 `ur5e.xml` 以绝对路径 include 进另一个目录中的顶层文件时，若该顶层目录下没有 `assets/`，编译找不到网格：`meshdir` 不会改走被 include 文件的真实目录。

### compiler

`ur5e.xml:3` 三项一起生效：

- `angle="radian"`：本文件里的 `range`、`ctrlrange` 按弧度读入。加载后 `shoulder_pan_joint` 的 `jnt_range` 是 ±6.28319，与 XML 字面相同。若把 6.28319 当成度再换算，大约只有 0.110 rad，与加载结果对不上。
- `meshdir="assets"`：见上一节。
- `autolimits="true"`：关节只写了 `range`，没有写 `limited`。加载后 6 个关节的 `jnt_limited` 都是 1，`range` 是生效限位。把同一份 XML 的 `autolimits` 改成 `false` 后，编译在 `shoulder_pan_joint`（`ur5e.xml:70`）失败，错误是 `joint has range but not limited`。

### option 与场景外观

`ur5e.xml:4` 的 `<option>` 只写了 `integrator="implicitfast"`。加载后 `model.opt.integrator` 是枚举 `mjINT_IMPLICITFAST`（整型值 3）。同一枚举里 Euler 是 `mjINT_EULER`（0）。文件没有写 `timestep`，加载值是 0.002。文件也没有写 `gravity`，加载值是 `[0, 0, -9.81]`。

下面三块不进入广义坐标：

- `<statistic center="0.3 0 0.4" extent="0.8"/>`（`scene.xml:4`）写入 `model.stat`，给自由相机取景。实测 `center=[0.3, 0, 0.4]`，`extent=0.8`。
- `<visual>`（`scene.xml:6-10`）是头灯、雾色和默认视角。实测 `azimuth=120`，`elevation=-20`，头灯 diffuse 是 `[0.6, 0.6, 0.6]`。
- `<asset>` 在 `scene.xml:12-17` 是天空盒和地面棋盘纹理，在 `ur5e.xml:34-59` 是材质与网格。地面几何体是 `scene.xml:23` 的 `floor`，使用材质 `groundplane`；它的 `contype=1`、`conaffinity=1`，参与接触。

`scene.xml` 的 `<worldbody>` 在机器人之外加了 `floor`、坐标轴 `world_frame`（`scene.xml:24-31`）和绿球 `target`（`scene.xml:33-35`）。身体计数见「数值契约表」。

<a id="sec-default-class"></a>

## default class 继承

**model/universal_robots_ur5e/ur5e.xml:6-32**

```xml
   6 │   <default>
   7 │     <default class="ur5e">
   8 │       <material specular="0.5" shininess="0.25"/>
   9 │       <joint axis="0 1 0" range="-6.28319 6.28319" armature="0.1"/>
  10 │       <general gaintype="fixed" biastype="affine" ctrlrange="-6.2831 6.2831" gainprm="2000" biasprm="0 -2000 -400"
  11 │         forcerange="-150 150"/>
  12 │       <default class="size3">
  13 │         <default class="size3_limited">
  14 │           <joint range="-3.1415 3.1415"/>
  15 │           <general ctrlrange="-3.1415 3.1415"/>
  16 │         </default>
  17 │       </default>
  18 │       <default class="size1">
  19 │         <general gainprm="500" biasprm="0 -500 -100" forcerange="-28 28"/>
  20 │       </default>
  21 │       <default class="visual">
  22 │         <geom type="mesh" contype="0" conaffinity="0" group="2"/>
  23 │       </default>
  24 │       <default class="collision">
  25 │         <geom type="capsule" group="3"/>
  26 │         <default class="eef_collision">
  27 │           <geom type="cylinder"/>
  28 │         </default>
  29 │       </default>
  30 │       <site size="0.001" rgba="0.5 0.5 0.5 0.3" group="4"/>
  31 │     </default>
  32 │   </default>
```

默认类（default class）嵌套继承：子 class 继承父 class 已写的属性，同名属性用更近的一层覆盖。元素上的 `class="..."` 选用那一层；元素自己再写的属性覆盖 class。`base` 上的 `childclass="ur5e"`（`ur5e.xml:64`）让未指定 class 的子元素进入这棵树。

定义在 `ur5e.xml:7-29`。`size3`（`:12-17`）自身没有新属性，只为了再嵌出 `size3_limited`。下表的执行器列是加载后的 `actuator_gainprm[:,0]`、`actuator_biasprm`、`actuator_forcerange`、`actuator_ctrlrange`；关节列是 `jnt_range`；geom 列是对应几何体的 `geom_group` / `geom_contype` / `geom_conaffinity`。`visual` 的接触标志写在 XML 里；`collision` 的 class 没有写 `contype` / `conaffinity`，表中 1/1 是加载值（例如 `shoulder_link` 上 `ur5e.xml:74` 那颗胶囊）。

| class           | 父 class | 用在                                                 | gainprm | biasprm        | forcerange | joint range | ctrlrange | group | contype / conaffinity |
| --------------- | -------- | ---------------------------------------------------- | ------- | -------------- | ---------- | ----------- | --------- | ----- | --------------------- |
| `ur5e`          | —        | 下面各层的父默认                                     | 2000    | `0 -2000 -400` | ±150       | ±6.28319    | ±6.2831   | —     | —                     |
| `size3`         | `ur5e`   | `shoulder_pan`、`shoulder_lift` 的 joint 与 actuator | 2000    | `0 -2000 -400` | ±150       | ±6.28319    | ±6.2831   | —     | —                     |
| `size3_limited` | `size3`  | `elbow` 的 joint 与 actuator                         | 2000    | `0 -2000 -400` | ±150       | ±3.1415     | ±3.1415   | —     | —                     |
| `size1`         | `ur5e`   | 三个 wrist 的 joint 与 actuator                      | 500     | `0 -500 -100`  | ±28        | ±6.28319    | ±6.2831   | —     | —                     |
| `visual`        | `ur5e`   | 各 link 的 mesh geom                                 | —       | —              | —          | —           | —         | 2     | 0 / 0                 |
| `collision`     | `ur5e`   | 各 link 的胶囊 geom                                  | —       | —              | —          | —           | —         | 3     | 1 / 1（加载值）       |

`size3_limited` 只收紧 `joint range` 和 `ctrlrange`（`ur5e.xml:14-15`），增益和 `forcerange` 仍是父层的 2000 与 ±150。`size1`（`:18-20`）改的是增益、偏置和 `forcerange`，关节范围和 `ctrlrange` 仍是 `ur5e` 的 ±6.28319 与 ±6.2831。关节范围与 `ctrlrange` 不是同一个字面量：前者 6.28319，后者 6.2831。

`collision` 里面还嵌了 `eef_collision`（`ur5e.xml:26-28`），把几何类型改成圆柱，接触标志和 group 3 仍从 `collision` 继承。`wrist_3_link` 的末端碰撞体用的是这个 class（`ur5e.xml:113`）。加载后它是 `group=3`、`contype=1`、`conaffinity=1` 的 cylinder。

两套 geom 的分工：

- `visual`（`ur5e.xml:21-23`）是网格，`contype=0` 且 `conaffinity=0`，不参与接触，`group=2`。模型说明里这组碰撞体外形是手工简化过的（`model/universal_robots_ur5e/README.md:36`），网格只负责外观。
- `collision` 是胶囊（末端那一个是圆柱），`contype=1`、`conaffinity=1`，与 `floor` 发生接触，`group=3`。
- 本环境新建 `mujoco.MjvOption()` 时 `geomgroup` 是 `[1, 1, 1, 0, 0, 0]`，group 0–2 打开、group 3 关闭。查看器保持这个默认时，窗口里是 mesh，胶囊要打开 group 3 才画出来。

`scene.xml:25-34` 的坐标轴和绿球也写了 `contype="0"`、`conaffinity="0"`，它们是标注，不参与接触；group 为默认 0，所以窗口里看得见。

<a id="sec-body-joint"></a>

## body/joint 树

**model/universal_robots_ur5e/ur5e.xml:62-123**

```xml
  62 │   <worldbody>
  63 │     <light name="spotlight" mode="targetbodycom" target="wrist_2_link" pos="0 -1 2"/>
  64 │     <body name="base" quat="1 0 0 0" childclass="ur5e">
  65 │       <inertial mass="4.0" pos="0 0 0" diaginertia="0.00443333156 0.00443333156 0.0072"/>
  66 │       <geom mesh="base_0" material="black" class="visual"/>
  67 │       <geom mesh="base_1" material="jointgray" class="visual"/>
  68 │       <body name="shoulder_link" pos="0 0 0.163">
  69 │         <inertial mass="3.7" pos="0 0 0" diaginertia="0.0102675 0.0102675 0.00666"/>
  70 │         <joint name="shoulder_pan_joint" class="size3" axis="0 0 1"/>
  71 │         <geom mesh="shoulder_0" material="urblue" class="visual"/>
  72 │         <geom mesh="shoulder_1" material="black" class="visual"/>
  73 │         <geom mesh="shoulder_2" material="jointgray" class="visual"/>
  74 │         <geom class="collision" size="0.06 0.06" pos="0 0 -0.04"/>
  75 │         <body name="upper_arm_link" pos="0 0.138 0" quat="1 0 1 0">
  76 │           <inertial mass="8.393" pos="0 0 0.2125" diaginertia="0.133886 0.133886 0.0151074"/>
  77 │           <joint name="shoulder_lift_joint" class="size3"/>
  78 │           <geom mesh="upperarm_0" material="linkgray" class="visual"/>
  79 │           <geom mesh="upperarm_1" material="black" class="visual"/>
  80 │           <geom mesh="upperarm_2" material="jointgray" class="visual"/>
  81 │           <geom mesh="upperarm_3" material="urblue" class="visual"/>
  82 │           <geom class="collision" pos="0 -0.04 0" quat="1 1 0 0" size="0.06 0.06"/>
  83 │           <geom class="collision" size="0.05 0.2" pos="0 0 0.2"/>
  84 │           <body name="forearm_link" pos="0 -0.131 0.425">
  85 │             <inertial mass="2.275" pos="0 0 0.196" diaginertia="0.0311796 0.0311796 0.004095"/>
  86 │             <joint name="elbow_joint" class="size3_limited"/>
  87 │             <geom mesh="forearm_0" material="urblue" class="visual"/>
  88 │             <geom mesh="forearm_1" material="linkgray" class="visual"/>
  89 │             <geom mesh="forearm_2" material="black" class="visual"/>
  90 │             <geom mesh="forearm_3" material="jointgray" class="visual"/>
  91 │             <geom class="collision" pos="0 0.08 0" quat="1 1 0 0" size="0.055 0.06"/>
  92 │             <geom class="collision" size="0.038 0.19" pos="0 0 0.2"/>
  93 │             <body name="wrist_1_link" pos="0 0 0.392" quat="1 0 1 0">
  94 │               <inertial mass="1.219" pos="0 0.127 0" diaginertia="0.0025599 0.0025599 0.0021942"/>
  95 │               <joint name="wrist_1_joint" class="size1"/>
  96 │               <geom mesh="wrist1_0" material="black" class="visual"/>
  97 │               <geom mesh="wrist1_1" material="urblue" class="visual"/>
  98 │               <geom mesh="wrist1_2" material="jointgray" class="visual"/>
  99 │               <geom class="collision" pos="0 0.05 0" quat="1 1 0 0" size="0.04 0.07"/>
 100 │               <body name="wrist_2_link" pos="0 0.127 0">
 101 │                 <inertial mass="1.219" pos="0 0 0.1" diaginertia="0.0025599 0.0025599 0.0021942"/>
 102 │                 <joint name="wrist_2_joint" axis="0 0 1" class="size1"/>
 103 │                 <geom mesh="wrist2_0" material="black" class="visual"/>
 104 │                 <geom mesh="wrist2_1" material="urblue" class="visual"/>
 105 │                 <geom mesh="wrist2_2" material="jointgray" class="visual"/>
 106 │                 <geom class="collision" size="0.04 0.06" pos="0 0 0.04"/>
 107 │                 <geom class="collision" pos="0 0.02 0.1" quat="1 1 0 0" size="0.04 0.04"/>
 108 │                 <body name="wrist_3_link" pos="0 0 0.1">
 109 │                   <inertial mass="0.1889" pos="0 0.0771683 0" quat="1 0 0 1"
 110 │                     diaginertia="0.000132134 9.90863e-05 9.90863e-05"/>
 111 │                   <joint name="wrist_3_joint" class="size1"/>
 112 │                   <geom material="linkgray" mesh="wrist3" class="visual"/>
 113 │                   <geom class="eef_collision" pos="0 0.08 0" quat="1 1 0 0" size="0.04 0.02"/>
 114 │                   <site name="attachment_site" pos="0 0.1 0" quat="-1 1 0 0"/>
 115 │                 </body>
 116 │               </body>
 117 │             </body>
 118 │           </body>
 119 │         </body>
 120 │       </body>
 121 │     </body>
 122 │   </worldbody>
 123 │
```

`pos` 和 `quat`（四元数，MuJoCo 顺序 w x y z）是父 body 坐标系里、关节角为 0 时的子 body 位姿。世界位姿由 world 沿父链把这些相对位姿连乘得到。铰链转角叠在这个参考位姿上；`qpos=0` 时铰链是单位旋转，此时 `data.xpos` 只由 `pos` / `quat` 的连乘决定。下面每级都给出 XML 里的父级相对量。`qpos=0` 且 `mj_forward` 之后的世界坐标是加载值。

`base`（`ur5e.xml:64`）的父级是 `world`，位置在原点，`quat="1 0 0 0"`。它没有 joint，加载后 `body_jntnum=0`，因此固连在世界上，不是自由刚体。

`shoulder_link`（`:68`）相对 `base` 的 `pos="0 0 0.163"`，姿态为单位阵。`qpos=0` 时世界坐标是 `[0, 0, 0.163]`。关节 `shoulder_pan_joint`（`:70`）属于这个 body，`class="size3"`，`axis="0 0 1"` 覆盖了 `ur5e` 默认的 `axis="0 1 0"`（`:9`）。

`upper_arm_link`（`:75`）相对 `shoulder_link` 的 `pos="0 0.138 0"`，`quat="1 0 1 0"`。该四元数不是单位长度，编译后归一化成 `[0.7071067811865475, 0, 0.7071067811865475, 0]`，即绕父系 Y 轴 +π/2。`qpos=0` 时世界坐标是 `[0, 0.138, 0.163]`。关节 `shoulder_lift_joint`（`:77`）`class="size3"`，轴用继承的 `0 1 0`。

`forearm_link`（`:84`）相对 `upper_arm_link` 的 `pos="0 -0.131 0.425"`，无 `quat`。把这个矢量用父级的 Ry(+π/2) 转到世界再加父级位置：Ry(+π/2) 作用后是 `(0.425, -0.131, 0)`，加上 `(0, 0.138, 0.163)` 得到 `(0.425, 0.007, 0.163)`。加载的 `forearm_link` 的 `xpos` 就是 `[0.425, 0.007, 0.163]`。关节 `elbow_joint`（`:86`）`class="size3_limited"`，轴 `0 1 0`。

`wrist_1_link`（`:93`）相对 `forearm_link` 的 `pos="0 0 0.392"`，`quat="1 0 1 0"`，编译后同样是绕 Y 的 +π/2。关节 `wrist_1_joint`（`:95`）`class="size1"`，轴 `0 1 0`。

`wrist_2_link`（`:100`）相对 `wrist_1_link` 的 `pos="0 0.127 0"`。关节 `wrist_2_joint`（`:102`）`class="size1"`，`axis="0 0 1"` 再次覆盖默认轴。

`wrist_3_link`（`:108`）相对 `wrist_2_link` 的 `pos="0 0 0.1"`。关节 `wrist_3_joint`（`:111`）`class="size1"`，轴 `0 1 0`。`attachment_site`（`:114`）还在这个 body 里，`pos="0 0.1 0"`、`quat="-1 1 0 0"`，同样是相对 `wrist_3_link`，不是世界坐标。`qpos=0`、`mj_forward` 之后 `data.site_xpos` 是 `[0.817, 0.234, 0.063]`；`mj_forward` 之前是 0。

六个关节都是铰链（`mjJNT_HINGE`），`jnt_limited=1`。`jnt_range` 与上表 class 继承一致。

| joint                 | body             | axis    | jnt_range（加载值） |
| --------------------- | ---------------- | ------- | ------------------- |
| `shoulder_pan_joint`  | `shoulder_link`  | `0 0 1` | ±6.28319            |
| `shoulder_lift_joint` | `upper_arm_link` | `0 1 0` | ±6.28319            |
| `elbow_joint`         | `forearm_link`   | `0 1 0` | ±3.1415             |
| `wrist_1_joint`       | `wrist_1_link`   | `0 1 0` | ±6.28319            |
| `wrist_2_joint`       | `wrist_2_link`   | `0 0 1` | ±6.28319            |
| `wrist_3_joint`       | `wrist_3_link`   | `0 1 0` | ±6.28319            |

<a id="sec-actuator-pd"></a>

## 执行器是 PD 位置伺服

**model/universal_robots_ur5e/ur5e.xml:125-136**

```xml
 125 │   <actuator>
 126 │     <general class="size3" name="shoulder_pan" joint="shoulder_pan_joint"/>
 127 │     <general class="size3" name="shoulder_lift" joint="shoulder_lift_joint"/>
 128 │     <general class="size3_limited" name="elbow" joint="elbow_joint"/>
 129 │     <general class="size1" name="wrist_1" joint="wrist_1_joint"/>
 130 │     <general class="size1" name="wrist_2" joint="wrist_2_joint"/>
 131 │     <general class="size1" name="wrist_3" joint="wrist_3_joint"/>
 132 │   </actuator>
 133 │
 134 │   <keyframe>
 135 │     <key name="home" qpos="-1.5708 -1.5708 1.5708 -1.5708 -1.5708 0" ctrl="-1.5708 -1.5708 1.5708 -1.5708 -1.5708 0"/>
 136 │   </keyframe>
```

`<actuator>`（`ur5e.xml:125-131`）里每个 `<general joint="...">` 直接驱动同名关节。加载后 `actuator_trntype` 是 `mjTRN_JOINT`，`actuator_gear` 的第一分量是 1，`actuator_dyntype` 是 `mjDYN_NONE`，`actuator_gaintype` 是 `mjGAIN_FIXED`，`actuator_biastype` 是 `mjBIAS_AFFINE`。没有执行器动力学，`ctrl` 不经过滤波，直接进入下面的仿射式。传动长度就是关节角 `q`，传动速度就是 `qdot`。

固定增益、仿射偏置写成

$$
\tau = p_{\mathrm{gain},0}\,\mathrm{ctrl} + p_{\mathrm{bias},0} + p_{\mathrm{bias},1}\, q + p_{\mathrm{bias},2}\, \dot{q}
$$

`p_gain` 是 `gainprm`，`p_bias` 是 `biasprm`。`ur5e` 的 general（`ur5e.xml:10-11`）是 `gainprm="2000"`、`biasprm="0 -2000 -400"`，于是 `p_gain,0 = 2000`，`p_bias = (0, -2000, -400)`。代入得

$$
\tau = K_p (\mathrm{ctrl} - q) - K_d \dot{q}
$$

符号：

- $\tau$：该执行器输出的关节力矩，读 `data.actuator_force`。gear 为 1 时与 `data.qfrc_actuator` 相同。实测在一组非零 `q`、`qvel`、`ctrl` 上，两者与上式截断后的值最大绝对误差是 0。
- $K_p$：位置增益，等于 `gainprm[0]`，也等于 `-biasprm[1]`。前三个执行器是 2000，三个腕是 500。
- $\mathrm{ctrl}$：`data.ctrl` 的对应分量，位置设定点（setpoint），单位弧度。
- $q$：`data.qpos` 的对应分量，当前关节角。
- $K_d$：速度增益，等于 `-biasprm[2]`。前三个是 400（来自 `-400`），腕是 100（来自 `biasprm="0 -500 -100"`）。
- $\dot{q}$：`data.qvel` 的对应分量。

算完之后再按 `forcerange` 截断。前三个执行器（含 elbow）是 ±150，腕是 ±28。elbow 走 `size3_limited`，$K_p$ / $K_d$ 仍是 2000 / 400，变的是关节范围和 `ctrlrange`（±3.1415）。腕的 500/100 与 ±28 来自 `size1`。加载后六个执行器 `actuator_forcelimited` 与 `actuator_ctrllimited` 都是 1：力矩限制是 `forcerange`，设定点限制是 `ctrlrange`（elbow ±3.1415，其余 ±6.2831）。

因此 `data.ctrl` 是目标关节角，不是力矩。力矩是上式的 $\tau$。对照：`q=0`、$\dot{q}=0$、`ctrl=-1.57` 时，未截断的 $\tau = 2000 \times (-1.57) = -3140$，截断后是 −150。若把 `ctrl` 的数值直接当成力矩，这一步会是 −1.57 N·m。

静平衡、未顶到 `forcerange`、接触力和限位力都不参与时，$\dot{q}=0$，上式剩下 $K_p (\mathrm{ctrl} - q) = \tau$。零速下的 `data.qfrc_bias` 就是广义重力 $\tau_g$：同一姿态把 `model.opt.gravity` 设为 0 再 `mj_forward`，`qfrc_bias` 变为全 0。平衡时执行器力矩等于这项，所以

$$
\mathrm{ctrl} - q = \frac{\tau_g}{K_p}
$$

稳态角度误差与 `K_p` 成反比。`biasprm[0]=0`，这个 general 没有用来抵消重力的常数项。

用 home 关键帧（`mj_resetDataKeyframe` 后 `ctrl` 与 `qpos` 同为 `[-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0]`）积分 5000 步，`data.time=10`，`ncon=0`，`qvel` 回到约 `1e-14`。此时

| joint           | `ctrl - q`（rad） | `actuator_force`（N·m） | `K_p` |
| --------------- | ----------------- | ----------------------- | ----- |
| `shoulder_lift` | -0.008079391      | -16.158783              | 2000  |
| `elbow`         | -0.007926223      | -15.852446              | 2000  |
| `wrist_1`       | -0.002756449      | -1.378225               | 500   |

`actuator_force / K_p` 与 `ctrl - q` 的差在 `1e-14` rad。同一时刻 `shoulder_pan`、`wrist_2`、`wrist_3` 的广义重力约为 0，静差也约为 0。这是该姿态的结果，不是这三个轴上永远没有重力。

<a id="sec-numeric-contract"></a>

## 数值契约表

**python 片段（行号为片段内序号 1-12，可直接运行）**

```python
   1 │ import mujoco
   2 │
   3 │ model = mujoco.MjModel.from_xml_path(
   4 │     "model/universal_robots_ur5e/scene.xml"
   5 │ )
   6 │ print("nq", model.nq)
   7 │ print("nv", model.nv)
   8 │ print("nu", model.nu)
   9 │ print("nbody", model.nbody)
  10 │ print("timestep", float(model.opt.timestep))
  11 │ print("integrator", mujoco.mjtIntegrator(int(model.opt.integrator)).name)
  12 │ print("armature", [float(x) for x in model.dof_armature])
```

下表来自加载 `scene.xml` 后的 `MjModel`。`README.md:69` 的探针记录了同一组 `nq nv nu nbody timestep`。

| 量           | 加载值             | 与文件的对应                                                                                                                                |
| ------------ | ------------------ | ------------------------------------------------------------------------------------------------------------------------------------------- |
| `nq`         | 6                  | 6 个铰链，每个贡献 1 个位置坐标                                                                                                             |
| `nv`         | 6                  | 每个铰链贡献 1 个速度坐标                                                                                                                   |
| `nu`         | 6                  | 6 个 `<general>`（`ur5e.xml:126-131`）                                                                                                      |
| `nbody`      | 10                 | `world`、`base`、`shoulder_link`、`upper_arm_link`、`forearm_link`、`wrist_1_link`、`wrist_2_link`、`wrist_3_link`、`world_frame`、`target` |
| `timestep`   | 0.002              | `<option>` 未写 timestep（`ur5e.xml:4`）                                                                                                    |
| `integrator` | `implicitfast`     | `ur5e.xml:4`；枚举 `mjINT_IMPLICITFAST`，值 3                                                                                               |
| `armature`   | 0.1                | `ur5e.xml:9`；六个 `dof_armature` 都是 0.1                                                                                                  |
| 基座         | 固定，无 freejoint | `base` 的 `jntnum=0`（`ur5e.xml:64` 无 joint）。关节类型全是铰链。自由关节会让 `nq` 再加 7，这里 `nq=6`                                     |

`model.qpos0` 是全 0。名为 `home` 的关键帧不会在 `MjData()` 构造时自动写入。

<a id="sec-mjcf-vs-urdf"></a>

## MJCF 与 URDF 的对应关系

**model/universal_robots_ur5e/ur5e.xml:108-115**

```xml
 108 │                 <body name="wrist_3_link" pos="0 0 0.1">
 109 │                   <inertial mass="0.1889" pos="0 0.0771683 0" quat="1 0 0 1"
 110 │                     diaginertia="0.000132134 9.90863e-05 9.90863e-05"/>
 111 │                   <joint name="wrist_3_joint" class="size1"/>
 112 │                   <geom material="linkgray" mesh="wrist3" class="visual"/>
 113 │                   <geom class="eef_collision" pos="0 0.08 0" quat="1 1 0 0" size="0.04 0.02"/>
 114 │                   <site name="attachment_site" pos="0 0.1 0" quat="-1 1 0 0"/>
 115 │                 </body>
```

**model/ur5e.urdf:229-240**

```xml
 229 │       <origin rpy="1.5707963267948966 0 0" xyz="0.0 0.0771 0.0"/>
 230 │       <inertia ixx="9.890410052167731e-05" ixy="0.0" ixz="0.0" iyy="9.890410052167731e-05" iyz="0.0" izz="0.0001321171875"/>
 231 │     </inertial>
 232 │   </link>
 233 │   <joint name="ee_fixed_joint" type="fixed">
 234 │     <parent link="wrist_3_link"/>
 235 │     <child link="ee_link"/>
 236 │     <origin rpy="0.0 0.0 1.5707963267948966" xyz="0.0 0.1 0.0"/>
 237 │   </joint>
 238 │   <link name="ee_link">
 239 │     <collision>
 240 │       <geometry>
```

URDF 末端固定关节 `ee_fixed_joint`（`model/ur5e.urdf:233-237`）把 `ee_link` 安在 `wrist_3_link` 上，`xyz="0.0 0.1 0.0"`，`rpy="0.0 0.0 1.5707963267948966"`（即绕 Z 轴 +π/2）。MJCF 的 `attachment_site`（`ur5e.xml:114`）在同一个父 body 上，`pos="0 0.1 0"`，`quat="-1 1 0 0"`。位置矢量相同。姿态属性不同。

编译后的 site 四元数是 `[-0.7071067811865475, 0.7071067811865475, 0, 0]`，即绕 X 轴 −π/2，父系旋转矩阵是

$$
R_{\mathrm{site,local}} = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 0 & 1 \\ 0 & -1 & 0 \end{bmatrix}
$$

URDF 的 rpy 是绕 Z 轴 +π/2：

$$
R_{\mathrm{ee,local}} = \begin{bmatrix} 0 & -1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 1 \end{bmatrix}
$$

两者都表达在 `wrist_3_link` 里，相对旋转 `R_ee,local` 的转置乘 `R_site,local` 就是下面的 `M`。

ikpy 4.2.1 按 `MuJoCo_project.py:114-116` 建链时，会在 URDF 外面加一个 `Base link`，末节名字是 `ee_fixed_joint`（子 link 为 `ee_link`）。所以 `active_links_mask` 长度是 8，首尾为 False，中间 6 个关节为 True。`forward_kinematics([0, q1, …, q6, 0])` 的 3×3 块记为 `R_ikpy`，平移记为末端位置。`data.site_xmat` 按行优先排成 3×3，记为 `R_site`。在全零、home、`MuJoCo_project.py:32` 的 `[0, 0, 2.65, -1.3, 1.55, 0]`，以及 `[0.4, -0.7, 1.2, -0.5, 0.8, -1.1]` 四个姿态上：

- 位置差的最大绝对值不超过 `2.3×10⁻¹⁶` m。父系里相同的 `0 0.1 0` 平移，加上两条链一致的关节原点，使末端原点重合。
- 姿态差是常数矩阵。实测

$$
M = R_{\mathrm{ikpy}}^{\mathsf T} R_{\mathrm{site}} = \begin{bmatrix} 0 & 0 & 1 \\ -1 & 0 & 0 \\ 0 & -1 & 0 \end{bmatrix}
$$

四个姿态上，`M` 与这个整数矩阵的元素差最大约 `1.4×10⁻¹⁵`。`\mathrm{tr}(M)=0`，转角

$$
\theta = \arccos\frac{\mathrm{tr}(M)-1}{2} = \arccos\left(-\frac{1}{2}\right) = 120^\circ
$$

因此 `R_site = R_ikpy M`。脚本里的 `ee_euler`（`MuJoCo_project.py:127`）经 `transforms3d.euler.euler2mat`（`:129`）交给 `inverse_kinematics`（`:132-134`），目标姿态是这条 URDF 链末端 `ee_link` 相对世界的旋转，也就是 `R_ikpy` 所在的坐标系。`data.site_xmat` 是 `attachment_site` 相对世界的旋转。二者相差这一个固定的 `M`，不能逐元素比较。位置可以直接比。

<a id="sec-joint-limits"></a>

## 限位差异表

**model/ur5e.urdf:65-72**

```xml
  65 │   <joint name="shoulder_pan_joint" type="revolute">
  66 │     <parent link="base_link"/>
  67 │     <child link="shoulder_link"/>
  68 │     <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.163"/>
  69 │     <axis xyz="0 0 1"/>
  70 │     <limit effort="150.0" lower="-3.0" upper="3.0" velocity="3.14"/>
  71 │     <dynamics damping="0.0" friction="0.0"/>
  72 │   </joint>
```

**model/ur5e.urdf:93-100**

```xml
  93 │   <joint name="shoulder_lift_joint" type="revolute">
  94 │     <parent link="shoulder_link"/>
  95 │     <child link="upper_arm_link"/>
  96 │     <origin rpy="0.0 1.5707963267948966 0.0" xyz="0.0 0.138 0.0"/>
  97 │     <axis xyz="0 1 0"/>
  98 │     <limit effort="150.0" lower="-3.141592653589793" upper="3.141592653589793" velocity="3.14"/>
  99 │     <dynamics damping="0.0" friction="0.0"/>
 100 │   </joint>
```

**model/ur5e.urdf:121-128**

```xml
 121 │   <joint name="elbow_joint" type="revolute">
 122 │     <parent link="upper_arm_link"/>
 123 │     <child link="forearm_link"/>
 124 │     <origin rpy="0.0 0.0 0.0" xyz="0.0 -0.131 0.425"/>
 125 │     <axis xyz="0 1 0"/>
 126 │     <limit effort="150.0" lower="-3.141592653589793" upper="3.141592653589793" velocity="3.14"/>
 127 │     <dynamics damping="0.0" friction="0.0"/>
 128 │   </joint>
```

**model/ur5e.urdf:149-156**

```xml
 149 │   <joint name="wrist_1_joint" type="revolute">
 150 │     <parent link="forearm_link"/>
 151 │     <child link="wrist_1_link"/>
 152 │     <origin rpy="0.0 1.5707963267948966 0.0" xyz="0.0 0.0 0.392"/>
 153 │     <axis xyz="0 1 0"/>
 154 │     <limit effort="28.0" lower="-3.141592653589793" upper="3.141592653589793" velocity="6.28"/>
 155 │     <dynamics damping="0.0" friction="0.0"/>
 156 │   </joint>
```

**model/ur5e.urdf:177-184**

```xml
 177 │   <joint name="wrist_2_joint" type="revolute">
 178 │     <parent link="wrist_1_link"/>
 179 │     <child link="wrist_2_link"/>
 180 │     <origin rpy="0.0 0.0 0.0" xyz="0.0 0.127 0.0"/>
 181 │     <axis xyz="0 0 1"/>
 182 │     <limit effort="28.0" lower="-3.141592653589793" upper="3.141592653589793" velocity="6.28"/>
 183 │     <dynamics damping="0.0" friction="0.0"/>
 184 │   </joint>
```

**model/ur5e.urdf:205-212**

```xml
 205 │   <joint name="wrist_3_joint" type="revolute">
 206 │     <parent link="wrist_2_link"/>
 207 │     <child link="wrist_3_link"/>
 208 │     <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.1"/>
 209 │     <axis xyz="0 1 0"/>
 210 │     <limit effort="28.0" lower="-3.141592653589793" upper="3.141592653589793" velocity="6.28"/>
 211 │     <dynamics damping="0.0" friction="0.0"/>
 212 │   </joint>
```

ikpy 链上六个活动关节的 `bounds` 与 URDF `<limit lower upper>` 一致。MuJoCo 侧是上一节加载的 `jnt_range`，另外给出执行器 `ctrlrange`，因为设定点还会被这里截断。

| joint                 | URDF `<limit>`     | 文件位置              | MJCF `jnt_range` | MJCF `ctrlrange` |
| --------------------- | ------------------ | --------------------- | ---------------- | ---------------- |
| `shoulder_pan_joint`  | ±3.0               | `model/ur5e.urdf:70`  | ±6.28319         | ±6.2831          |
| `shoulder_lift_joint` | ±3.141592653589793 | `model/ur5e.urdf:98`  | ±6.28319         | ±6.2831          |
| `elbow_joint`         | ±3.141592653589793 | `model/ur5e.urdf:126` | ±3.1415          | ±3.1415          |
| `wrist_1_joint`       | ±3.141592653589793 | `model/ur5e.urdf:154` | ±6.28319         | ±6.2831          |
| `wrist_2_joint`       | ±3.141592653589793 | `model/ur5e.urdf:182` | ±6.28319         | ±6.2831          |
| `wrist_3_joint`       | ±3.141592653589793 | `model/ur5e.urdf:210` | ±6.28319         | ±6.2831          |

`3.141592653589793` 就是 π。π − 3.1415 = `9.265×10⁻⁵` rad。

`MuJoCo_project.py:114-116` 的 ikpy 在 URDF 的 `bounds` 里求解；仿真器按 MJCF 的 `jnt_range` 约束关节，并按 `ctrlrange` 限制 `data.ctrl`。后果：

- `shoulder_pan`（URDF ±3.0）、`shoulder_lift` 和三个 wrist（URDF ±π）的 URDF 区间落在 MJCF 区间内部。MJCF 允许、而 ikpy 不会返回的角度，例如 pan 的 4 rad，不会出现在这些调用的解里。
- elbow 相反：URDF 的 ±π 比 MJCF 的 ±3.1415 每侧宽 `9.265×10⁻⁵` rad，elbow 的 `ctrlrange` 也是 ±3.1415。解若顶在 URDF 边界上，写进 `data.ctrl` 时会被收到 ±3.1415，关节本身也不能越过 `jnt_range`。

<a id="sec-minimal-experiments"></a>

## 三个最小实验

**三条实验命令总览（行号为片段内序号 1-3）**

```bash
   1 │ uv run python -c 'import mujoco; m = mujoco.MjModel.from_xml_path("model/universal_robots_ur5e/scene.xml"); print(m.nq, m.nv, m.nu, m.nbody, m.opt.timestep)'
   2 │ uv run python - <<'PY'   # 完整片段见「不一致的 qpos 与 ctrl」小节
   3 │ uv run python MuJoCo_project.py
```

命令都在仓库根目录执行。

### 打印模型维度

```bash
uv run python -c 'import mujoco; m = mujoco.MjModel.from_xml_path("model/universal_robots_ur5e/scene.xml"); print(m.nq, m.nv, m.nu, m.nbody, m.opt.timestep)'
```

应该看到：

```text
6 6 6 10 0.002
```

这就是「数值契约表」的前五项。

### 不一致的 qpos 与 ctrl

`MuJoCo_project.py:32-34` 把 `qpos` 设成 `[0, 0, 2.65, -1.3, 1.55, 0]`，把 `ctrl` 设成 `[-1.57, -1.34, 2.65, -1.3, 1.55, 0]`。前两轴差 1.57 rad 和 1.34 rad，后四轴相同。下面先走这一步，再走 `ctrl` 抄 `qpos` 的对照步。

```bash
uv run python << 'PY'
import numpy as np
import mujoco

model = mujoco.MjModel.from_xml_path("model/universal_robots_ur5e/scene.xml")
q = np.array([0.0, 0.0, 2.65, -1.3, 1.55, 0.0])

data = mujoco.MjData(model)
data.qpos[:6] = q
data.ctrl[:6] = [-1.57, -1.34, 2.65, -1.3, 1.55, 0.0]
mujoco.mj_step(model, data)
print("mismatch time", data.time)
print("mismatch force", data.actuator_force)
print("mismatch qvel", data.qvel)

data = mujoco.MjData(model)
data.qpos[:6] = q
data.ctrl[:6] = q
mujoco.mj_step(model, data)
print("matched force", data.actuator_force)
print("matched qvel", data.qvel)
PY
```

应该看到：

- `mismatch time` 是 0.002。
- `mismatch force` 是 `[-150, -150, 0, 0, 0, 0]`。前两轴打满 `forcerange` 的下界。未截断值分别是 `2000×(-1.57)=-3140` 和 `2000×(-1.34)=-2680`。后四轴在这一步求力矩时 `ctrl=q` 且 `qvel` 仍为 0，所以力矩是 0。
- `mismatch qvel` 约为 `[-0.704679, -1.150792, 0.783114, 0.512595, 0.015413, -0.142630]`。一个 0.002 s 的步里前两轴已经到约 −0.70 rad/s 和 −1.15 rad/s。
- 对照步的 `matched force` 全是 0，`matched qvel` 约为 `[-0.347169, -0.629834, 0.765168, 0.475089, 0.013192, -0.116231]`。重力在两步里都会产生速度；不一致那一步多出来的，是前两轴被 ±150 N·m 打满之后的速度差。后四轴两步接近，因为它们的 `ctrl` 本来就等于 `qpos`。

`ur5e()` 随后在循环里持续 `mj_step`（`MuJoCo_project.py:36-38`），这个初始误差会一直被伺服拉。

### 打开场景

```bash
uv run python MuJoCo_project.py
```

`MuJoCo_project.py:558-560` 调用 `ur5e()`，加载的就是 `scene.xml`（`:29`）。判据：

- 原点有三根轴：X 红、Y 绿、Z 蓝，长度 0.3 m，来自 `world_frame` 的 `fromto`（`scene.xml:24-30`）。`contype=0`，与机器人不相撞。
- 有一个绿色半透明球，球心 `(-0.13, 0.5, 0.1)`，半径 0.02，rgba 加载值 `[0, 1, 0, 0.5]`（`scene.xml:33-34`）。
- 手臂会离开全零姿态并运动，因为 `ur5e()` 用的是上一节那组不一致的 `qpos` / `ctrl`。
- 这个绿球的 y 是 0.5。`point()` 的目标位置是 `[-0.13, 0.6, 0.1]`（`MuJoCo_project.py:126`），`JointSpaceControl` 用了同一组数（`:195`）。球和这两处 IK 目标只在 y 上差 0.1 m，不是同一个点。本次命令也没有调用 `point()`。
- 胶囊碰撞体是 group 3，默认 `MjvOption.geomgroup` 不显示它们。窗口里的臂体是 group 2 的 mesh。
- 本机 Wayland 会话下，终端可能打印 `OpenGL error 0x502`。含义见下一节，窗口里的轴和绿球仍然按上面的 geom 绘制。

<a id="sec-pitfalls"></a>

## 常见坑

**model/universal_robots_ur5e/ur5e.xml:3-23**

```xml
   3 │
   4 │   <option integrator="implicitfast"/>
   5 │
   6 │   <default>
   7 │     <default class="ur5e">
   8 │       <material specular="0.5" shininess="0.25"/>
   9 │       <joint axis="0 1 0" range="-6.28319 6.28319" armature="0.1"/>
  10 │       <general gaintype="fixed" biastype="affine" ctrlrange="-6.2831 6.2831" gainprm="2000" biasprm="0 -2000 -400"
  11 │         forcerange="-150 150"/>
  12 │       <default class="size3">
  13 │         <default class="size3_limited">
  14 │           <joint range="-3.1415 3.1415"/>
  15 │           <general ctrlrange="-3.1415 3.1415"/>
  16 │         </default>
  17 │       </default>
  18 │       <default class="size1">
  19 │         <general gainprm="500" biasprm="0 -500 -100" forcerange="-28 28"/>
  20 │       </default>
  21 │       <default class="visual">
  22 │         <geom type="mesh" contype="0" conaffinity="0" group="2"/>
  23 │       </default>
```

**model/universal_robots_ur5e/ur5e.xml:134-136**

```xml
 134 │   <keyframe>
 135 │     <key name="home" qpos="-1.5708 -1.5708 1.5708 -1.5708 -1.5708 0" ctrl="-1.5708 -1.5708 1.5708 -1.5708 -1.5708 0"/>
 136 │   </keyframe>
```

| 坑                                         | 本仓库里的事实                                                                                                                                                                                                                                                                                                                              |
| ------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| include / meshdir 相对路径                 | `<include file="ur5e.xml"/>`（`scene.xml:2`）相对 `scene.xml` 所在目录，与 cwd 无关。`meshdir="assets"`（`ur5e.xml:3`）相对顶层 XML 所在目录。二者同目录时网格在 `model/universal_robots_ur5e/assets/`。顶层文件若换到没有 `assets/` 的目录，obj 会找不到。`from_xml_path` 的相对路径参数仍然相对 cwd。                                     |
| 可视 geom 的 `contype` / `conaffinity` = 0 | `visual` class 写了这两项为 0（`ur5e.xml:22`），加载后 mesh 不参与接触。`collision` 加载后是 1/1。`world_frame` 与 `target` 在 `scene.xml:25-34` 也显式写了 0，只做标注。                                                                                                                                                                   |
| `ctrl` 是设定点，力矩另算                  | `<general>` 用 `gainprm` / `biasprm` 把 `ctrl` 和 `q`、`qvel` 合成力矩，再按 `forcerange` 截断。`MuJoCo_project.py:32-34` 前两轴的角度差会在第一步打出 −150 N·m；−1.57 是角度差，不是这一步的力矩。                                                                                                                                         |
| `armature` 与 `implicitfast`               | `integrator="implicitfast"` 在 `ur5e.xml:4`，加载枚举值 3。`armature="0.1"` 在 `:9`。六个 `dof_damping` 都是 0，速度项在执行器 `biasprm` 的第三个数里，不在关节 damping 里。把 `dof_armature` 改成 0 后，广义质量矩阵的六个对角元各减小 0.1：`armature` 是加在对角线上的惯量。                                                              |
| `keyframe home` 同时给 `qpos` 与 `ctrl`    | `ur5e.xml:135` 把同一组六个数写入 `qpos` 和 `ctrl`。`MjData()` 的初值仍是全 0，`model.qpos0` 也是全 0；要 `mj_resetDataKeyframe(model, data, 0)` 才两者一起变成 home。只把 `qpos` 设成 home、`ctrl` 留 0，再 `mj_forward`，`actuator_force` 是 `[150, 150, -150, 28, 28, 0]`，前五个执行器打满 `forcerange`。位置伺服的关键帧必须带设定点。 |
| Wayland 下 OpenGL `0x502`                  | 本机是 GNOME on Wayland（`README.md:19`）。`mujoco.viewer` 建 UI 上下文时会打印 `WARNING: OpenGL error 0x502 in or before mjr_makeContext`（`README.md:98`，日志样例 `MUJOCO_LOG.TXT:1`）。`scripts/verify_window_render.py:3-4` 用这个警告当探针：它不表示这一帧没有画出来。                                                               |

<a id="sec-next-steps"></a>

## 下一步

**MuJoCo_project.py:558-565**

```python
 558 │ if __name__ == "__main__":
 559 │     # car()
 560 │     ur5e()
 561 │     # ur5e_vision()
 562 │     # point()
 563 │     # JointSpaceControl()
 564 │     # MoveCurve()
 565 │     # MoveEllipse()
```

- [ ] 关节空间控制：`JointSpaceControl`（`MuJoCo_project.py:183`）已经把路径点写进 `data.ctrl`。接着要处理的是 `K_p` 下的重力静差，以及 URDF `bounds` 与 MJCF `jnt_range` / `ctrlrange` 两套限位。
- [ ] 笛卡尔控制：`MoveCurve`（`MuJoCo_project.py:292`）和 `MoveEllipse`（`:451`）把 ikpy 的 `ee_link` 姿态送给求解器。拿 `data.site_xmat` 做姿态误差之前，先乘上「MJCF 与 URDF 的对应关系」里的固定矩阵 `M`。位置可以直接用 `data.site_xpos`。
- [ ] 夹爪装配：`model/robotiq_2f85/2f85.xml` 是单独的模型，指尖 site 为 `pinch`（`model/robotiq_2f85/2f85.xml:67`）。当前 `scene.xml` 没有 include 它。臂上的安装点是 `attachment_site`（`ur5e.xml:114`）。
