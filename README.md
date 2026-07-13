# MuJoCo UR5 Articulated Simulation

基于 [roboticsleeds/mujoco-ur5-model](https://github.com/roboticsleeds/mujoco-ur5-model) 改造，在 macOS + MuJoCo 3.10 上运行。UR5 六轴全可动 + Ridgeback 移动底座 + 圆柱代理夹爪。

## 快速开始

```bash
./work/venv/bin/python3 run_ur5_articulated.py
```

## 控制

| 按键 | 功能 |
|------|------|
| G / H | 底座 X 平移（按住移动，松手停） |
| T / Y | 底座 Y 平移 |
| R / F | 底座 Z 旋转 |
| **1 / 2** | J1 shoulder_pan |
| **W / S** | J2 shoulder_lift |
| **5 / 6** | J3 elbow |
| **7 / 8** | J4 wrist_1 |
| **9 / 0** | J5 wrist_2 |
| **- / =** | J6 wrist_3 |
| **Z** | 夹爪合拢 |
| **X** | 夹爪张开 |
| ← → ↑ ↓ | 旋转/俯仰视角 |
| 滚轮 | 缩放 |
| ESC | 退出 |

## 项目结构

```
├── run_ur5_articulated.py          # 启动脚本（11 DOF，含夹爪）
├── run_ur5.py                      # 原始固定臂版（仅底座 3 DOF）
├── ur5_articulated.xml             # 顶层模型
├── example.xml                     # 原始模型（臂固定不动）
├── include/
│   ├── robot.xml                   # 原始：body quaternion 硬编码，臂不可动
│   ├── robot_articulated.xml       # 改造版：hinge UR5 + slide 夹爪
│   ├── robot_assets.xml            # STL 网格引用
│   ├── actuators.xml               # 原始执行器（3 个 velocity）
│   ├── actuators_articulated.xml   # 改造版（9 UR5 + 2 夹爪，position + forcerange）
│   └── table_assets.xml            # 地面材质
├── stl_files/                      # UR5 3D 网格
└── work/venv/                      # Python 虚拟环境
```

## 模型参数

### 自由度（11 DOF）

| 索引 | 关节 | 类型 | 范围 | 说明 |
|------|------|------|------|------|
| 0 | base_x | slide | ±100 m | 底座 X |
| 1 | base_y | slide | ±100 m | 底座 Y |
| 2 | base_theta | hinge | ±100 rad | 底座旋转 |
| 3 | shoulder_pan | hinge | ±360° | UR5 J1 |
| 4 | shoulder_lift | hinge | ±360° | UR5 J2 |
| 5 | elbow | hinge | ±360° | UR5 J3 |
| 6 | wrist_1 | hinge | ±360° | UR5 J4 |
| 7 | wrist_2 | hinge | ±360° | UR5 J5 |
| 8 | wrist_3 | hinge | ±360° | UR5 J6 |
| 9 | grp_L | slide | ±0.015 m | 左指 |
| 10 | grp_R | slide | ±0.015 m | 右指 |

### 执行器参数（position actuator + forcerange）

| 关节 | kp | kv | 扭矩上限 |
|------|----|-----|----------|
| shoulder_pan | 300 | 60 | ±200 Nm |
| shoulder_lift | 300 | 60 | ±400 Nm |
| elbow | 300 | 60 | ±300 Nm |
| wrist_1 | 200 | 40 | ±100 Nm |
| wrist_2 | 200 | 40 | ±100 Nm |
| wrist_3 | 200 | 40 | ±100 Nm |
| grp_L / grp_R | 50 | 5 | ±2 N |

### 初始臂姿

| 关节 | 角度 |
|------|------|
| shoulder_pan | -90° |
| shoulder_lift | -175° |
| elbow | -5° |
| wrist_1 | -180° |
| wrist_2 | -90° |
| wrist_3 | -180° |

## 技术方案

### 控制架构

```
键盘事件 → data.ctrl[i] = target
               ↓
MuJoCo position actuator:  torque = kp·(ctrl − qpos) − kv·qvel
               ↓
          forcerange 钳制
               ↓
           关节运动
```

### 夹爪实现

- 原 Robotiq 网格无内置关节 → 用两个独立 `<body>` + cylinder geom 作为代理手指
- slide joint 沿 X 轴 ±15mm 行程
- 原 knuckle/finger mesh 设为透明（`rgba="0 0 0 0"`），只保留 palm 底座可见
- 低增益控制（kp=50, kv=5, 力上限 2N），无碰撞无抖动

### 原始模型 → 可动版改动

1. UR5 的 6 个 body quaternion 拆解为 home quat + hinge joint
2. 用 position actuator + forcerange（扭矩上限伺服）替代固定姿态
3. 底座保留 Ridgeback 3 自由度移动平台
4. 添加圆柱代理夹爪

## 许可证

基于 [mujoco-ur5-model](https://github.com/roboticsleeds/mujoco-ur5-model)，GNU GPL v3.0。
