# 调试记录

从原始 mujoco-ur5-model 到可动版 UR5 + 夹爪仿真过程中遇到的所有关键问题和解决方法。

---

## 1. macOS 上启动 MuJoCo viewer

**问题**：`mujoco.viewer.launch_passive()` 要求 `mjpython`，但 `mjpython` 把 Python 跑在后台线程，macOS GLFW 要求主线程。

**解决**：绕过内置 viewer，直接用 GLFW 创建窗口 + MuJoCo 底层 `mjr_render` 渲染。

---

## 2. 模型改造：quaternion → 关节

**背景**：原始 UR5 用 body quaternion 硬编码位姿，臂不可动。

**实现**：
- 从 README 提取 home quaternion（所有关节 0° 时的姿态）
- 在 home quat 基础上添加 `<joint type="hinge">`
- 验证：`body_quat = q_home × q_rot(θ)` 等价于 MuJoCo 的 `final = q_joint(θ) × q_home`

---

## 3. 位置执行器抖动 / 失稳

### 阶段 1：直接改 qpos

**现象**：按键改 `data.qpos` + 调 `mj_forward` → 物理引擎和 actuator 打架，臂抖动。

**修复**：改用 `data.ctrl` 设置执行器目标，不直接改 qpos。

### 阶段 2：高 kp 数值不稳定

**现象**：`kp=2000` → `QACC at DOF 3 unstable` 报错。

**原因**：position actuator 扭矩 = `kp × error`，2000 × 小误差 = 大扭矩，2ms 积分步长下发散。

**修复**：降 kp 到 80。

### 阶段 3：低 kp 抬不动臂

**现象**：`kp=80` → J2（shoulder_lift）承重关节按不动。80 × 0.05 = 4Nm，不够抗重力。

**尝试**：velocity actuator + Python P 控制，仍振荡。

### 最终方案：position actuator + forcerange

```xml
<position joint='shoulder_lift' kp="300" kv="60" forcerange="-400 400"/>
```

`forcerange` 是关键——无论 kp 算多大，扭矩硬上限钳制输出。等同真实伺服电机。既能抗重力又防发散。

### 控制架构迭代总结

| 版本 | 方案 | 结果 |
|------|------|------|
| 1 | 直接改 qpos + mj_forward | 抖动 |
| 2 | position kp=1e5 | 剧烈振荡 |
| 3 | position kp=450, kv=80 | 轻微振荡 |
| 4 | position kp=80 | J2/J3 抬不动 |
| 5 | position kp=2000 | QACC 爆炸 |
| 6 | velocity + Python P | 重力下垂振荡 |
| 7 | position kp=300, kv=60 + forcerange | ✅ 稳定 |

---

## 4. 按键 J2（3/4）无响应

**现象**：`3` 和 `4` 键不工作。

**排查**：`test_keys.py` 验明 GLFW key code 正常，`KEY_3=51, KEY_4=52`。

**原因**：kp=80，臂自重压住 shoulder_lift 关节。不是按键问题，是扭矩不足。

**解决**：提升 kp 到 300（见问题 3 最终方案），同时将 J2 键位改为 W/S（数字键在终端可能被拦截）。

---

## 5. 控制延迟

**现象**：按键到画面有延迟。

**原因**：mj_step 曾嵌入 GLFW 渲染循环，帧率卡住物理。

**解决**：物理线程独立 500Hz 运行，渲染只读不写。

---

## 6. 夹爪整合（6 轮迭代）

### 尝试 1：hinge joint on inner knuckle + equality constraint

在现有 knuckle body 上直接加 hinge。**结果**：knuckle 旋转立即与其他 mesh 碰撞，动弹不得。

### 尝试 2：hinge joint + 改增益

加 damping、调 kp/kv。**结果**：碰撞依然存在，物理引擎强制推回零位。

### 尝试 3：slide joint on knuckle

换 slide joint 沿 X 轴平移。**结果**：仍被周围 mesh 卡住。

### 尝试 4：参考 PaulDanielML 模型

他们用了同样的 knuckle 结构 + hinge，但右指靠 `motor` actuator（`gear=101`，101Nm 硬碾碰撞），左指用 equality 锁死。这是我们不想要的暴力方案。

### 尝试 5：圆柱体代理夹爪

在 `robotiq_85_base_link` 下添加两个独立 `<body>`，各配 slide joint + cylinder geom。**结果**：夹爪能开合，但与原始 knuckle mesh 视觉重叠。

### 最终方案：代理夹爪 + 隐藏原 mesh

- 保留 palm mesh 可见
- 隐藏 inner/outer knuckle 和 finger meshes（`rgba="0 0 0 0"`, `contype="0"`）
- 两个 φ8mm × 40mm 圆柱体作为功能手指，slide joint ±15mm
- kp=50, kv=5, forcerange=±2N — 轻量、无碰撞、无抖动

---

## 7. 竞态条件（Z/X 键无效）

**现象**：按键设 qpos 但物理线程 `mj_step` 同时写入，覆盖了变化。

**解决**：用 `threading.Lock` 保护 `data` 访问，key_cb 和 physics_loop 互斥。

---

## 8. 物理碰撞 vs 高精度定位的冲突

### 问题

在 UR5 + 2F-85 模型上，**物理碰撞检测**和**末端高精度定位**无法兼得。

- **物理碰撞**需要 `mj_step`（仿真器运行约束求解）→ 手臂受重力作用会下垂
- **高精度定位**需要 `mj_forward`（运动学模式）→ 直接设 qpos 精确定位，但不运行碰撞检测

### 原因

UR5 肩部到指尖臂长约 0.8m。重力在肩关节产生约 50 Nm 扭矩。position actuator 用 `kp*(ctrl-qpos)` 计算力——误差为 0 时力为 0，重力拉偏后产生误差才有力。这是一个**有静差的弹簧系统**。kp 越大稳态误差越小，但 MuJoCo 的数值积分在高增益下不稳定（kp=50000 时系统发散）。

即使 kp=5000、forcerange=500 Nm，200 步后末端漂移仍达 34cm。这不是参数问题，是架构问题——**position actuator 没有重力补偿项。**

### 当前方案

采用混合架构：手臂用 kinematic 模式（`qpos` 直接设置，精度 <5mm），夹爪和方块走物理（`mj_step`，有碰撞）。穿模问题通过 **reward 惩罚** 解决——检测手指进入平台区域时扣分，RL 学习回避。

### 终极方案（未实现）

在 `mj_step` 前用 `mj_rne` 计算重力补偿力矩，加到 `data.qfrc_applied` 上，这样手臂在物理模式下也能保持位置。复杂度高，留给后续迭代。

### 穿模惩罚的尝试

**尝试**：在 reward 中加入平台穿透检查——当 flange 或 pinch 进入平台 bounding box 时扣分。

**结果**：kinematic 模式下法兰必经平台内部（因为方块在平台上方，手臂穿过平台才能到达），惩罚恒为真，RL 学到的是"靠近方块就扣分"的反向策略。

**结论**：kinematic 模式下穿模惩罚不可行。物理碰撞和精确定位是互斥需求（见第 8 节）。当前接受视觉穿模，专注抓取任务本身。
