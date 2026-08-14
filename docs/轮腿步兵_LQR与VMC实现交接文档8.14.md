# 轮腿机器人 LQR 与 VMC 实现交接文档

版本：2026-08-14  
适用工程：FINAL-2026  
参考资料：《[轮腿式平衡机器人控制_陈阳(1).pdf](轮腿式平衡机器人控制_陈阳(1).pdf)》图 16  
交接范围：五连杆运动学、状态估计、LQR、腿长增益调度、VMC，以及这些模块接入整机控制任务所需的接口说明。

## 1. 先看结论

当前仓库已经完成的是“控制算法计算模块”和对应的主机数学测试，不是完整的机器人图 16 闭环。

已经存在并可被其他代码调用的模块有：

- 五连杆位置、速度、Jacobian 和 Jacobian 奇异性检查；
- 基于五连杆状态、轮速和机体俯仰状态的六维状态估计；
- 两输入、六状态 LQR 矩阵计算；
- 根据腿长 L0 对离线增益表做线性插值；
- 基于虚功原理 \(\boldsymbol{\tau}_q = J^\mathsf{T}[F,T_p]^\mathsf{T}\) 的双腿 VMC；
- 对上述模块的 GCC 主机编译和断言测试。

尚未完成的是“把模块接到真实传感器、控制任务和六个电机上”：当前 CHASSIS_TASK() 仍执行旧四麦轮底盘控制，仓库中没有调用 WheelLeg_* 或 FiveBar_* 接口的业务代码。因此当前交付应准确表述为：

> LQR/VMC 计算模块已实现，图 16 的整机控制链尚未完成实机接入和验证。

不能表述为“机器人已经完成 LQR/VMC 平衡控制”。

## 2. 当前代码位置和工程状态

### 2.1 算法文件

| 模块 | 文件 | 当前作用 |
|---|---|---|
| 五连杆运动学 | [five_bar.h](../FINAL-2026/User/WheelLeg/five_bar.h)、five_bar.c | 由两个主动关节角计算腿长、腿轴角、速度和 Jacobian |
| 公共类型 | [wheel_leg_types.h](../FINAL-2026/User/WheelLeg/wheel_leg_types.h)、wheel_leg_control_types.h | 定义左右轮、四个关节、估计器输入、运动学状态和 LQR 状态 |
| 状态估计 | [wheel_leg_state_estimator.h](../FINAL-2026/User/WheelLeg/wheel_leg_state_estimator.h)、wheel_leg_state_estimator.c | 组合腿部运动学、轮输出轴速度和机体俯仰信息 |
| LQR | [wheel_leg_lqr.h](../FINAL-2026/User/WheelLeg/wheel_leg_lqr.h)、wheel_leg_lqr.c | 计算 \(\mathbf{u}=K(\mathbf{x}_{\mathrm{ref}}-\mathbf{x})\)，其中 \(\mathbf{u}=[T,T_p]^\mathsf{T}\) |
| LQR 调度 | [wheel_leg_lqr_schedule.h](../FINAL-2026/User/WheelLeg/wheel_leg_lqr_schedule.h)、wheel_leg_lqr_schedule.c | 对给定的 K(L0) 表做运行时线性插值 |
| 已生成的增益表 | [wheel_leg_lqr_schedule_generated.h](../FINAL-2026/User/WheelLeg/wheel_leg_lqr_schedule_generated.h) | 11 个腿长节点的静态增益表，使用前仍需验证模型和参数 |
| VMC | [wheel_leg_vmc.h](../FINAL-2026/User/WheelLeg/wheel_leg_vmc.h)、wheel_leg_vmc.c | 将每条腿的虚拟力/力矩转换为两个关节力矩 |

这些 .c 文件已经加入 Keil 工程的 User/WheelLeg 分组，工程文件证据见 [FINAL-2026.uvprojx](../FINAL-2026/MDK-ARM/FINAL-2026.uvprojx)。但“加入工程”只代表参与工程编译配置，不代表已经被任务调用。

### 2.2 当前顶层任务不是轮腿控制任务

[Task/chassis_task.c](../FINAL-2026/Task/chassis_task.c) 当前仍然：

- 读取旧四麦轮电机反馈；
- 根据云台编码器计算底盘坐标角；
- 将速度指令做四麦轮逆运动学；
- 使用四路旧底盘 PID；
- 通过 CAN_cmd_chassis() 发送四路旧底盘命令。

虽然该任务的周期是 1 ms，但它没有执行以下任何调用：

    FiveBar_Solve(...)
    WheelLeg_StateEstimator_Update(...)
    WheelLeg_LQR_Calculate(...)
    WheelLeg_LQR_ScheduleInterpolate(...)
    WheelLeg_VMC_Calculate(...)

因此后续接手者应把 CHASSIS_TASK() 作为图 16 的顶层周期调度入口，或者明确新建独立轮腿任务；不能只在旧四麦轮控制逻辑后面追加一个 LQR 调用。

## 3. 对照论文图 16 的完成情况

论文图 16 的控制链可以按以下数据流理解：

    传感器反馈
        ↓
    估计器 ───────────────→ x_hat、L0_hat、psi_hat、theta_l/theta_r、gamma_hat、L0/dL0
        ↓                         ↓
    参考输入 ───────→ LQR K(L0) ─→ [T, Tp]^T
                                  ├─ 转向 PD / 双腿协调 PD → 左右轮力矩、左右腿绕轴力矩
                                  └─ 腿长 PID+前馈 / 横滚补偿 → 左右腿虚拟力 F_l/F_r
                                                             ↓
                                      VMC：J^T[F_i, Tp_i]^T → 四个关节力矩
                                                             ↓
                                   2 个轮电机力矩 + 4 个关节电机力矩
                                                             ↓
                                        电机内置力矩闭环 / 机器人

| 图 16 方框或信号 | 当前代码状态 | 结论 |
|---|---|---|
| 估计器中的五连杆几何 | FiveBar_Solve() | 已实现数学模块 |
| L0、腿轴角及其速度 | LegKinematicState | 已实现；左右腿分别计算 |
| J | LegKinematicState.J[2][2] | 已实现；使用中心差分 |
| \(\mathbf{x}=[\theta,\dot\theta,x_b,\dot x_b,\phi,\dot\phi]^\mathsf{T}\) | WheelLegEstimate.x[6] | 已实现基础版本 |
| 轮速组合得到 dxb | WheelLeg_StateEstimator_Update() | 已实现；依赖轮速、phi_bc 角速度和俯仰角速度 |
| K(L0) | WheelLeg_LQR_ScheduleInterpolate() + 生成表 | 接口和表插值已实现；参数有效性未完成验证 |
| \(\mathbf{u}=K(L_0)(\mathbf{x}_{\mathrm{ref}}-\mathbf{x})\) | WheelLeg_LQR_Calculate() | 已实现矩阵运算 |
| T 到左右轮力矩 | 无业务调用、无转向 PD | 未实现 |
| \(T_p\) 到左右腿绕中心轴力矩 | 无双腿协调 PD | 未实现 |
| \([L_d^l,L_d^r]\) 到 \([F_l,F_r]\) | 无腿长 PID+前馈 | 未实现 |
| \(\gamma_d-\hat\gamma\) 横滚补偿 | 无 gamma 控制器接口 | 未实现 |
| VMC \(J^\mathsf{T}[F,T_p]^\mathsf{T}\) | WheelLeg_VMC_Calculate() | 已实现数学模块，尚未接到电机输出 |
| 4 个关节电机力矩发送 | 无 DM 电机业务适配/发送层 | 未实现 |
| 2 个轮电机力矩发送 | 现有代码只有旧四麦轮 CAN_cmd_chassis() | 未实现轮腿版接入 |
| 反馈、限幅、超时、故障门控 | 各数学函数有局部输入检查 | 系统级保护未实现 |

论文文字中，\(T\) 是纵向状态反馈产生的驱动轮力矩，转向 PD 输出以相反符号叠加到左右轮力矩；\(T_p\) 是纵向状态反馈产生的俯仰/腿轴相关力矩，双腿协调 PD 输出以相反符号叠加到左右腿力矩；腿长控制和横滚补偿共同产生左右腿沿腿方向的虚拟力。最终才将每条腿的 \([F_i,T_{p,i}]^\mathsf{T}\) 交给 VMC。

## 4. 已完成内容详述

### 4.1 五连杆位置与速度解算

FiveBar_Solve() 当前完成的计算为：

1. 根据 q1_rad、q4_rad 和四根连杆长度求五连杆末端位置；
2. 得到腿长 L0_m；
3. 得到相对于机体 +Y 方向的腿轴角 leg_axis_body_rad，直立位置约定为 0；
4. 通过位置函数的中心差分得到

$$
J =
\begin{bmatrix}
\dfrac{\partial L_0}{\partial q_1} & \dfrac{\partial L_0}{\partial q_4}\\[4pt]
\dfrac{\partial \theta}{\partial q_1} & \dfrac{\partial \theta}{\partial q_4}
\end{bmatrix}.
$$

5. 用

   $$
   \begin{bmatrix}\dot L_0\\ \dot\theta\end{bmatrix}
   =J\begin{bmatrix}\dot q_1\\ \dot q_4\end{bmatrix}
   $$

   得到 `dL0_m_s` 和 `dleg_axis_body_rad_s`；
6. 计算 detJ，当其绝对值小于 min_abs_detJ 时将 jacobian_valid 置 0，并返回失败。

代码对输入有限性、三角形可解性、腿长范围和 Jacobian 奇异性做了检查。失败时输出状态会被清零或保持无效，调用方不能继续使用该状态做 VMC。

需要注意：

- FiveBar_CadGeometry() 当前只是向结构体写入代码中的固定参数，并不等于已经完成新机器人 CAD 参数确认；
- branch_sign、左右腿镜像关系、关节零位和电机方向仍需要实物确认；
- Jacobian 使用数值中心差分，不是解析 Jacobian；
- 五连杆模块没有自行推导论文中的 phi_bc，该量由上层输入估计器。

### 4.3 LQR 矩阵计算

WheelLeg_LQR_Calculate() 只负责做矩阵乘法：

$$
\mathbf{e}=\mathbf{x}_{\mathrm{ref}}-\mathbf{x},
\qquad
\mathbf{u}=K\mathbf{e}
=\begin{bmatrix}T\\T_p\end{bmatrix}.
$$

该模块不包含：

- 系统矩阵 A/B 的在线建模；
- Q/R 参数配置；
- 参考输入生成；
- 输出限幅；
- 左右轮力矩分配；
- 转向和双腿协调补偿。

因此 LQR 函数的输入增益必须由调用方提供。它不会自动使用生成的 WL_LQR_SCHEDULE，也不会自动根据当前 L0 选取增益。

#### 当前离线生成使用的 Q/R 权重

当前 LQR 增益由 `tools/wheel_leg/solve_lqr_schedule.py` 离线生成。当前使用的
Q/R 权重沿用论文《轮腿式平衡机器人控制》印刷页 653（PDF 第 6 页）的最终选值，
并记录在 [`tools/wheel_leg/cad_model.json`](../tools/wheel_leg/cad_model.json) 中。
对应的代价函数为：

$$
J_{\mathrm{LQR}}
=\int_0^\infty
\left[
(\mathbf{x}_{\mathrm{ref}}-\mathbf{x})^\mathsf{T}Q
(\mathbf{x}_{\mathrm{ref}}-\mathbf{x})
+\mathbf{u}^\mathsf{T}R\mathbf{u}
\right]\,\mathrm{d}t.
$$

状态顺序为
\(\mathbf{x}=[\theta,\dot\theta,x_b,\dot x_b,\phi,\dot\phi]^\mathsf{T}\)，
控制输入为 \(\mathbf{u}=[T,T_p]^\mathsf{T}\)。当前权重为：

$$
Q=\operatorname{diag}(1,\,1,\,500,\,100,\,5000,\,1)
\quad\text{对应}
\quad
[\theta,\dot\theta,x_b,\dot x_b,\phi,\dot\phi],
$$

$$
R=\operatorname{diag}(1,\,0.25)
\quad\text{对应}
\quad
[T,T_p].
$$

因此，当前并不是运行时在线优化 Q/R，而是使用固定 Q/R 在 11 个腿长
\(L_0\) 节点离线计算增益 \(K(L_0)\)，再由固件在相邻节点之间逐元素线性插值。
其中 \(R\) 的第二项当前采用 `0.25`，不是旧记录中的 `0.025`。

> **参数状态：** 当前 Q/R 只是论文值和离线生成的基线，不代表已经针对本项目
> 轮腿机器人的 CAD 参数、传感器噪声、执行器能力和实机响应完成最优调参。后续应
> 根据仿真、支架测试和实机平衡/跟踪效果重新评估并可能修改 Q/R；修改后必须重新
> 生成全部 \(K(L_0)\) 增益节点，并记录参数版本和验证结果。

### 4.4 LQR 腿长调度

WheelLeg_LQR_ScheduleInterpolate() 对调用方提供的 WheelLegLqrSchedule 做线性插值：

- l0_m 小于表格最小值时使用第一个节点；
- l0_m 大于表格最大值时使用最后一个节点；
- 位于两个节点之间时逐元素线性插值；
- 节点数为 1 时直接返回唯一增益；
- 节点非递增、指针为空或增益含非有限值时返回 0。

当前生成表有 11 个 L0 节点，范围约为 0.1042 m 到 0.3672 m。表头明确写有“使用前验证”，原因是 tools/wheel_leg/cad_model.json 中的质量、质心、转动惯量和几何参数目前仍需与最终实机 CAD/实测参数核对。增益表可以作为软件链路和离线算法测试输入，不能直接视为最终实机增益。

### 4.5 VMC

WheelLeg_VMC_Calculate() 对左右腿分别计算：

$$
\begin{bmatrix}\tau_{q1}\\\tau_{q4}\end{bmatrix}
=J^\mathsf{T}
\begin{bmatrix}F\\T_p\end{bmatrix}.
$$

输入数组顺序：

    force_N[0]          = 左腿 F_l
    force_N[1]          = 右腿 F_r
    pitch_torque_Nm[0]  = 左腿 Tp_l
    pitch_torque_Nm[1]  = 右腿 Tp_r

输出数组顺序：

    joint_torque_Nm[0] = 左腿 q1
    joint_torque_Nm[1] = 左腿 q4
    joint_torque_Nm[2] = 右腿 q1
    joint_torque_Nm[3] = 右腿 q4

VMC 在以下任一条件不满足时返回 0 并将四路输出清零：

- 输入指针非空；
- 两侧 valid == 1；
- 两侧 jacobian_valid == 1；
- detJ、输入力/力矩和 Jacobian 元素均为有限值。

当前 VMC 没有做电机减速比、方向、零位、关节力矩上限或 DM 协议量化。它的输出是关节机械输出轴期望力矩，不能直接强制转换为 CAN 原始命令。

### 4.6 六状态基础组合模块（局部实现）

当前代码实现的是图 16 六维状态向量的基础组合与相对位置积分模块，
不应表述为“完整状态估计器已经完成”。该模块依赖调用方提供已经完成
单位、方向和有效性处理的轮速、IMU pitch 以及 `phi_bc` 数据。

`WheelLeg_StateEstimator_Update()` 的输出状态顺序固定为：

$$
\mathbf{x}=
\begin{bmatrix}
\theta & \dot\theta & x_b & \dot x_b & \phi & \dot\phi
\end{bmatrix}^{\mathsf{T}}.
$$

当前模块的具体规则：

- `theta`：左右腿 `leg_axis_body_rad` 平均值，再加机体俯仰角，并做 \([-\pi,\pi]\) 归一化；
- `dtheta`：左右腿腿轴角速度平均值，加机体俯仰角速度；
- `phi`、`dphi`：直接使用输入的机体俯仰角和俯仰角速度；
- 每侧轮地速度先按论文关系计算，再取左右平均作为 \(\dot{x}_b\)；
- `xb` 从 0 开始，按离散积分更新：\(x_b[k+1]=x_b[k]+\dot{x}_b[k]\,\Delta t\)；
- 第一次有效更新时先把估计器初始化到 `xb=0`；
- 任意一侧轮速、姿态、`phi_bc` 或五连杆状态无效，整次更新返回 0，并把 `estimate` 清零。

当前速度公式为：

$$
\dot x_{b,i}
=R\left(\omega_{w,i}+\dot\phi_{bc,i}+\dot\phi\right)
 +L_{0,i}\dot\theta_i\cos\theta_i
 +\dot L_{0,i}\sin\theta_i.
$$

其中 `wheel_output_velocity` 对应 \(\omega_{w,i}\)，`body_pitch_rate` 对应
\(\dot\phi\)，`dphi_bc` 对应 \(\dot\phi_{bc,i}\)。所有角度使用 rad，角速度使用
rad/s，长度使用 m，速度使用 m/s。

该模块目前明确不包含：

- 论文图 16 中的航向角速度 \(\hat\psi\) 估计；
- 横滚角 \(\hat\gamma\) 估计；
- 左右独立的 \(L_{0,l}/L_{0,r}\)、\(\theta_l/\theta_r\) 控制量输出结构；
- `phi_bc` 的真实机械定义、CAD 映射和由机构运动学得到的计算；
- 原始编码器、RPM、IMU 轴向、零位、方向和减速比的转换与校准；
- 完整姿态融合、航向/横滚估计及实机状态验证。

## 5. 当前未完成内容和接手边界

### 5.1 顶层控制链未接入

当前没有轮腿版控制器结构体、参考输入生成器、周期快照、控制模式状态机，也没有在 CHASSIS_TASK() 中按图 16 调度上述模块。接手者需要补齐：

    电机/IMU反馈适配
      → 五连杆状态
      → 状态估计
      → 参考输入
      → LQR与K(L0)调度
      → 辅助环
      → VMC
      → 输出保护
      → 六电机发送

### 5.2 图 16 的三个辅助控制环未实现

论文图 16 中 LQR 不是全部控制器，还需要补充：

1. 转向 PD：使用期望航向角速度 psi_d 与估计航向角速度 psi_hat 的误差，输出左右轮相反方向的力矩补偿；
2. 双腿协调 PD：使用左右腿角度差，输出左右腿相反方向的绕中心轴力矩，防止转向时“劈叉”；
3. 腿长 PID+前馈与横滚补偿：根据左右腿期望长度和实际长度生成 F_l/F_r，再根据 gamma_d-gamma_hat 做左右相反方向补偿。

这些环不能通过“把某个现有 PID 参数直接套过来”视为完成。需要定义输入输出、符号、左右腿镜像规则和饱和策略，并通过低风险台架测试确认方向。

### 5.3 传感器和电机适配未完成

当前算法层明确不接触 CAN ID、编码器原始计数、RPM 或电机原始命令。尚缺：

- C620/M3508 轮电机反馈到输出轴 rad/s 的换算；
- DM 关节电机反馈解包；
- DM 关节电机输出力矩到协议量的映射；
- 六个电机的 CAN ID、零位、方向和离线超时判断；
- 左右腿和左右关节的硬件映射；
- 发送失败、反馈过期、温度异常、CAN 错误和 Bus-Off 处理。

这些内容属于算法模块之上的适配层，不能在 wheel_leg_vmc.c 内通过临时乘一个符号或比例解决。

### 5.4 关键机械和模型参数未最终确认

以下参数会直接影响运动学、状态估计、VMC 或 LQR，当前不能当成最终标定值：

- 五连杆四根杆长、基座宽度、左右镜像分支；
- 四个关节的零位和正方向；
- 轮半径、轮电机减速比和轮输出速度换算；
- \(\phi_{bc}\) 的实际杆件定义及其角速度测量方式；
- 机器人质量、质心位置、腿部质量和转动惯量；
- 轮地纯滚动假设的适用范围；
- LQR 的 \(A(L_0)\)、\(B(L_0)\) 与最终 \(Q/R\)；
- 左右腿虚拟力和关节力矩的物理正方向。

### 5.5 验证状态

已完成的验证：

- tools/wheel_leg/tests/phase2_test.c 可以用 GCC 编译；
- 使用 -std=c99 -Wall -Wextra -Werror 编译算法源文件和测试文件成功；
- 测试覆盖五连杆典型姿态、估计器有效性、LQR 数值、VMC 虚功映射、CAD 几何常量和增益插值。

未完成的验证：

- 当前环境没有安装 pytest，因此不能宣称 Python 测试套件整体通过；
- 尚未完成 Keil/ARMCC 构建、下载和真实 STM32 上电验证；
- 尚未完成传感器轴向、关节零位和电机方向验收；
- 尚未完成支架状态、固定腿长站立、低速运动、转向、横滚补偿和腿长调度实机验收；
- 尚未验证 VMC 的 J^T 结果与论文静力学闭式公式在真实几何下的一致性。

## 6. 接口说明

### 6.1 公共索引和状态顺序

使用以下枚举，不要在业务代码中重新定义左右顺序：

    WL_WHEEL_LEFT  = 0
    WL_WHEEL_RIGHT = 1

    WL_JOINT_LEFT_Q1  = 0
    WL_JOINT_LEFT_Q4  = 1
    WL_JOINT_RIGHT_Q1 = 2
    WL_JOINT_RIGHT_Q4 = 3

单位约定：

| 数据 | 单位 |
|---|---|
| 角度 | rad |
| 角速度 | rad/s |
| 长度 | m |
| 线速度 | m/s |
| 力 | N |
| 力矩 | N·m |
| 时间步长 | s |

算法接口不接受原始编码器计数、RPM、度或 CAN 命令整数。

### 6.2 FiveBarGeometry

    typedef struct
    {
        float l1_m;
        float l2_m;
        float l3_m;
        float l4_m;
        float base_width_m;
        float branch_sign;
        float finite_difference_step_rad;
        float triangle_tolerance_m;
        float min_length_m;
        float max_length_m;
        float min_abs_detJ;
    } FiveBarGeometry;

推荐初始化方式：

    FiveBarGeometry geometry;
    if (FiveBar_CadGeometry(&geometry) == 0u)
    {
        /* 配置失败：禁止输出 */
    }

如果接手者已有经过确认的 CAD 参数，可以自行填充该结构体，但必须同时确认 branch_sign、长度范围和 Jacobian 奇异阈值。FiveBar_CadGeometry() 中的常量不是硬件自动读取结果。

### 6.3 FiveBar_Solve()

    uint8_t FiveBar_Solve(const FiveBarGeometry *geometry,
                          float q1_rad,
                          float q4_rad,
                          float dq1_rad_s,
                          float dq4_rad_s,
                          LegKinematicState *state);

调用者需要提供一侧腿的两个主动关节角和角速度。成功返回 1u，并填充：

    state->L0_m
    state->leg_axis_body_rad
    state->dL0_m_s
    state->dleg_axis_body_rad_s
    state->J[2][2]
    state->detJ
    state->valid
    state->jacobian_valid

失败返回 0u。VMC 前必须同时检查返回值、state->valid 和 state->jacobian_valid。

左右腿应分别调用：

    LegKinematicState leg[WL_WHEEL_COUNT];

    if (FiveBar_Solve(&geometry,
                      left_q1, left_q4, left_dq1, left_dq4,
                      &leg[WL_WHEEL_LEFT]) == 0u ||
        FiveBar_Solve(&geometry,
                      right_q1, right_q4, right_dq1, right_dq4,
                      &leg[WL_WHEEL_RIGHT]) == 0u)
    {
        /* 任一侧失败，整周期禁止关节力矩输出 */
    }

### 6.4 WheelLeg_StateEstimator_Update()

输入结构体：

    typedef struct
    {
        float wheel_output_velocity_rad_s[2];
        float body_pitch_rad;
        float body_pitch_rate_rad_s;
        float phi_bc_rad[2];
        float dphi_bc_rad_s[2];
        uint8_t wheel_velocity_valid[2];
        uint8_t body_attitude_valid;
        uint8_t phi_bc_valid[2];
    } WheelLegEstimatorInput;

参数结构体：

    typedef struct
    {
        float wheel_radius_m;
        float dt_s;
    } WheelLegEstimatorParams;

初始化一次：

    WheelLegStateEstimator estimator;
    WheelLeg_StateEstimator_Reset(&estimator);

控制周期内调用：

    WheelLegEstimate estimate;
    uint8_t estimate_ok = WheelLeg_StateEstimator_Update(
        &estimator,
        &estimator_input,
        &estimator_params,
        leg,
        &estimate);

estimate_ok == 1u 才能使用 estimate.x。estimate.x 顺序必须保持：

    0: theta_rad
    1: dtheta_rad_s
    2: xb_m
    3: dxb_m_s
    4: phi_rad
    5: dphi_rad_s

注意：首次有效调用会将 xb 初始化为 0。进入平衡模式时，建议先完成传感器有效性确认，再将 x_ref[2] 对齐当前 estimate.x[2]，避免使能瞬间因位置误差产生大力矩。

### 6.5 WheelLeg_LQR_ScheduleInterpolate()

增益表类型：

    typedef struct
    {
        uint32_t count;
        const float *l0_m;
        const float (*gain)[2][6];
    } WheelLegLqrSchedule;

调用示例：

    #include "wheel_leg_lqr_schedule_generated.h"

    float gain[WHEEL_LEG_LQR_OUTPUT_COUNT][WHEEL_LEG_LQR_STATE_COUNT];

    if (WheelLeg_LQR_ScheduleInterpolate(&WL_LQR_SCHEDULE,
                                         0.20f,
                                         gain) == 0u)
    {
        /* 增益无效，禁止输出 */
    }

WL_LQR_SCHEDULE 来自生成头文件。该头文件中的数据是静态常量，接入前要确认：

- L0 范围覆盖当前机械姿态；
- 表格来自最终质量和惯量参数；
- 增益符号与实际电机方向一致；
- 实机输出经过合理限幅。

### 6.6 WheelLeg_LQR_Calculate()

    uint8_t WheelLeg_LQR_Calculate(
        const float gain[2][6],
        const float x[6],
        const float x_ref[6],
        float output[2]);

调用示例：

    float x_ref[WHEEL_LEG_LQR_STATE_COUNT] = {
        theta_ref_rad,
        dtheta_ref_rad_s,
        xb_ref_m,
        dxb_ref_m_s,
        phi_ref_rad,
        dphi_ref_rad_s
    };
    float lqr_output[WHEEL_LEG_LQR_OUTPUT_COUNT];

    if (WheelLeg_LQR_Calculate(gain,
                               estimate.x,
                               x_ref,
                               lqr_output) == 0u)
    {
        /* LQR 输入无效，禁止输出 */
    }

    float T = lqr_output[0];
    float Tp = lqr_output[1];

函数内部按 \(\mathbf{x}_{\mathrm{ref}}-\mathbf{x}\) 计算误差。调用者不应再额外取一次负号。\(T\) 和 \(T_p\) 仍然是图 16 的集中式虚拟控制输出，需要经过辅助环分配后才能送给轮电机和 VMC。

### 6.7 WheelLeg_VMC_Calculate()

    uint8_t WheelLeg_VMC_Calculate(
        const LegKinematicState leg_state[2],
        const float force_N[2],
        const float pitch_torque_Nm[2],
        float joint_torque_Nm[4]);

调用示例：

    float force_N[WL_WHEEL_COUNT] = {F_left, F_right};
    float leg_pitch_torque_Nm[WL_WHEEL_COUNT] = {
        Tp_left,
        Tp_right
    };
    float joint_torque_Nm[WL_JOINT_COUNT];

    if (WheelLeg_VMC_Calculate(leg,
                               force_N,
                               leg_pitch_torque_Nm,
                               joint_torque_Nm) == 0u)
    {
        /* VMC 无效，四路关节命令置零 */
    }

输出只表示四个关节的机械输出轴期望力矩。接手者还需要完成：

    关节机械力矩
      → 减速器/转子力矩换算
      → 零位与方向映射
      → DM 协议范围归一化
      → 饱和
      → CAN 发送

## 7. 推荐的整机接入顺序

### 7.1 周期调度顺序

建议以现有 CHASSIS_TASK() 的 1 ms 周期作为顶层入口，业务代码按以下顺序执行：

    /* 伪代码：具体硬件接口由接手者实现 */
    for (;;)
    {
        /* 1. 读取同一时刻的 IMU、轮电机和四个关节电机快照 */
        ReadWheelLegSensorSnapshot(&snapshot);

        /* 2. 原始量转换为 rad/rad/s/m/s，并完成零位和方向映射 */
        ConvertSnapshotToSi(&snapshot, &joint_si, &estimator_input);

        /* 3. 两侧五连杆运动学 */
        uint8_t kinematics_ok =
            FiveBar_Solve(&geometry, joint_si.left_q1_rad,
                          joint_si.left_q4_rad,
                          joint_si.left_dq1_rad_s,
                          joint_si.left_dq4_rad_s,
                          &leg[WL_WHEEL_LEFT]) &&
            FiveBar_Solve(&geometry, joint_si.right_q1_rad,
                          joint_si.right_q4_rad,
                          joint_si.right_dq1_rad_s,
                          joint_si.right_dq4_rad_s,
                          &leg[WL_WHEEL_RIGHT]);

        /* 4. 估计器 */
        uint8_t estimate_ok = kinematics_ok &&
            WheelLeg_StateEstimator_Update(&estimator,
                                           &estimator_input,
                                           &estimator_params,
                                           leg,
                                           &estimate);

        /* 5. 参考输入和 K(L0) */
        BuildWheelLegReference(&input, &estimate, x_ref, &aux_ref);
        float l0_for_schedule =
            0.5f * (leg[WL_WHEEL_LEFT].L0_m +
                    leg[WL_WHEEL_RIGHT].L0_m);
        uint8_t gain_ok = estimate_ok &&
            WheelLeg_LQR_ScheduleInterpolate(&WL_LQR_SCHEDULE,
                                             l0_for_schedule,
                                             gain);

        /* 6. LQR */
        uint8_t lqr_ok = gain_ok &&
            WheelLeg_LQR_Calculate(gain, estimate.x, x_ref, lqr_output);

        /* 7. 图16辅助环：由接手者实现 */
        BuildWheelTorquePair(lqr_output[0], aux_ref.yaw_rate_d,
                             estimate_yaw_rate, &wheel_torque[0],
                             &wheel_torque[1]);
        BuildLegPitchTorquePair(lqr_output[1], leg, &leg_pitch_torque[0],
                                &leg_pitch_torque[1]);
        BuildLegForcePair(aux_ref, estimate, &force_N[0], &force_N[1]);

        /* 8. VMC */
        uint8_t vmc_ok = lqr_ok &&
            WheelLeg_VMC_Calculate(leg, force_N, leg_pitch_torque,
                                   joint_torque);

        /* 9. 统一安全门控和六电机输出 */
        if (!AllFeedbackFresh(&snapshot) || !kinematics_ok ||
            !estimate_ok || !gain_ok || !lqr_ok || !vmc_ok ||
            !ControlEnableIsSafe())
        {
            ZeroWheelAndJointCommands();
        }
        else
        {
            LimitWheelTorque(wheel_torque);
            LimitJointTorque(joint_torque);
            SendWheelTorque(wheel_torque);
            SendJointTorque(joint_torque);
        }

        osDelayUntil(&last_wake_time, 1);
    }

上面是伪代码，具体硬件函数和类型可按现有工程规范补齐。重要的是调用顺序和同一周期快照原则：不能让 LQR 使用新一周期的 IMU，而 VMC 使用上一周期的 Jacobian。

### 7.2 接入时不要直接复用的旧逻辑

轮腿控制接入时，应停用或替换 chassis_task.c 中的：

- chassis_wheel_inverse_resolution()；
- gimbal_coordinate_resolution()；
- 四路旧麦轮速度 PID；
- WHEEL_TO_CORE_DISTANCE、旧 GEAR_RATIO、旧 WHEEL_RADIUS 的四麦轮定义；
- output_map() 中固定乘 819.2f 的旧电流/命令映射；
- 通过云台编码器计算四麦轮底盘坐标角的逻辑。

遥控器解析可以保留，但输出应改成图 16 的参考量，例如 dxb_ref、dphi_ref、psi_d、L_d^l、L_d^r 和 gamma_d，而不是直接生成四个麦轮速度。

## 8. 接手者需要新增的适配接口

下面这些接口不是当前仓库已有接口，名称仅作为推荐边界，接手时可按项目命名规范调整：

    typedef struct
    {
        float q1_rad[WL_WHEEL_COUNT];
        float q4_rad[WL_WHEEL_COUNT];
        float dq1_rad_s[WL_WHEEL_COUNT];
        float dq4_rad_s[WL_WHEEL_COUNT];
        float wheel_output_velocity_rad_s[WL_WHEEL_COUNT];
        float body_pitch_rad;
        float body_pitch_rate_rad_s;
        float phi_bc_rad[WL_WHEEL_COUNT];
        float dphi_bc_rad_s[WL_WHEEL_COUNT];
        uint8_t valid;
    } WheelLegSiSnapshot;

    uint8_t WheelLeg_ReadAndConvertSnapshot(WheelLegSiSnapshot *snapshot);
    uint8_t WheelLeg_SendWheelTorque(const float torque_Nm[WL_WHEEL_COUNT]);
    uint8_t WheelLeg_SendJointTorque(const float torque_Nm[WL_JOINT_COUNT]);

适配层应负责：

- 从 CAN 反馈结构中读取原始编码器、速度、电流和温度；
- 处理电机 ID 到 WL_WHEEL_*/WL_JOINT_* 的映射；
- 处理零位、方向、减速比和单位转换；
- 记录每路反馈时间戳并判断是否过期；
- 将算法输出的 N·m 命令转换成具体电机协议；
- 处理单路发送失败和总线故障。

算法层不应反向依赖 CAN ID 或底层 motor_measure_t，这样才能在主机测试、仿真和不同电机配置之间复用。

## 9. 实机调试建议

必须按风险从低到高推进：

1. 不使能力矩，确认六路反馈 ID、在线状态、方向、零位和单位；
2. 架空机器人，单路小电流验证六个电机正方向和停止命令；
3. 固定机体或支架上验证五连杆 L0/theta/J 与实测位移的一致性；
4. 固定腿长、关闭运动参考，只验证 VMC 小力矩方向；
5. 低增益验证腿长控制和左右腿协调；
6. 固定腿长、零速度、低输出限幅下验证 LQR 平衡；
7. 依次加入低速纵向运动、转向 PD、横滚补偿和 K(L0) 调度；
8. 最后才提高力矩和速度限幅。

至少记录以下图 16 节点，不能只记录最终电机电流：

    q1/q4、dq1/dq4、L0、theta、J、detJ
    IMU pitch/pitch_rate、yaw_rate、roll
    x、x_ref、K(L0)、T、Tp
    转向补偿、双腿协调补偿、F_l/F_r
    四路 VMC 关节力矩
    两路轮力矩、四路最终电机命令
    反馈新鲜度、故障原因和控制模式

## 10. 交付判定

当前版本可以交付为“LQR/VMC 算法模块交接”，满足以下条件：

- 接手者知道每个现有函数的输入输出和数据顺序；
- 接手者知道 WheelLeg_* 当前没有被顶层任务调用；
- 接手者不会把生成的增益表、CAD 常量或 phi_bc 占位当作最终实机参数；
- 接手者可以按照第 7 节把模块接入统一周期控制链；
- 任一运动学、估计器、LQR、VMC 或反馈有效性失败时，系统有明确的零输出路径。

只有完成传感器/电机适配、辅助环、保护、Keil/STM32 构建和实机验收后，才能把项目状态升级为“图 16 控制系统已打通”。
