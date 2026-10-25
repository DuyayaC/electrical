# VMC

这是单腿二维虚拟模型控制映射模块。它只负责把腿端虚拟力和虚拟俯仰力矩通过 Jacobian 转换为两个关节力矩，不依赖机器人左右侧定义、传感器、CAN 或电机驱动。

## 本文件夹的文件组织

整合结果（按"对外 / 对内"重排后的 `vmc_merged.h` / `vmc_merged.c`）已集中到
**`../Chassis/`**，与 FiveBar、WheelLegLQR 的整合结果组成一套库；本文件夹现在只保留
**原始文件** `vmc.h` / `vmc.c` 作对照，内容未改动。

整合版的文件说明、编入工程的方法与调用顺序见 `../Chassis/README.md`。
**新旧两组文件不要同时编译**（会重复定义 `VMC_ConvertWrenchToJointTorque`）。

### 整合版对外接口（`../Chassis/vmc_merged.h` 导出）

| 接口 | 作用 |
| --- | --- |
| `VMC_ConvertWrenchToJointTorque()` | 单腿虚拟力 → 两关节力矩，模块的唯一入口 |

### 整合版对内实现（`.c` 内，头文件不导出）

- `VMC_IS_FINITE_VALUE` —— 有限值判定宏，合并了原先对 4 个 Jacobian 元素、2 个虚拟力分别书写的同义判定

## 数学关系

```text
tau = J^T [F, Tp]
```

其中 `J[2][2]` 的行分别对应腿长方向和腿轴角方向，列对应两个驱动关节；`F` 单位为 `N`，`Tp` 单位为 `N·m`，输出关节力矩单位为 `N·m`。

## 接口与责任

`VMC_ConvertWrenchToJointTorque()` 每次只处理一条腿。左右腿由上层分别调用两次。调用方应在进入 VMC 前使用 FiveBar 的 `jacobian_valid` 或等价判定确认 Jacobian 不接近奇异位形；VMC 本身只负责有限值检查和矩阵转置乘法。

输入为空或包含非有限值时返回 `0` 并将两个输出清零，成功返回 `1`。

```c
float J[2][2];
float tau[2];

if (VMC_ConvertWrenchToJointTorque(J, force_N, pitch_torque_Nm, tau) != 0u)
{
    /* tau[0] and tau[1] are the two joint torques. */
}
```
