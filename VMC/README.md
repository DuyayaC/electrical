# VMC

这是单腿二维虚拟模型控制映射模块。它只负责把腿端虚拟力和虚拟俯仰力矩通过 Jacobian 转换为两个关节力矩，不依赖机器人左右侧定义、传感器、CAN 或电机驱动。

## 数学关系

```text
tau = J^T [F, Tp]
```

其中 `J[2][2]` 的行分别对应腿长方向和腿轴角方向，列对应两个驱动关节；`F` 单位为 `N`，`Tp` 单位为 `N·m`，输出关节力矩单位为 `N·m`。

## 接口与责任

`VMC_Calculate2D()` 每次只处理一条腿。左右腿由上层分别调用两次。调用方应在进入 VMC 前使用 FiveBar 的 `jacobian_valid` 或等价判定确认 Jacobian 不接近奇异位形；VMC 本身只负责有限值检查和矩阵转置乘法。

输入为空或包含非有限值时返回 `0` 并将两个输出清零，成功返回 `1`。

```c
float J[2][2];
float tau[2];

if (VMC_Calculate2D(J, force_N, pitch_torque_Nm, tau) != 0u)
{
    /* tau[0] and tau[1] are the two joint torques. */
}
```
