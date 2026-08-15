# FiveBar

这是五连杆位置、速度和 Jacobian 求解模块。几何尺寸由调用方传入，模块不依赖电机、编码器、CAN、IMU 或具体主控。

## 输入输出

`FiveBarGeometry` 中所有长度使用 `m`，角度使用 `rad`，速度使用 `rad/s`。`branch_sign` 选择圆交点分支；`min_length_m`、`max_length_m` 限定有效腿长；`min_abs_detJ` 用于标记 Jacobian 是否接近奇异。

`FiveBar_Solve()` 输出 `FiveBarState`：

```text
L0_m, leg_axis_body_rad, dL0_m_s, dleg_axis_body_rad_s,
J[2][2], detJ, valid, jacobian_valid
```

腿轴角按当前实现从机体 `+Y` 方向测量，直立方向为零。求解失败时状态清零并返回 `0`；成功且 Jacobian 通过阈值时返回 `1`。`valid` 与 `jacobian_valid` 仍分别表示几何结果和 Jacobian 安全性。

## 默认配置

`FiveBar_GetDefault2026Geometry()` 只填充当前 2026 CAD 参数，调用方必须显式调用它，求解器不会偷偷选择默认值。默认参数属于机器人配置，实机使用前应重新核对。

## 最小调用示例

```c
FiveBarGeometry geometry;
FiveBarState state;

FiveBar_GetDefault2026Geometry(&geometry);
if (FiveBar_Solve(&geometry, q1_rad, q4_rad,
                  dq1_rad_s, dq4_rad_s, &state) != 0u)
{
    /* Pass state.J to VMC and state.L0_m to LQR scheduling. */
}
```
