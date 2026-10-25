# FiveBar

这是五连杆位置、速度和 Jacobian 求解模块。几何尺寸由调用方传入，模块不依赖电机、编码器、CAN、IMU 或具体主控。

## 本文件夹的文件组织

整合结果（按"对外 / 对内"重排后的 `five_bar_merged.h` / `five_bar_merged.c`）已集中到
**`../Chassis/`**，与 VMC、WheelLegLQR 的整合结果组成一套库；本文件夹现在只保留
**原始文件** `five_bar.h` / `five_bar.c` 作对照，内容未改动。

整合版的文件说明、编入工程的方法与调用顺序见 `../Chassis/README.md`。
**新旧两组文件不要同时编译**（会重复定义同一批 `FiveBar_*` 符号）。

### 整合版对外接口（`../Chassis/five_bar_merged.h` 导出）

| 接口 | 作用 |
| --- | --- |
| `FiveBar_Get2026DefaultGeometry()` | 写入 2026 CAD 默认几何 |
| `FiveBar_SolveLegKinematics()` | 主入口：位置 + 速度 + Jacobian |
| `FiveBar_GetLengthAndAxis()` | 腿长与腿轴角单查询（离线校验、可达性判断） |

### 整合版对内实现（`.c` 内 `static`，头文件不导出）

- `FIVE_BAR_IS_FINITE_VALUE` —— 有限值判定宏，合并了原先分散在多处的同义 helper
- `five_bar_normalize_angle()` —— 角度归一化到 `(-pi, pi]`
- `five_bar_is_geometry_usable()` —— 几何参数结构校验
- `five_bar_compute_length_and_axis()` —— 圆交点位置解算核心，供差分复用
- `FiveBarPoint` —— 内部平面点类型

## 输入输出

`FiveBarGeometry` 中所有长度使用 `m`，角度使用 `rad`，速度使用 `rad/s`。`branch_sign` 选择圆交点分支；`min_length_m`、`max_length_m` 限定有效腿长；`min_abs_detJ` 用于标记 Jacobian 是否接近奇异。

`FiveBar_SolveLegKinematics()` 输出 `FiveBarState`：

```text
L0_m, leg_axis_body_rad, dL0_m_s, dleg_axis_body_rad_s,
J[2][2], detJ, valid, jacobian_valid
```

腿轴角按当前实现从机体 `+Y` 方向测量，直立方向为零。求解失败时状态清零并返回 `0`；成功且 Jacobian 通过阈值时返回 `1`。`valid` 与 `jacobian_valid` 仍分别表示几何结果和 Jacobian 安全性。

## 默认配置

`FiveBar_Get2026DefaultGeometry()` 只填充当前 2026 CAD 参数，调用方必须显式调用它，求解器不会偷偷选择默认值。默认参数属于机器人配置，实机使用前应重新核对。

## 最小调用示例

```c
FiveBarGeometry geometry;
FiveBarState state;

FiveBar_Get2026DefaultGeometry(&geometry);
if (FiveBar_SolveLegKinematics(&geometry, q1_rad, q4_rad,
                               dq1_rad_s, dq4_rad_s, &state) != 0u)
{
    /* Pass state.J to VMC and state.L0_m to LQR scheduling. */
}
```
