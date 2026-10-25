# WheelLegLQR

这是固定轮腿状态定义的 LQR 反馈与腿长增益调度模块，不依赖 STM32、CAN、FreeRTOS 或具体电机驱动。

## 本文件夹的文件组织

整合结果（按"对外 / 对内"重排后的 `wheel_leg_lqr_merged.h` / `wheel_leg_lqr_merged.c`）
已集中到 **`../Chassis/`**，与 FiveBar、VMC 的整合结果组成一套库；本文件夹现在只保留
**原始文件** `wheel_leg_lqr.h/.c`、`wheel_leg_lqr_schedule.h/.c`、
`wheel_leg_lqr_default.h/.c` 作对照，内容未改动。原先是三个编译单元，已在整合版中
合并为一个。

整合版的文件说明、合并掉的重复代码、编入工程的方法与调用顺序见 `../Chassis/README.md`。
**新旧两组文件不要同时编译**（会重复定义同一批 `WheelLegLQR_*` 符号）。

### 整合版对外接口（`../Chassis/wheel_leg_lqr_merged.h` 导出）

| 接口 | 作用 |
| --- | --- |
| `WheelLegLQR_CalculateFeedbackOutput()` | 反馈入口：`u = K * (x_ref - x)` |
| `WheelLegLQR_GetGainsForLegLength()` | 调度入口：按 `L0` 在 K 表上线性插值 |
| `WheelLegLQR_Get2026DefaultSchedule()` | 显式取得 2026 默认调度表 |
| `WHEEL_LEG_LQR_STATE_COUNT` / `_OUTPUT_COUNT` | 状态/输出维数宏 |
| `WheelLegLQRSchedule` | 调用方自备 K 表的数据类型 |

### 整合版对内实现（`.c` 内 `static`/宏，头文件不导出）

- `WHEEL_LEG_LQR_IS_FINITE_VALUE` —— 有限值判定宏。原 `wheel_leg_lqr.c` 与 `wheel_leg_lqr_schedule.c` 各有一份逐字节相同的静态判定函数，已合并为唯一一份
- `WHEEL_LEG_LQR_2026_SCHEDULE_NODE_COUNT` —— 默认表节点数（对外隐藏，请用返回结构的 `count` 字段）
- `wheel_leg_lqr_2026_leg_length_nodes_m` / `_gain_nodes` / `_default_schedule` —— 默认表本体与只读视图

## 接口约定

状态向量固定为：

```text
x = [theta, dtheta, xb, dxb, phi, dphi]
```

单位依次为 `rad`、`rad/s`、`m`、`m/s`、`rad`、`rad/s`。控制输出为 `[T, Tp]`，计算式为：

```text
u = K * (x_ref - x)
```

`WheelLegLQR_CalculateFeedbackOutput()` 的 K 矩阵和状态由调用方提供。输入指针为空或包含非有限值时返回 `0`，输出清零；成功返回 `1`。

## 增益调度

`WheelLegLQR_GetGainsForLegLength()` 根据腿长 `L0` 对调用方提供的 K 表做线性插值，超出表范围时使用最近端点。`WheelLegLQR_Get2026DefaultSchedule()` 提供当前 2026 轮腿 CAD 模型对应的 11 个节点，仅作为显式默认配置；使用前必须由项目负责人复核。

## 最小调用示例

```c
float gain[WHEEL_LEG_LQR_OUTPUT_COUNT][WHEEL_LEG_LQR_STATE_COUNT];
float output[WHEEL_LEG_LQR_OUTPUT_COUNT];
const WheelLegLQRSchedule *schedule =
    WheelLegLQR_Get2026DefaultSchedule();

if (WheelLegLQR_GetGainsForLegLength(schedule, L0_m, gain) != 0u &&
    WheelLegLQR_CalculateFeedbackOutput(gain, x, x_ref, output) != 0u)
{
    /* output[0] = T, output[1] = Tp */
}
```
