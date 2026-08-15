# WheelLegLQR

这是固定轮腿状态定义的 LQR 反馈与腿长增益调度模块，不依赖 STM32、CAN、FreeRTOS 或具体电机驱动。

## 接口约定

状态向量固定为：

```text
x = [theta, dtheta, xb, dxb, phi, dphi]
```

单位依次为 `rad`、`rad/s`、`m`、`m/s`、`rad`、`rad/s`。控制输出为 `[T, Tp]`，计算式为：

```text
u = K * (x_ref - x)
```

`WheelLegLQR_Calculate()` 的 K 矩阵和状态由调用方提供。输入指针为空或包含非有限值时返回 `0`，输出清零；成功返回 `1`。

## 增益调度

`WheelLegLQR_ScheduleInterpolate()` 根据腿长 `L0` 对调用方提供的 K 表做线性插值，超出表范围时使用最近端点。`WheelLegLQR_GetDefault2026Schedule()` 提供当前 2026 轮腿 CAD 模型对应的 11 个节点，仅作为显式默认配置；使用前必须由项目负责人复核。

## 最小调用示例

```c
float gain[WHEEL_LEG_LQR_OUTPUT_COUNT][WHEEL_LEG_LQR_STATE_COUNT];
float output[WHEEL_LEG_LQR_OUTPUT_COUNT];
const WheelLegLQRSchedule *schedule =
    WheelLegLQR_GetDefault2026Schedule();

if (WheelLegLQR_ScheduleInterpolate(schedule, L0_m, gain) != 0u &&
    WheelLegLQR_Calculate(gain, x, x_ref, output) != 0u)
{
    /* output[0] = T, output[1] = Tp */
}
```
