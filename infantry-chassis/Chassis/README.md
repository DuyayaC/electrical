# Chassis

轮腿底盘的**整合代码库**。原先分散在 `FiveBar`、`VMC`、`WheelLegLQR` 三个文件夹里的
代码，已在各自文件夹内完成整合（合并重复实现、补注释、按对外/对内分层），
再把整合结果集中到本文件夹，形成一套可直接编入工程的库。

## 文件清单

| 文件 | 来源模块 | 对外接口 |
| --- | --- | --- |
| `five_bar_merged.h` / `.c` | FiveBar | `FiveBar_Get2026DefaultGeometry()`、`FiveBar_SolveLegKinematics()`、`FiveBar_GetLengthAndAxis()` |
| `vmc_merged.h` / `.c` | VMC | `VMC_ConvertWrenchToJointTorque()` |
| `wheel_leg_lqr_merged.h` / `.c` | WheelLegLQR | `WheelLegLQR_CalculateFeedbackOutput()`、`WheelLegLQR_GetGainsForLegLength()`、`WheelLegLQR_Get2026DefaultSchedule()` |
| `test/selftest.c` | 验证程序 | 对外/对内等价性回归测试（非产品代码） |

### 函数改名对照

整合过程中所有函数名都改成了能直接读出用途的形式，**不留旧名**，请同步更新调用处：

| 旧名 | 新名 |
| --- | --- |
| `FiveBar_Solve` | `FiveBar_SolveLegKinematics` |
| `FiveBar_Position` | `FiveBar_GetLengthAndAxis` |
| `FiveBar_GetDefault2026Geometry` | `FiveBar_Get2026DefaultGeometry` |
| `VMC_Calculate2D` | `VMC_ConvertWrenchToJointTorque` |
| `WheelLegLQR_Calculate` | `WheelLegLQR_CalculateFeedbackOutput` |
| `WheelLegLQR_ScheduleInterpolate` | `WheelLegLQR_GetGainsForLegLength` |
| `WheelLegLQR_GetDefault2026Schedule` | `WheelLegLQR_Get2026DefaultSchedule` |

`FiveBar` / `VMC` / `WheelLegLQR` 文件夹里的原始文件仍使用旧名，可作对照。

## 六个文件可以平铺共存

三组文件的头文件宏与公开符号互不冲突，因此本文件夹不需要子目录：

- 包含卫士各自独立：`FIVE_BAR_MERGED_H`、`VMC_MERGED_H`、`WHEEL_LEG_LQR_MERGED_H`
- 公开符号各自带模块前缀：`FiveBar_*`、`VMC_*`、`WheelLegLQR_*`
- 各模块的私有实现全部 `static` 在自己的 `.c` 内，`.c` 之间无交叉引用

三个模块互相独立、无跨模块依赖：只用到 FiveBar 的工程可以只拷这一对文件。

## 编入工程

把需要的 `.c` 加入编译、把本文件夹加入头文件搜索路径即可：

```c
#include "five_bar_merged.h"      /* 或直接包含全部三个 */
#include "vmc_merged.h"
#include "wheel_leg_lqr_merged.h"
```

只编 `.c`，不要把 `.h` 加入编译列表。**不要**同时编译原文件夹里的旧文件
（`five_bar.c`、`vmc.c`、`wheel_leg_lqr*.c`），否则会重复定义同一批符号。
旧文件仍保留在原文件夹内作对照。

## 典型调用顺序（每个控制周期）

```c
FiveBarGeometry geometry;
FiveBarState    state;
float           gain[WHEEL_LEG_LQR_OUTPUT_COUNT][WHEEL_LEG_LQR_STATE_COUNT];
float           tau[2];
float           u[WHEEL_LEG_LQR_OUTPUT_COUNT];

/* 初始化阶段：显式取默认配置，模块不会隐式使用默认值 */
FiveBar_Get2026DefaultGeometry(&geometry);

/* 主链路 */
if (FiveBar_SolveLegKinematics(&geometry, q1, q4, dq1, dq4, &state) == 0u)
{
    /* 无解、接近奇异或输入非法：本周期放弃控制，不要把旧值当新值用 */
}
else if (VMC_ConvertWrenchToJointTorque(state.J, force_N, pitch_torque_Nm, tau) == 0u ||
         WheelLegLQR_GetGainsForLegLength(WheelLegLQR_Get2026DefaultSchedule(),
                                          state.L0_m, gain) == 0u ||
         WheelLegLQR_CalculateFeedbackOutput(gain, x, x_ref, u) == 0u)
{
    /* 任一环节失败，输出必已清零 */
}
```

要点：

- `FiveBar_SolveLegKinematics()` 返回值等于 `jacobian_valid`，为 `0` 时**不要**把 `state.J`
  送进 VMC（VMC 自身不做奇异检测）
- `state.L0_m` 是增益调度的输入，`state.J` 是 VMC 的输入，两者来自同一次解算
- 所有接口失败时都会清零自己的输出，不会残留上一周期结果

## 单元测试

`test/selftest.c` 是等价性回归测试：同一份测试代码分别链接**整合库**和**原文件**，
把全部公开接口跑遍（有效输入、空指针、NaN、±Inf、退化几何、非升序增益表、
单节点表、以及 FiveBar→VMC 与 FiveBar→LQR 全链路），把每个状态字节与输出字节
折进一个 FNV-1a 指纹。指纹一致即证明整合没有改变行为。

```powershell
$gcc = 'D:\mingw\mingw64\bin\gcc.exe'
$inc = '-IChassis','-IFiveBar','-IVMC','-IWheelLegLQR'

# 整合库
& $gcc -std=c99 -O0 -ffp-contract=off -Wall -Wextra -Werror @inc `
  'Chassis\test\selftest.c' 'Chassis\five_bar_merged.c' 'Chassis\vmc_merged.c' `
  'Chassis\wheel_leg_lqr_merged.c' -o 'Chassis\test\bin\selftest_merged.exe' -lm

# 原文件（基线）
& $gcc -std=c99 -O0 -ffp-contract=off -Wall -Wextra -Werror @inc `
  -DINFANTRY_CHASSIS_BUILD_ORIGINAL 'Chassis\test\selftest.c' `
  'FiveBar\five_bar.c' 'VMC\vmc.c' 'WheelLegLQR\wheel_leg_lqr.c' `
  'WheelLegLQR\wheel_leg_lqr_schedule.c' 'WheelLegLQR\wheel_leg_lqr_default.c' `
  -o 'Chassis\test\bin\selftest_original.exe' -lm

# 两次运行的 fingerprint 应完全一致，failures 应为 0
& 'Chassis\test\bin\selftest_merged.exe'
& 'Chassis\test\bin\selftest_original.exe'
```

`test/bin/` 下保留了预编译好的四个可执行文件（`-O0`/`-O2` × 整合/原文件），
编译环境不同时请按上面的命令重新生成。该目录只用于验证，不参与固件编译。

## 参数责任

整合库中带 "2026" 的默认配置来自当前 CAD 模型，属**机器人配置而非算法常量**：

- `FiveBar_Get2026DefaultGeometry()` —— 五连杆几何
- `WheelLegLQR_Get2026DefaultSchedule()` —— 11 节点腿长增益表

模块在任何情况下都不会隐式使用它们，必须由调用方显式取得。更换机型、改动结构件
或重新整定后必须重新核对/生成，实机使用前须由项目负责人复核。
