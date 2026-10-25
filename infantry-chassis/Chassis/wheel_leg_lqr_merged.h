#ifndef WHEEL_LEG_LQR_MERGED_H
#define WHEEL_LEG_LQR_MERGED_H

/*
 * ============================================================================
 * WheelLegLQR -- 轮腿 LQR 反馈 + 腿长增益调度模块（对外唯一头文件）
 * ============================================================================
 *
 * 模块职责
 * --------
 *   1) LQR 反馈：u = K * (x_ref - x)；
 *   2) 增益调度：按实测腿长 L0 在调用方给出的 K 表上做线性插值，
 *      并提供一份显式的 2026 机型默认 K 表。
 *
 * 依赖约定
 * --------
 * 纯计算，无硬件依赖：不含 STM32、CAN、FreeRTOS、电机驱动、传感器采集代码，
 * 也不做状态估计——x 与 K 全部由调用方提供。本文件与 wheel_leg_lqr_merged.c
 * 成对使用，可独立拷贝到其他工程。
 *
 * 本模块只做"给定状态与增益算控制量"，不含动力学模型、不含积分器、不含限幅；
 * 限幅、积分抗饱和、状态估计均属上层职责。
 *
 * 状态与单位约定
 * --------------
 *   x = [theta, dtheta, xb, dxb, phi, dphi]
 *         rad    rad/s   m    m/s   rad  rad/s
 *
 *   theta  : 机体俯仰角          dtheta : 俯仰角速度
 *   xb     : 机体水平位置        dxb    : 机体水平速度
 *   phi    : 轮子转角            dphi   : 轮子角速度
 *
 * 控制输出
 * --------
 *   u = [T, Tp]
 *   T  : 轮子驱动力矩，Tp : 关节/腿轴力矩（具体落到哪个执行器由上层决定）
 *
 * 接口分层
 * --------
 * 公开（对外，供上层调用）：
 *   - 宏             WHEEL_LEG_LQR_STATE_COUNT / _OUTPUT_COUNT
 *   - 类型           WheelLegLQRSchedule
 *   - 反馈入口       WheelLegLQR_CalculateFeedbackOutput()
 *   - 调度入口       WheelLegLQR_GetGainsForLegLength()
 *   - 默认配置       WheelLegLQR_Get2026DefaultSchedule()
 *
 * 本文件不导出的内部实现（全部 static 于 wheel_leg_lqr_merged.c）：
 *   - 有限值判定宏、2026 默认表节点数与两张表本体、默认调度结构体。
 *
 * 调用顺序（典型主链路，每个控制周期执行）
 * ----------------------------------------
 *   FiveBar_SolveLegKinematics() -> L0、J
 *     -> WheelLegLQR_GetGainsForLegLength(schedule, L0, gain)   // 取当前腿长下的 K
 *     -> WheelLegLQR_CalculateFeedbackOutput(gain, x, x_ref, output)         // 算 [T, Tp]
 *
 * 线程/重入性
 * -----------
 * 只读写调用方传入的对象。默认调度表为 static const，多任务并发只读安全；
 * 但**不得修改**其内容（见 WheelLegLQR_Get2026DefaultSchedule 说明）。
 *
 * 失败语义（对本模块全部接口统一）
 * --------------------------------
 * 参数为空、长度为零、或含 NaN/±Inf 时返回 0，并把调用方提供的输出清零
 * （Calculate 清 output，ScheduleInterpolate 清 gain），不残留上次结果。
 */

#include <stdint.h>

/* ==========================================================================
 * 一、公开常量
 * ========================================================================== */

/* 状态向量维数，同时也是 K 矩阵的列数。 */
#define WHEEL_LEG_LQR_STATE_COUNT 6u

/* 控制输出维数，同时也是 K 矩阵的行数。 */
#define WHEEL_LEG_LQR_OUTPUT_COUNT 2u

/* ==========================================================================
 * 二、公开类型
 * ========================================================================== */

/*
 * 运行时增益调度表：K 随腿长 L0 变化的离散节点。
 *
 * 字段：
 *   count -- 节点个数，至少 1；为 0 视为非法表
 *   l0_m  -- [in] 节点腿长数组，长度 count，单位 m。
 *            调用方须保证按升序排列；本模块在插值时检查相邻节点是否非升序，
 *            发现 `l0_m[i+1] <= l0_m[i]` 即判表非法并返回失败
 *   gain  -- [in] 节点增益数组，长度 count，每节点一个
 *            [输出维][状态维] 的 K 矩阵
 *
 * 表由调用方持有，本模块只读，不拷贝、不修改；生命周期须覆盖整个使用期。
 * 通过 WheelLegLQR_Get2026DefaultSchedule() 取得的是模块内静态表的只读视图。
 */
typedef struct
{
    uint32_t count;
    const float *l0_m;
    const float (*gain)[WHEEL_LEG_LQR_OUTPUT_COUNT]
                      [WHEEL_LEG_LQR_STATE_COUNT];
} WheelLegLQRSchedule;

/* ==========================================================================
 * 三、公开接口
 * ========================================================================== */

/*
 * [反馈入口] u = K * (x_ref - x)。
 *
 * 参数：gain   -- [in]  K 矩阵，[2][6]，通常来自 ScheduleInterpolate
 *       x      -- [in]  当前状态 [theta, dtheta, xb, dxb, phi, dphi]
 *       x_ref  -- [in]  目标状态，同序同单位
 *       output -- [out] 输出 [T, Tp]，不可为空
 * 返回：1 成功；0 失败。
 * 说明：三个输入数组长度固定为 WHEEL_LEG_LQR_STATE_COUNT，本模块不做长度
 *       校验（C 无法获知数组实参长度），由调用方保证。
 *       失败（含任一输入非有限值）时 output 清零。
 */
uint8_t WheelLegLQR_CalculateFeedbackOutput(
    const float gain[WHEEL_LEG_LQR_OUTPUT_COUNT]
                [WHEEL_LEG_LQR_STATE_COUNT],
    const float x[WHEEL_LEG_LQR_STATE_COUNT],
    const float x_ref[WHEEL_LEG_LQR_STATE_COUNT],
    float output[WHEEL_LEG_LQR_OUTPUT_COUNT]);

/*
 * [调度入口] 按腿长 L0 在当前 K 表上线性插值。
 *
 * 参数：schedule -- [in]  调度表，不可为空，count 不得为 0
 *       l0_m     -- [in]  当前腿长，单位 m
 *       gain     -- [out] 插值得到的 K 矩阵，不可为空
 * 返回：1 成功；0 失败。
 *
 * 边界行为：L0 落在表范围之外时取最近端点（不外推），即
 *   l0 <= l0_m[0]        -> 取第 0 个节点
 *   l0 >= l0_m[count-1]  -> 取最后一个节点
 *   count == 1           -> 恒取该唯一节点，与 l0 无关
 * 表校验：非升序的相邻节点视为非法表，返回 0；节点增益含非有限值同样返回 0。
 * 失败时 gain 会被清零。
 */
uint8_t WheelLegLQR_GetGainsForLegLength(
    const WheelLegLQRSchedule *schedule,
    float l0_m,
    float gain[WHEEL_LEG_LQR_OUTPUT_COUNT]
                [WHEEL_LEG_LQR_STATE_COUNT]);

/*
 * [默认配置] 返回当前 2026 轮腿 CAD 机型对应的 11 节点默认调度表。
 *
 * 返回：静态存储的只读表指针；模块内建，永不为空。
 *
 * 生命周期与只读性：返回值指向 static const 数据，生命周期为整个程序运行期，
 * 调用方**不得修改**其内容（表本身以 const 修饰，写入会被编译器拒绝；
 * 若通过强制转换绕过，属未定义行为）。
 *
 * 数值来源与责任：节点为当前 2026 机型模型参数，属机器人配置而非算法常量。
 * 模块绝不会隐式使用它——必须由调用方显式取得并传入。实机使用前须由项目
 * 负责人复核，机型或整定结果变化后必须重新生成该表。
 */
const WheelLegLQRSchedule *WheelLegLQR_Get2026DefaultSchedule(void);

#endif
