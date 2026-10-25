#ifndef FIVE_BAR_MERGED_H
#define FIVE_BAR_MERGED_H

/*
 * ============================================================================
 * FiveBar -- 五连杆腿部运动学求解模块（对外唯一头文件）
 * ============================================================================
 *
 * 模块职责
 * --------
 * 由两个驱动关节角度 (q1, q4) 与角速度 (dq1, dq4)，求解腿端到髋部中心的
 * 腿长 L0、腿轴方位角，以及对应的一阶微分（Jacobian）与有效性标志。
 *
 * 依赖约定
 * --------
 * 纯计算，无硬件依赖：不含电机、编码器、CAN、IMU、FreeRTOS、主控相关代码。
 * 几何尺寸由调用方通过 FiveBarGeometry 传入，模块不读取任何全局配置。
 * 本文件与 five_bar_merged.c 成对使用，可独立拷贝到其他工程。
 *
 * 单位约定
 * --------
 * 长度 m，角度 rad，角速度 rad/s。全部接口不使用角度制。
 *
 * 接口分层
 * --------
 * 公开（对外，供上层调用）：
 *   - 类型           FiveBarGeometry / FiveBarState
 *   - 默认配置       FiveBar_Get2026DefaultGeometry()
 *   - 主求解入口     FiveBar_SolveLegKinematics()
 *   - 位置单查询     FiveBar_GetLengthAndAxis()
 *
 * 本文件不导出的内部实现（全部 static 于 five_bar_merged.c）：
 *   - 几何参数校验、有限值判定、角度归一化、圆交点解算等。
 *
 * 线程/重入性
 * -----------
 * 所有函数只读写调用方传入的对象，不持有任何可变全局状态，因此天然可重入；
 * 同一 geometry 可被多任务并发只读使用。输出结构体由调用方提供，模块内部
 * 不做内存分配。
 *
 * 失败语义（对全部接口统一）
 * --------------------------
 * 参数为空、含 NaN/±Inf、或几何上无解时：返回 0，并把调用方提供的输出对象
 * 清零（FiveBar_SolveLegKinematics 整结构体清零；FiveBar_GetLengthAndAxis 输出保持调用前的值——
 * 该函数为位置单查询，不写输出即视为无效）。调用方必须先判断返回值，再使用
 * 输出。
 */

#include <stdint.h>

/* ==========================================================================
 * 一、公开类型
 * ========================================================================== */

/*
 * 五连杆几何参数。
 *
 * 坐标系约定：以两驱动关节连线中点为原点 O，X 轴沿两关节连线、
 * 由关节 A 指向关节 E，Y 轴垂直向上；腿轴方位角自 +Y 方向测量，直立为零。
 *
 * 字段：
 *   l1_m, l4_m              主动臂长度（A-B、E-D），必须 > 0
 *   l2_m, l3_m              从动臂长度（B-C、D-C），必须 > 0
 *   base_width_m            两驱动关节间距（A-E），必须 > 0
 *   branch_sign             圆交点分支选择，非 0；本机型 2026 CAD 取 -1.0f
 *   finite_difference_step_rad
 *                           数值 Jacobian 的中心差分步长，必须 > 0。
 *                           步长直接影响 Jacobian 精度，改小噪声敏感、改大截断
 *                           误差变大，实机调整前需重新验证
 *   triangle_tolerance_m    三角形不等式/开方前的容差，必须 > 0
 *   min_length_m, max_length_m
 *                           有效腿长区间，max >= min；超出即判定无解
 *   min_abs_detJ            |detJ| 奇异判定阈值，必须 > 0
 *
 * 说明：模块只校验上述结构性约束，不做物理可达性推断。参数属于机器人配置，
 * 实机使用前必须重新核对。
 */
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

/*
 * 单腿运动学求解结果。
 *
 * 输入回显（便于上层把结果与输入一起记录）：
 *   q1_rad, q4_rad, dq1_rad_s, dq4_rad_s
 *
 * 位置量：
 *   L0_m                    髋部中心到腿端的距离
 *   leg_axis_body_rad       腿轴在机体坐标系下的方位角（自 +Y 起算）
 *
 * 速度量：
 *   dL0_m_s                 腿长变化率
 *   dleg_axis_body_rad_s    腿轴角速度
 *
 * 微分与判定：
 *   J[2][2]                 Jacobian，行 = [腿长方向, 腿轴角方向]，
 *                           列 = [关节 1, 关节 4]，即 d[L0; angle]/d[q1; q4]
 *   detJ                    J 的行列式
 *   valid                   位置解算是否成功（1 = 几何有解）
 *   jacobian_valid          在 valid 基础上，|detJ| 是否达到阈值（1 = 非奇异）
 *
 * 注意：FiveBar_SolveLegKinematics 返回值等同于 jacobian_valid。valid 表示几何结果可用，
 * jacobian_valid 表示可安全用于 VMC 力矩映射；两者分别判断，不要互相替代。
 */
typedef struct
{
    float q1_rad;
    float q4_rad;
    float dq1_rad_s;
    float dq4_rad_s;
    float L0_m;
    float leg_axis_body_rad;
    float dL0_m_s;
    float dleg_axis_body_rad_s;
    float J[2][2];
    float detJ;
    uint8_t valid;
    uint8_t jacobian_valid;
} FiveBarState;

/* ==========================================================================
 * 二、公开接口
 * ========================================================================== */

/*
 * [默认配置] 写入当前 2026 CAD 的五连杆几何参数。
 *
 * 参数：geometry -- [out] 目标结构体，不可为空。
 * 返回：1 成功；0 入参为空。
 * 说明：默认值属于机器人配置，模块绝不会隐式选用默认值，调用方必须显式调用。
 *       实机使用前须由项目负责人重新核对。
 */
uint8_t FiveBar_Get2026DefaultGeometry(FiveBarGeometry *geometry);

/*
 * [主求解入口] 由关节角度与角速度求解位置、速度与 Jacobian。
 *
 * 参数：geometry -- [in]  几何参数；为空或违反结构约束时失败
 *       q1_rad   -- [in]  关节 1 角度
 *       q4_rad   -- [in]  关节 4 角度
 *       dq1_rad_s-- [in]  关节 1 角速度
 *       dq4_rad_s-- [in]  关节 4 角速度
 *       state    -- [out] 求解结果，不可为空
 * 返回：1 成功且 Jacobian 非奇异；0 失败。
 * 说明：失败时 state 被整结构体清零。返回 1 保证 state->valid 与
 *       state->jacobian_valid 均为 1。上层若要判断"几何有解但接近奇异"，
 *       必须改调 FiveBar_GetLengthAndAxis + 自行判定，或检查 state->valid。
 */
uint8_t FiveBar_SolveLegKinematics(const FiveBarGeometry *geometry,
                      float q1_rad,
                      float q4_rad,
                      float dq1_rad_s,
                      float dq4_rad_s,
                      FiveBarState *state);

/*
 * [位置单查询] 只算位置量，不做差分、不写状态结构体。
 *
 * 参数：geometry             -- [in]  几何参数
 *       q1_rad, q4_rad       -- [in]  关节角度
 *       length_m             -- [out] 腿长，不可为空
 *       leg_axis_body_rad    -- [out] 腿轴方位角，不可为空
 * 返回：1 成功；0 失败（参数为空、含非有限值或几何无解）。
 * 用途：离线校验、标定查表，以及上层只想判断几何可达性的场合；
 *       性能敏感的主链路应直接调用 FiveBar_SolveLegKinematics，避免重复解算。
 */
uint8_t FiveBar_GetLengthAndAxis(const FiveBarGeometry *geometry,
                         float q1_rad,
                         float q4_rad,
                         float *length_m,
                         float *leg_axis_body_rad);

#endif
