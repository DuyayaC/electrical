/*
 * ============================================================================
 * FiveBar -- 五连杆腿部运动学求解模块（对外唯一源文件）
 * ============================================================================
 *
 * 配套头文件：five_bar_merged.h（全部对外声明见该文件）
 *
 * 本文件结构（按"对外 / 对内"排布）
 * --------------------------------
 *   一、编译期常量与内部工具宏      [对内]
 *   二、内部类型                    [对内]
 *   三、内部静态函数（几何校验、
 *       有限值判定、角度归一化、
 *       圆交点位置解算）            [对内]
 *   四、公开接口实现
 *       - FiveBar_Get2026DefaultGeometry
 *       - FiveBar_SolveLegKinematics
 *       - FiveBar_GetLengthAndAxis          [对外]
 *
 * 求解流程
 * --------
 *   FiveBar_SolveLegKinematics
 *     ├─ 几何与输入校验
 *     ├─ 位置解算（five_bar_compute_length_and_axis）
 *     ├─ 对 q1、q4 各做一次中心差分 -> 2x2 Jacobian J
 *     ├─ detJ、dL0、d(leg_axis) 由 J 与关节角速度合成
 *     └─ |detJ| 与阈值比较 -> jacobian_valid
 *
 * 数值实现说明
 * ------------
 * Jacobian 用中心差分而非解析式：解析式需要区分分支且易在三角退化处出现
 * 除零，中心差分在靠近奇异位形时同样会暴露问题（detJ 变小），由阈值统一兜住。
 * 全程使用单精度 float，与主控一致；除零、开方负数等边界均由容差与有限值
 * 判定提前拦截，不依赖异常或浮点环境。
 */

#include "five_bar_merged.h"

#include <float.h>
#include <math.h>
#include <string.h>

/* ==========================================================================
 * 一、编译期常量与内部工具宏                      [对内]
 * ========================================================================== */

/*
 * 圆周率。允许外部在编译期覆盖，便于与其他模块共用常量。
 */
#ifndef FIVE_BAR_PI
#define FIVE_BAR_PI 3.14159265358979323846f
#endif

/*
 * 有限值判定（内部工具）。
 *
 * 为什么用表达式而不是 isfinite()/std::isfinite：
 *   - 部分嵌入式 C 库未提供 isfinite，或需要额外 feature 宏；
 *   - 本模块要求不依赖 libm 的浮点分类函数，保持可在任意工具链编译。
 * 判定方式 value == value 排除 NaN，与 FLT_MAX 比较排除 ±Inf。
 *
 * 用法约定：参数必须是"可多次求值且无副作用"的表达式（本模块只传结构体成员
 * 与已计算的局部变量，均满足）。
 *
 * 注：模块内原有多处同义静态 helper（几何校验、位置解算各自一份），
 *     已合并为本宏，避免同一判定出现多份实现而逐渐走偏。
 */
#define FIVE_BAR_IS_FINITE_VALUE(value)                                       \
    (((value) == (value) && (value) <= FLT_MAX && (value) >= -FLT_MAX)   \
         ? 1u                                                            \
         : 0u)

/* ==========================================================================
 * 二、内部类型                                    [对内]
 * ========================================================================== */

/* 平面点，仅在圆交点解算中使用，不对外暴露。 */
typedef struct
{
    float x;
    float y;
} FiveBarPoint;

/* ==========================================================================
 * 三、内部静态函数                                [对内]
 * ========================================================================== */

/*
 * 角度归一化到 (-pi, pi]。
 *
 * 用途：中心差分得到的是两个单独归一的角之差，其量程可能落在 (-2pi, 2pi)，
 * 若不归一化，差分在 ±pi 附近会给出错误的巨大导数值。
 *
 * 参数：angle_rad -- 待归一化角度。
 * 返回：等价角度，落在 (-pi, pi]。
 */
static float five_bar_normalize_angle(float angle_rad)
{
    while (angle_rad > FIVE_BAR_PI)
    {
        angle_rad -= 2.0f * FIVE_BAR_PI;
    }
    while (angle_rad < -FIVE_BAR_PI)
    {
        angle_rad += 2.0f * FIVE_BAR_PI;
    }
    return angle_rad;
}

/*
 * 几何参数结构校验。
 *
 * 返回：1 可用于求解；0 不可用。
 *
 * 校验内容分两组：
 *   1) 所有字段必须为有限值（NaN/±Inf 直接判定不可用）；
 *   2) 结构性约束：臂长与基座宽度为正、分支非零、差分步长与容差为正、
 *      腿长区间非负且上界不小于下界、奇异阈值为正。
 *
 * 注意：本函数只做结构性检查，不判断物理可达性；参数是否与真实 CAD 一致
 * 由调用方负责，因此默认值必须显式获取并复核。
 */
static uint8_t five_bar_is_geometry_usable(const FiveBarGeometry *geometry)
{
    if (geometry == 0 ||
        FIVE_BAR_IS_FINITE_VALUE(geometry->l1_m) == 0u ||
        FIVE_BAR_IS_FINITE_VALUE(geometry->l2_m) == 0u ||
        FIVE_BAR_IS_FINITE_VALUE(geometry->l3_m) == 0u ||
        FIVE_BAR_IS_FINITE_VALUE(geometry->l4_m) == 0u ||
        FIVE_BAR_IS_FINITE_VALUE(geometry->base_width_m) == 0u ||
        FIVE_BAR_IS_FINITE_VALUE(geometry->branch_sign) == 0u ||
        FIVE_BAR_IS_FINITE_VALUE(geometry->finite_difference_step_rad) == 0u ||
        FIVE_BAR_IS_FINITE_VALUE(geometry->triangle_tolerance_m) == 0u ||
        FIVE_BAR_IS_FINITE_VALUE(geometry->min_length_m) == 0u ||
        FIVE_BAR_IS_FINITE_VALUE(geometry->max_length_m) == 0u ||
        FIVE_BAR_IS_FINITE_VALUE(geometry->min_abs_detJ) == 0u ||
        geometry->l1_m <= 0.0f || geometry->l2_m <= 0.0f ||
        geometry->l3_m <= 0.0f || geometry->l4_m <= 0.0f ||
        geometry->base_width_m <= 0.0f ||
        geometry->branch_sign == 0.0f ||
        geometry->finite_difference_step_rad <= 0.0f ||
        geometry->triangle_tolerance_m <= 0.0f ||
        geometry->min_length_m < 0.0f ||
        geometry->max_length_m < geometry->min_length_m ||
        geometry->min_abs_detJ <= 0.0f)
    {
        return 0u;
    }
    return 1u;
}

/*
 * 位置解算核心（内部实现，无对外语义包装）。
 *
 * 几何关系（A、E 为两驱动关节，B、D 为主动臂末端，C 为腿端）：
 *
 *        B (A + l1 * dir(q1))            D (E + l4 * dir(q4))
 *         \                               /
 *          \ l2                        l3 /
 *           \                             /
 *            ----------- C --------------
 *                        |
 *                        | L0
 *                        O (两关节连线中点)
 *
 * 步骤：
 *   1) 由 q1 求 B、由 q4 求 D；
 *   2) 求以 B 为心半径 l2、以 D 为心半径 l3 的两圆交点，得 C；
 *      branch_sign 决定取上/下哪一个交点，是本机型装配分支的唯一来源；
 *   3) 求 C 到中点的距离 L0 与方位角（自 +Y 起算）。
 *
 * 参数：geometry          -- [in]  几何参数（调用方保证自身非空；本函数仍会校验）
 *       q1_rad, q4_rad    -- [in]  关节角度
 *       length_m          -- [out] 腿长，可空 -> 直接失败
 *       leg_axis_body_rad -- [out] 腿轴方位角，可空 -> 直接失败
 * 返回：1 成功；0 无解（两圆相离、两臂重合、腿长超出有效区间、或参数非法）。
 *
 * 说明：本函数不校验 q1/q4 的有限性之外的东西，也不写调用方状态结构体；
 *       中心差分需要复用本函数，故独立于 FiveBar_GetLengthAndAxis 存在。
 */
static uint8_t five_bar_compute_length_and_axis(const FiveBarGeometry *geometry,
                                          float q1_rad,
                                          float q4_rad,
                                          float *length_m,
                                          float *leg_axis_body_rad)
{
    FiveBarPoint A;
    FiveBarPoint E;
    FiveBarPoint B;
    FiveBarPoint D;
    FiveBarPoint C;
    FiveBarPoint O;
    FiveBarPoint BD;
    FiveBarPoint unit;
    FiveBarPoint perpendicular;
    FiveBarPoint base_point;
    float distance;
    float projection;
    float height_squared;
    float height;
    float dx;
    float dy;
    float radius;

    if (length_m == 0 || leg_axis_body_rad == 0 ||
        five_bar_is_geometry_usable(geometry) == 0u ||
        FIVE_BAR_IS_FINITE_VALUE(q1_rad) == 0u ||
        FIVE_BAR_IS_FINITE_VALUE(q4_rad) == 0u)
    {
        return 0u;
    }

    /* 两驱动关节位置：A 在原点，E 沿 X 轴偏移一个基座宽度。 */
    A.x = 0.0f;
    A.y = 0.0f;
    E.x = geometry->base_width_m;
    E.y = 0.0f;

    /* 主动臂末端 B、D。 */
    B.x = A.x + geometry->l1_m * cosf(q1_rad);
    B.y = A.y + geometry->l1_m * sinf(q1_rad);
    D.x = E.x + geometry->l4_m * cosf(q4_rad);
    D.y = E.y + geometry->l4_m * sinf(q4_rad);

    /* 两主动臂末端距离；过近说明机构接近折叠，此时法向不稳定，直接判无解。 */
    BD.x = D.x - B.x;
    BD.y = D.y - B.y;
    distance = sqrtf(BD.x * BD.x + BD.y * BD.y);
    if (distance <= geometry->triangle_tolerance_m)
    {
        return 0u;
    }

    /* 两圆交点：在 B-D 连线上取投影长度，再由勾股定理得垂距。
     * projection 为交点到 B 的投影距离；height 为垂距。 */
    projection = (geometry->l2_m * geometry->l2_m -
                  geometry->l3_m * geometry->l3_m + distance * distance) /
                 (2.0f * distance);
    height_squared = geometry->l2_m * geometry->l2_m - projection * projection;
    if (height_squared < -geometry->triangle_tolerance_m)
    {
        /* 两圆相离：从动臂长度组合在几何上不可能。 */
        return 0u;
    }
    if (height_squared < 0.0f)
    {
        /* 相切附近的浮点负零修正，避免对极小负数开方。 */
        height_squared = 0.0f;
    }

    unit.x = BD.x / distance;
    unit.y = BD.y / distance;
    base_point.x = B.x + projection * unit.x;
    base_point.y = B.y + projection * unit.y;
    /* 法向为 B-D 方向逆时针旋转 90 度，branch_sign 选择两侧之一。 */
    perpendicular.x = -unit.y;
    perpendicular.y = unit.x;
    height = sqrtf(height_squared);

    C.x = base_point.x + geometry->branch_sign * height * perpendicular.x;
    C.y = base_point.y + geometry->branch_sign * height * perpendicular.y;
    O.x = geometry->base_width_m * 0.5f;
    O.y = 0.0f;

    /* 腿长与腿轴方位角：方位角自 +Y 起算，故用 atan2(dx, dy)。 */
    dx = C.x - O.x;
    dy = C.y - O.y;
    radius = sqrtf(dx * dx + dy * dy);
    if (radius < geometry->min_length_m || radius > geometry->max_length_m)
    {
        return 0u;
    }

    *length_m = radius;
    *leg_axis_body_rad = atan2f(dx, dy);
    return 1u;
}

/* ==========================================================================
 * 四、公开接口实现                                [对外]
 * ========================================================================== */

/*
 * [默认配置] 2026 CAD 五连杆几何参数。
 *
 * 数值来源：当前 2026 轮腿 CAD 模型。这些是机器人配置而非算法常量，更换机型
 * 或改动结构件后必须重新测量并核对；模块在任何情况下都不会隐式使用它们。
 */
uint8_t FiveBar_Get2026DefaultGeometry(FiveBarGeometry *geometry)
{
    if (geometry == 0)
    {
        return 0u;
    }
    geometry->l1_m = 0.145f;
    geometry->l2_m = 0.270f;
    geometry->l3_m = 0.270f;
    geometry->l4_m = 0.145f;
    geometry->base_width_m = 0.150f;
    geometry->branch_sign = -1.0f;
    geometry->finite_difference_step_rad = 0.0001f;
    geometry->triangle_tolerance_m = 0.000001f;
    geometry->min_length_m = 0.1042f;
    geometry->max_length_m = 0.3672f;
    geometry->min_abs_detJ = 0.00001f;
    return 1u;
}

/*
 * [位置单查询] 对外入口，包装内部位置解算。
 *
 * 与直接调用 five_bar_compute_length_and_axis 等价；独立保留此公开函数是为了
 * 保证整合前的调用方（离线校验、标定脚本、上层可达性判断）源码不变。
 */
uint8_t FiveBar_GetLengthAndAxis(const FiveBarGeometry *geometry,
                         float q1_rad,
                         float q4_rad,
                         float *length_m,
                         float *leg_axis_body_rad)
{
    return five_bar_compute_length_and_axis(geometry,
                                      q1_rad,
                                      q4_rad,
                                      length_m,
                                      leg_axis_body_rad);
}

/*
 * [主求解入口] 位置 + 速度 + Jacobian。
 *
 * 实现要点：
 *   - 先整体清零输出，保证任何失败路径下调用方拿到的都是确定值；
 *   - 位置解算成功后，对 q1、q4 各做 ±step 的中心差分共 4 次位置解算，
 *     得到 J 的 4 个元素（差分期间任一次解算失败即整体判失败；
 *     注意这里要求 ±step 仍在可达范围内，因此边界位形附近会整体返回失败）；
 *   - 角度方向的一行必须先归一化再除以 2*step，否则 ±pi 附近导数错误；
 *   - detJ 与速度量算完后统一做有限值复查，防止溢出值悄悄传给 VMC。
 */
uint8_t FiveBar_SolveLegKinematics(const FiveBarGeometry *geometry,
                      float q1_rad,
                      float q4_rad,
                      float dq1_rad_s,
                      float dq4_rad_s,
                      FiveBarState *state)
{
    float length_plus_q1;
    float length_minus_q1;
    float angle_plus_q1;
    float angle_minus_q1;
    float length_plus_q4;
    float length_minus_q4;
    float angle_plus_q4;
    float angle_minus_q4;
    float step;

    if (state == 0)
    {
        return 0u;
    }
    memset(state, 0, sizeof(*state));
    if (five_bar_is_geometry_usable(geometry) == 0u ||
        FIVE_BAR_IS_FINITE_VALUE(q1_rad) == 0u ||
        FIVE_BAR_IS_FINITE_VALUE(q4_rad) == 0u ||
        FIVE_BAR_IS_FINITE_VALUE(dq1_rad_s) == 0u ||
        FIVE_BAR_IS_FINITE_VALUE(dq4_rad_s) == 0u)
    {
        return 0u;
    }

    /* 回显输入，便于上层把结果与输入一起记录/回放。 */
    state->q1_rad = q1_rad;
    state->q4_rad = q4_rad;
    state->dq1_rad_s = dq1_rad_s;
    state->dq4_rad_s = dq4_rad_s;

    if (FiveBar_GetLengthAndAxis(geometry, q1_rad, q4_rad,
                         &state->L0_m, &state->leg_axis_body_rad) == 0u)
    {
        return 0u;
    }

    /* 中心差分求 Jacobian：d[L0; angle] / d[q1; q4]。 */
    step = geometry->finite_difference_step_rad;
    if (FiveBar_GetLengthAndAxis(geometry, q1_rad + step, q4_rad,
                         &length_plus_q1, &angle_plus_q1) == 0u ||
        FiveBar_GetLengthAndAxis(geometry, q1_rad - step, q4_rad,
                         &length_minus_q1, &angle_minus_q1) == 0u ||
        FiveBar_GetLengthAndAxis(geometry, q1_rad, q4_rad + step,
                         &length_plus_q4, &angle_plus_q4) == 0u ||
        FiveBar_GetLengthAndAxis(geometry, q1_rad, q4_rad - step,
                         &length_minus_q4, &angle_minus_q4) == 0u)
    {
        memset(state, 0, sizeof(*state));
        return 0u;
    }

    /* 第 0 行：腿长对两关节的偏导。 */
    state->J[0][0] = (length_plus_q1 - length_minus_q1) / (2.0f * step);
    state->J[0][1] = (length_plus_q4 - length_minus_q4) / (2.0f * step);
    /* 第 1 行：腿轴角对两关节的偏导（先归一化角度差）。 */
    state->J[1][0] = five_bar_normalize_angle(angle_plus_q1 - angle_minus_q1) /
                     (2.0f * step);
    state->J[1][1] = five_bar_normalize_angle(angle_plus_q4 - angle_minus_q4) /
                     (2.0f * step);
    state->detJ = state->J[0][0] * state->J[1][1] -
                  state->J[0][1] * state->J[1][0];
    /* 由 J 与关节角速度合成腿长/腿轴速度。 */
    state->dL0_m_s = state->J[0][0] * dq1_rad_s +
                     state->J[0][1] * dq4_rad_s;
    state->dleg_axis_body_rad_s = state->J[1][0] * dq1_rad_s +
                                  state->J[1][1] * dq4_rad_s;
    if (FIVE_BAR_IS_FINITE_VALUE(state->detJ) == 0u ||
        FIVE_BAR_IS_FINITE_VALUE(state->dL0_m_s) == 0u ||
        FIVE_BAR_IS_FINITE_VALUE(state->dleg_axis_body_rad_s) == 0u)
    {
        memset(state, 0, sizeof(*state));
        return 0u;
    }

    state->valid = 1u;
    /* 奇异判定：|detJ| 低于阈值说明该位形下无法可靠换算力矩，返回 0 拒绝。 */
    state->jacobian_valid =
        (fabsf(state->detJ) >= geometry->min_abs_detJ) ? 1u : 0u;
    return state->jacobian_valid;
}
