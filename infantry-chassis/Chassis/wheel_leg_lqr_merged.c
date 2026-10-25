/*
 * ============================================================================
 * WheelLegLQR -- 轮腿 LQR 反馈 + 腿长增益调度模块（对外唯一源文件）
 * ============================================================================
 *
 * 配套头文件：wheel_leg_lqr_merged.h（对外声明、状态约定、失败语义见该文件）
 *
 * 本文件结构（按"对外 / 对内"排布）
 * --------------------------------
 *   一、内部工具宏                  [对内]
 *   二、内部数据：2026 默认调度表    [对内]
 *   三、公开接口实现
 *       - WheelLegLQR_CalculateFeedbackOutput
 *       - WheelLegLQR_GetGainsForLegLength
 *       - WheelLegLQR_Get2026DefaultSchedule   [对外]
 *
 * 合并说明（本文件由原三个编译单元合并而成）
 * -----------------------------------------
 *   原 wheel_leg_lqr.c          -> 三、反馈实现
 *   原 wheel_leg_lqr_schedule.c -> 三、插值实现
 *   原 wheel_leg_lqr_default.c  -> 二、默认表
 *
 *   合并时消除的重复代码：
 *     * 原 wheel_leg_lqr.c 与 wheel_leg_lqr_schedule.c 各有一份逐字节相同的
 *       静态有限值判定函数，现已合并为单一宏 WHEEL_LEG_LQR_IS_FINITE_VALUE；
 *     * Calculate 中"状态向量与参考向量"两段结构相同的校验/求差循环已合并为
 *       一个循环，判失败时的清零动作也统一为同一段代码。
 *   函数名已统一改为可直接读出用途的形式（如 WheelLegLQR_Calculate ->
 *   WheelLegLQR_CalculateFeedbackOutput）；结构体名、返回语义与全部默认表
 *   数值保持与合并前一致，新旧名对照见 ../Chassis/README.md。
 */

#include "wheel_leg_lqr_merged.h"

#include <float.h>

/* ==========================================================================
 * 一、内部工具宏                                  [对内]
 * ========================================================================== */

/*
 * 有限值判定（内部工具）。
 *
 * 为什么用表达式而不是 isfinite()：
 *   保持不依赖 libm 的浮点分类函数，任意嵌入式 C 库都能编译。
 * 判定方式：value == value 排除 NaN，与 FLT_MAX 比较排除 ±Inf。
 *
 * 用法约定：参数必须是可多次求值且无副作用的表达式（本模块只传结构体成员、
 * 数组元素与已计算的局部变量）。
 */
#define WHEEL_LEG_LQR_IS_FINITE_VALUE(value)                                  \
    (((value) == (value) && (value) <= FLT_MAX && (value) >= -FLT_MAX)   \
         ? 1u                                                            \
         : 0u)

/* ==========================================================================
 * 二、内部数据：2026 默认调度表                    [对内]
 * ========================================================================== */

/*
 * 2026 机型默认 K 表节点数。对外隐藏：调用方只应通过
 * WheelLegLQR_Get2026DefaultSchedule() 取表，并用返回结构的 count 字段，
 * 不要把节点个数写死在业务代码里。
 */
#define WHEEL_LEG_LQR_2026_SCHEDULE_NODE_COUNT 11u

/*
 * 节点腿长（单位 m，升序）。
 * 区间端点与 FiveBar 默认几何的 min_length_m / max_length_m 对齐，
 * 即整条物理可达腿长区间都被覆盖，L0 不会落到表外。
 */
static const float wheel_leg_lqr_2026_leg_length_nodes_m[
    WHEEL_LEG_LQR_2026_SCHEDULE_NODE_COUNT] =
{
    0.1042f, 0.1305f, 0.1568f, 0.1831f, 0.2094f, 0.2357f,
    0.2620f, 0.2883f, 0.3146f, 0.3409f, 0.3672f
};

/*
 * 各节点对应的 K 矩阵 [2][6]：
 *   行 0 = T  的 6 个增益，行 1 = Tp 的 6 个增益；
 *   列序与状态向量一致 [theta, dtheta, xb, dxb, phi, dphi]。
 *
 * 该表为整定结果，属机器人配置，不是算法常量。更换机型、改动结构件
 * （连杆长度、质量分布、轮径）或重新整定后必须整体重新生成；使用前须复核。
 */
static const float wheel_leg_lqr_2026_gain_nodes[
    WHEEL_LEG_LQR_2026_SCHEDULE_NODE_COUNT]
    [WHEEL_LEG_LQR_OUTPUT_COUNT][WHEEL_LEG_LQR_STATE_COUNT] =
{
    {
        {-16.5684277f, -1.3288866f, -19.029955f, -13.2179515f,
          40.9654058f, 3.74741149f},
        { 14.0846636f,  1.29986917f,  23.482829f,  15.0198634f,
         119.078823f, 4.23297661f}
    },
    {
        {-17.575166f, -1.32897796f, -19.7414551f, -13.4561708f,
          36.6270845f, 3.44044843f},
        { 12.71275f, 1.08652886f, 21.0023761f, 13.1322188f,
         124.365587f, 4.71720645f}
    },
    {
        {-18.3378144f, -1.30995566f, -20.2292192f, -13.5660815f,
          33.1625956f, 3.15874556f},
        { 11.6452533f, 0.926434831f, 19.0555702f, 11.6959357f,
         127.99739f, 5.06836854f}
    },
    {
        {-18.9627573f, -1.2916596f, -20.6164251f, -13.6489503f,
          30.0971246f, 2.9122041f},
        { 10.6631856f, 0.795601593f, 17.3162371f, 10.4640172f,
         130.843952f, 5.34307472f}
    },
    {
        {-19.4852717f, -1.27573834f, -20.9293385f, -13.7153653f,
          27.3520728f, 2.69676246f},
        { 9.75139024f, 0.686162325f, 15.7432892f, 9.38494058f,
         133.120103f, 5.56212607f}
    },
    {
        {-19.9282666f, -1.26212581f, -21.1831331f, -13.768416f,
          24.8897111f, 2.5077627f},
        { 8.91007076f, 0.593451979f, 14.3212947f, 8.43292517f,
         134.952008f, 5.73881943f}
    },
    {
        {-20.3081736f, -1.25045824f, -21.3892916f, -13.8101499f,
          22.680786f, 2.34113564f},
        { 8.13884934f, 0.514197358f, 13.0381297f, 7.58995455f,
         136.432011f, 5.88266621f}
    },
    {
        {-20.6369999f, -1.24032f, -21.5570365f, -13.8423491f,
          20.6988863f, 2.19348042f},
        { 7.43518681f, 0.445933024f, 11.8817808f, 6.84157718f,
         137.631803f, 6.00074152f}
    },
    {
        {-20.9233321f, -1.23126397f, -21.693821f, -13.8665146f,
          18.9196333f, 2.06197015f},
        { 6.79495329f, 0.386730343f, 10.8403194f, 6.17560816f,
         138.607838f, 6.0984084f}
    },
    {
        {-21.1726684f, -1.22274194f, -21.8056308f, -13.8837217f,
          17.320892f, 1.94423498f},
        { 6.21317034f, 0.33503975f, 9.90241665f, 5.58166139f,
         139.404628f, 6.17977907f}
    },
    {
        {-21.386556f, -1.21386713f, -21.8972133f, -13.8942896f,
          15.8833735f, 1.83823374f},
        { 5.68456259f, 0.289570381f, 9.05804574f, 5.05104288f,
         140.057053f, 6.24802969f}
    }
};

/* 默认调度表视图：只读，指向上面两张表。 */
static const WheelLegLQRSchedule wheel_leg_lqr_2026_default_schedule =
{
    WHEEL_LEG_LQR_2026_SCHEDULE_NODE_COUNT,
    wheel_leg_lqr_2026_leg_length_nodes_m,
    wheel_leg_lqr_2026_gain_nodes
};

/* ==========================================================================
 * 三、公开接口实现                                [对外]
 * ========================================================================== */

uint8_t WheelLegLQR_CalculateFeedbackOutput(
    const float gain[WHEEL_LEG_LQR_OUTPUT_COUNT]
                [WHEEL_LEG_LQR_STATE_COUNT],
    const float x[WHEEL_LEG_LQR_STATE_COUNT],
    const float x_ref[WHEEL_LEG_LQR_STATE_COUNT],
    float output[WHEEL_LEG_LQR_OUTPUT_COUNT])
{
    float error[WHEEL_LEG_LQR_STATE_COUNT];
    uint8_t row;
    uint8_t column;

    /* 先处理输出：输出为空则无从写入，直接失败；否则立即清零。 */
    if (output == 0)
    {
        return 0u;
    }
    output[0] = 0.0f;
    output[1] = 0.0f;
    if (gain == 0 || x == 0 || x_ref == 0)
    {
        return 0u;
    }

    /*
     * 误差与校验一次完成：同一下标同时检查 x 与 x_ref，并顺带求出误差
     * (x_ref - x)。合并前这里是对 x 和 x_ref 各写一遍结构相同的循环。
     * state 维是编译期常量，循环次数固定，展开与否交给编译器。
     */
    for (column = 0u; column < WHEEL_LEG_LQR_STATE_COUNT; column++)
    {
        if (WHEEL_LEG_LQR_IS_FINITE_VALUE(x[column]) == 0u ||
            WHEEL_LEG_LQR_IS_FINITE_VALUE(x_ref[column]) == 0u)
        {
            return 0u;
        }
        error[column] = x_ref[column] - x[column];
    }

    /*
     * u = K * error。输出在进入本循环前已清零，故此处直接累加。
     * 增益表逐项校验：任一元素非有限值即整体失败，并把已经累加的部分清掉，
     * 避免把半成品输出留给调用方。
     */
    for (row = 0u; row < WHEEL_LEG_LQR_OUTPUT_COUNT; row++)
    {
        for (column = 0u; column < WHEEL_LEG_LQR_STATE_COUNT; column++)
        {
            if (WHEEL_LEG_LQR_IS_FINITE_VALUE(gain[row][column]) == 0u)
            {
                output[0] = 0.0f;
                output[1] = 0.0f;
                return 0u;
            }
            output[row] += gain[row][column] * error[column];
        }
    }
    return 1u;
}

uint8_t WheelLegLQR_GetGainsForLegLength(
    const WheelLegLQRSchedule *schedule,
    float l0_m,
    float gain[WHEEL_LEG_LQR_OUTPUT_COUNT]
                [WHEEL_LEG_LQR_STATE_COUNT])
{
    uint32_t lower;
    uint32_t upper;
    float alpha;
    uint32_t row;
    uint32_t column;

    /* 表与输出指针、以及查询点自身的有限性检查。 */
    if (schedule == 0 || gain == 0 || schedule->count == 0u ||
        schedule->l0_m == 0 || schedule->gain == 0 ||
        WHEEL_LEG_LQR_IS_FINITE_VALUE(l0_m) == 0u)
    {
        return 0u;
    }

    /*
     * 定位查询区间 [lower, upper]，并计算插值系数 alpha。
     * 表外一律取最近端点（alpha = 0），即不外推。
     */
    if (schedule->count == 1u)
    {
        /* 只有一个节点：与 l0 无关，恒取该节点。 */
        lower = 0u;
        upper = 0u;
    }
    else if (l0_m <= schedule->l0_m[0])
    {
        /* 低于表下界：夹到下端点。 */
        lower = 0u;
        upper = 0u;
    }
    else if (l0_m >= schedule->l0_m[schedule->count - 1u])
    {
        /* 高于表上界：夹到上端点。 */
        lower = schedule->count - 1u;
        upper = lower;
    }
    else
    {
        /* 区间内：线性查找第一个不低于 l0 的节点。
         * 节点数较少（默认 11），线性查找比二分更省代码且分支可预测；
         * 若将来节点数大幅增加，此处可换成二分。 */
        lower = 0u;
        while (lower + 1u < schedule->count &&
               l0_m > schedule->l0_m[lower + 1u])
        {
            lower++;
        }
        upper = lower + 1u;
        /* 表必须严格升序；发现非升序说明调用方表构造有误，拒绝插值。 */
        if (schedule->l0_m[upper] <= schedule->l0_m[lower])
        {
            return 0u;
        }
    }

    /* 端点重合（表外夹取或单节点）时不做插值，直接取该节点值。 */
    alpha = (upper == lower) ? 0.0f :
            (l0_m - schedule->l0_m[lower]) /
            (schedule->l0_m[upper] - schedule->l0_m[lower]);

    /* 逐元素线性插值：K = K_low + alpha * (K_high - K_low)。
     * 两个端点值都先做有限性检查，避免 NaN 混入后的传播。 */
    for (row = 0u; row < WHEEL_LEG_LQR_OUTPUT_COUNT; row++)
    {
        for (column = 0u; column < WHEEL_LEG_LQR_STATE_COUNT; column++)
        {
            float low = schedule->gain[lower][row][column];
            float high = schedule->gain[upper][row][column];
            if (WHEEL_LEG_LQR_IS_FINITE_VALUE(low) == 0u ||
                WHEEL_LEG_LQR_IS_FINITE_VALUE(high) == 0u)
            {
                return 0u;
            }
            gain[row][column] = low + alpha * (high - low);
        }
    }
    return 1u;
}

const WheelLegLQRSchedule *WheelLegLQR_Get2026DefaultSchedule(void)
{
    /* 返回模块内静态只读表；调用方不得修改其内容。 */
    return &wheel_leg_lqr_2026_default_schedule;
}
