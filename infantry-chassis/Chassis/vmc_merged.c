/*
 * ============================================================================
 * VMC -- 单腿二维虚拟模型控制映射模块（对外唯一源文件）
 * ============================================================================
 *
 * 配套头文件：vmc_merged.h（对外声明、前置条件与数学关系见该文件）
 *
 * 本文件结构（按"对外 / 对内"排布）
 * --------------------------------
 *   一、内部工具宏                  [对内]
 *   二、公开接口实现
 *       - VMC_ConvertWrenchToJointTorque          [对外]
 *
 * 说明：本模块足够小，除有限值判定外没有可提炼的内部逻辑；原实现对 4 个
 * Jacobian 元素、2 个虚拟力和输出指针各写了一次同义判定，已统一为下面的
 * 单一宏，避免多处判定在后续修改中走偏。
 */

#include "vmc_merged.h"

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
#define VMC_IS_FINITE_VALUE(value)                                            \
    (((value) == (value) && (value) <= FLT_MAX && (value) >= -FLT_MAX)   \
         ? 1u                                                            \
         : 0u)

/* ==========================================================================
 * 二、公开接口实现                                [对外]
 * ========================================================================== */

uint8_t VMC_ConvertWrenchToJointTorque(
    const float jacobian[2][2],
    float force_N,
    float pitch_torque_Nm,
    float joint_torque_Nm[2])
{
    /*
     * 先处理输出：输出为空则无从写入，直接失败；否则立即清零，
     * 使后续任何失败路径都留下确定的 {0, 0}。
     */
    if (joint_torque_Nm == 0)
    {
        return 0u;
    }
    joint_torque_Nm[0] = 0.0f;
    joint_torque_Nm[1] = 0.0f;

    /* 入参校验：空指针与非有限值一律拒绝，避免 NaN 沿力链路传播到电机。 */
    if (jacobian == 0 ||
        VMC_IS_FINITE_VALUE(force_N) == 0u ||
        VMC_IS_FINITE_VALUE(pitch_torque_Nm) == 0u ||
        VMC_IS_FINITE_VALUE(jacobian[0][0]) == 0u ||
        VMC_IS_FINITE_VALUE(jacobian[0][1]) == 0u ||
        VMC_IS_FINITE_VALUE(jacobian[1][0]) == 0u ||
        VMC_IS_FINITE_VALUE(jacobian[1][1]) == 0u)
    {
        return 0u;
    }

    /* tau = J^T * [F, Tp]：列取 J 的行、行取 J 的列。 */
    joint_torque_Nm[0] = jacobian[0][0] * force_N +
                         jacobian[1][0] * pitch_torque_Nm;
    joint_torque_Nm[1] = jacobian[0][1] * force_N +
                         jacobian[1][1] * pitch_torque_Nm;
    return 1u;
}
