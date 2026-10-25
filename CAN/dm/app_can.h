#ifndef APP_CAN_H
#define APP_CAN_H

#include "fdcan.h"
#include "main.h"

/* ==================== 文件说明（dm 版：STM32H7 FDCAN） ====================
 * 本文件与 dji/app_can.h 是同一套设备接口的两个平台版本：
 *   dji 版：STM32F4 经典 bxCAN（can.h / HAL_CAN_xxx）
 *   dm  版：STM32H7 FDCAN（fdcan.h / HAL_FDCAN_xxx）
 *
 * 对外接口约定：函数名、参数个数/顺序、返回值与 dji 版保持一致；类型名按平台
 * 用对应 HAL 类型——dm 版用 FDCAN_HandleTypeDef / FDCAN_FilterTypeDef。所以从
 * dji 版换到 dm 版时：函数调用代码不用改，滤波器/句柄等变量声明改成 FDCAN 类型即可
 * （例如 CAN_FilterTypeDef filter; -> FDCAN_FilterTypeDef filter;）。
 *
 * 另外 dm 版额外支持达妙（DM）电机的 MIT 模式控制帧与反馈帧解析（第 2 节），
 * DJI 相关接口不受影响（dm 头是 dji 头的超集）。
 *
 * 使用 FDCAN 前要知道的几点（详见 app_can.c 文件头）：
 *   1. 滤波器按实例独立：FDCAN1、FDCAN2 各自有一份标准 ID 滤波器列表，
 *      数量由 CubeMX 里 "Std Filters Nbr" 决定。bank 参数因此是
 *      "该实例内的滤波器序号"；为兼容 dji 工程的调用代码，CAN2 按 dji 版
 *      惯例写 14~27 时会被自动折算成 FDCAN2 的 0~13。
 *   2. DJI 电机/云台/拨盘的反馈帧和控制帧都是经典 CAN（Classic CAN）
 *      8 字节标准帧；本模块发送时显式指定 FDCAN_CLASSIC_CAN + FDCAN_BRS_OFF，
 *      与总线是否配置成 CAN FD 无关。
 *   3. 本模块初始化时会配置全局滤波器，丢弃不匹配的标准帧/扩展帧，
 *      与 dji 版"只有滤波器命中的帧才进接收回调"的行为一致。
 *   4. 工程集成注意（重复的 HAL_FDCAN_RxFifo0Callback 符号、初始化顺序、
 *      FDCAN3 的处理）见 app_can.c 文件头"移植注意"。
 * ====================================================================== */

/* ==================== 运行时设备配置表 ====================
 * 每个设备一条配置记录（挂载的 FDCAN 总线句柄 + 发送 ID），
 * 运行时可改 .hcan 换总线 -> 换板子/改接线无需改代码。
 *
 * 用法（按功能顺序）：
 *   // 1) 初始化滤波器（必须先调用；配置结构体在外面创建，can_filter_std() 填充，
 *   //    完整流程见 app_can.c 文件头的"FDCAN 滤波配置使用说明"）
 *   FDCAN_FilterTypeDef filter_can1, filter_can2;
 *   can_filter_std(&filter_can1, 0x000, 0x000, 0);    // FDCAN1 全接收, 序号 0
 *   can_filter_std(&filter_can2, 0x200, 0x7F8, 14);   // FDCAN2 收 0x200~0x207, 序号 14 自动折算为 0
 *   app_can1_filter_init(&filter_can1);
 *   app_can2_filter_init(&filter_can2);
 *
 *   // 2) 读取电机反馈（bus 由用户指定；该设备不在该总线时返回 NULL）
 *   const motor_measure_t *m = get_chassis_motor_measure_point(APP_CAN2, 0);
 *
 *   // 3) 按设备配置发送（总线可运行时切换）
 *   app_can_cmd_chassis(&can_dev_chassis, m1, m2, m3, m4);
 *   app_can_cmd_yaw(&can_dev_yaw, val);
 *   app_can_cmd_pitch(&can_dev_pitch, val);
 *   app_can_cmd_shoot(&can_dev_shoot, m1, m2, shoot);
 *
 *   // 4) DM（达妙）电机：初始化槽位 -> 使能 -> 发控制帧 -> 读反馈（详见第 2 节）
 *   app_dm_motor_init(&app_dm_motor[0], &hfdcan1, 1, 0);      // CAN ID=1, Master ID=0
 *   app_dm_enable(&app_dm_motor[0], APP_DM_MODE_MIT);         // 使能并进入 MIT 模式
 *   app_dm_cmd_mit(&app_dm_motor[0], 0.0f, 0.0f, 30.0f, 1.0f, 0.0f);
 *   // 位置速度模式：app_dm_enable(&app_dm_motor[0], APP_DM_MODE_POS_VEL);
 *   //               app_dm_cmd_pos_vel(&app_dm_motor[0], 1.0f, 0.0f);
 *   const dm_motor_measure_t *dm = app_dm_get_measure(&app_dm_motor[0]);
 * ============================================================== */

/* ==================== 1. 类型定义 ==================== */

/* 电机反馈数据结构（存放解包后的 ecd / speed_rpm / given_current 等字段） */
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
    uint8_t error;
} motor_measure_t __attribute__((aligned(4)));

/* CAN 发送/接收标准帧 ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x203,
    CAN_GIMBAL_ALL_ID = 0x1FE,

} can_msg_id_e;

/* 设备槽位下标 = 标准帧 ID - 0x201（0x201~0x208 -> 0~7），两条 CAN 线相同，无需查表 */
enum
{
    APP_IDX_M1   = CAN_3508_M1_ID       - CAN_3508_M1_ID, /* 0x201 -> 0 */
    APP_IDX_M2   = CAN_3508_M2_ID       - CAN_3508_M1_ID, /* 0x202 -> 1 */
    APP_IDX_M3   = CAN_3508_M3_ID       - CAN_3508_M1_ID, /* 0x203 -> 2 */
    APP_IDX_M4   = CAN_3508_M4_ID       - CAN_3508_M1_ID, /* 0x204 -> 3 */
    APP_IDX_YAW  = CAN_YAW_MOTOR_ID     - CAN_3508_M1_ID, /* 0x205 -> 4 */
    APP_IDX_PIT  = CAN_PIT_MOTOR_ID     - CAN_3508_M1_ID, /* 0x206 -> 5 */
    APP_IDX_TRIG = CAN_TRIGGER_MOTOR_ID - CAN_3508_M1_ID, /* 0x203 -> 2 */
};

/* CAN 总线枚举（dm 版对应 FDCAN1 / FDCAN2） */
typedef enum
{
    APP_CAN1 = 0,
    APP_CAN2 = 1,
} app_can_bus_e;

/* 设备配置记录（挂载的 CAN 总线句柄 + 发送 ID）。
 * 字段名保持 hcan（与 dji 版一致）；类型是 FDCAN 句柄，赋值时直接用
 * &hfdcan1 / &hfdcan2 即可。 */
typedef struct
{
    FDCAN_HandleTypeDef *hcan;  /* 设备挂载的 FDCAN 总线句柄 */
    uint32_t           std_id;  /* 发送标准帧 ID */
} can_tx_device_t;

/* ==================== 2. DM 电机（达妙）协议 —— MIT / 位置速度模式 ====================
 * 对应达妙调试助手里的"控制报文 / 反馈帧"（经典 CAN、标准帧、8 字节）：
 *
 *   控制帧（MIT 模式）    帧 ID = CAN ID，8 字节
 *       D[0..1] = p_des（16 位，高字节在前）
 *       D[2]    = v_des[11:4]
 *       D[3]    = v_des[3:0]<<4 | Kp[11:8]
 *       D[4]    = Kp[7:0]
 *       D[5]    = Kd[11:4]
 *       D[6]    = Kd[3:0]<<4 | t_ff[11:8]
 *       D[7]    = t_ff[7:0]
 *   控制帧（位置速度模式）帧 ID = 0x100 + CAN ID，8 字节
 *       D[0..3] = p_des（float，低位在前）  D[4..7] = v_des（float，低位在前）
 *   反馈帧                帧 ID = 该电机的 Master ID（MST_ID，调试助手设置，默认 0），8 字节
 *       D[0]    = 控制器 ID（低 4 位）| 故障码（高 4 位）
 *       D[1..2] = POS[15:0]     D[3] = VEL[11:4]
 *       D[4]    = VEL[3:0]<<4 | T[11:8]     D[5] = T[7:0]
 *       D[6]    = T_MOS（℃）    D[7] = T_Rotor（℃）
 *
 * 控制帧里的 p_des / v_des / t_ff 是"线性映射的有符号定点数（补码）"：
 *   0 对应码值 0，正/负满量程分别对应 +/-(2^(N-1)-1)，负值就是负码；
 *   位置 16 位（-32767~+32767），速度/扭矩 12 位（-2047~+2047）。
 *   Kp / Kd 仍是无符号 12 位（0~500 / 0~5）。
 * 本模块默认范围按 DM4310-2EC（P ±12.5 rad、V ±30 rad/s、T ±10 N·m）；
 * 8009P 请按调试助手里的 P_MAX / V_MAX / T_MAX（对称）改成一致，
 * 即改 app_dm_motor[i] 里的 p_max / v_max / t_max（p_min 等保留为 -max）。
 *
 * 反馈帧里的 POS / VEL / T 同样是补码有符号数，解包时按 0 对应 0 还原。
 *
 * 反馈分发规则（接收回调里自动处理）：
 *   - mst_id != 0：帧 ID == mst_id 的反馈帧交给该电机（多台电机共用同一个非 0
 *     mst_id 时只有数组里第一个匹配到的电机会收到，请给它们设置不同 mst_id）；
 *   - mst_id == 0（默认）：帧 ID == 0 时按 D[0] 低 4 位 == CAN ID 低 4 位匹配，
 *     此时多台电机共用默认 Master ID，必须保证 CAN ID 低 4 位互不相同。
 *   另外：Master ID 不要设在 0x201~0x208。经典帧会先按 DJI 电机解包；如果是
 *   FD 格式的 8 字节帧则会绕过 DJI 判断被当成 DM 反馈（不建议这样用）。
 * ============================================================ */

/* 反馈帧 D[0] 高 4 位的故障码直接按原始数值读取（不做字符串转换，
 * 包里是多少就是多少，例如 0x8/0x9/0xA/0xB）。 */

/* 控制帧 ID 的模式偏移 */
#define APP_DM_MODE_MIT      ((uint16_t)0x000U)  /* MIT 模式：帧 ID = CAN ID */
#define APP_DM_MODE_POS_VEL  ((uint16_t)0x100U)  /* 位置速度模式：帧 ID = 0x100 + CAN ID */

/* 默认映射范围（DM4310-2EC；换 8009P 时把 *_max 改成调试助手里的
 * P_MAX / V_MAX / T_MAX，有符号编码只使用 *_max，*_min 保留为 -max） */
#define APP_DM_DEFAULT_P_MIN   (-12.5f)
#define APP_DM_DEFAULT_P_MAX   ( 12.5f)
#define APP_DM_DEFAULT_V_MIN   (-30.0f)
#define APP_DM_DEFAULT_V_MAX   ( 30.0f)
#define APP_DM_DEFAULT_T_MIN   (-10.0f)
#define APP_DM_DEFAULT_T_MAX   ( 10.0f)
#define APP_DM_DEFAULT_KP_MIN  (0.0f)
#define APP_DM_DEFAULT_KP_MAX  (500.0f)
#define APP_DM_DEFAULT_KD_MIN  (0.0f)
#define APP_DM_DEFAULT_KD_MAX  (5.0f)

/* 本模块管理的 DM 电机槽位数（按需改大；每个槽位约 80 字节 RAM，8 槽约 640B）。
 * 注意：该宏必须在所有编译单元里保持一致（建议在工程公共头文件里统一 define），
 * 否则 app_dm_motor[] 的 extern 尺寸会和定义不一致。 */
#ifndef APP_DM_MOTOR_NUM
#define APP_DM_MOTOR_NUM 8
#endif

/* DM 电机反馈数据（反馈帧解包结果） */
typedef struct
{
    float    pos;       /* 位置（rad） */
    float    vel;       /* 速度（rad/s） */
    float    tor;       /* 转矩（N·m） */
    float    t_mos;     /* 驱动板 MOS 平均温度（℃） */
    float    t_rotor;   /* 电机线圈平均温度（℃） */
    uint16_t pos_int;   /* 位置原始码值（16 位补码） */
    uint16_t vel_int;   /* 速度原始码值（12 位补码） */
    uint16_t tor_int;   /* 转矩原始码值（12 位补码） */
    uint8_t  id;        /* 控制器 ID（反馈帧 D[0] 低 4 位） */
    uint8_t  error;     /* 故障码：直接取反馈帧 D[0] 高 4 位的原始值（0xA/0xB 就保留原值），
                           不做字符串/含义转换，和 dji 版的 error 字段用法一致 */
    volatile uint8_t updated;  /* 收到过反馈后置 1（中断里写，主循环可轮询；只由 init 清零） */
} dm_motor_measure_t;

/* 一台 DM 电机的配置 + 反馈 */
typedef struct
{
    FDCAN_HandleTypeDef *hfdcan;  /* 挂载的 FDCAN 总线句柄 */
    uint16_t can_id;              /* 电机 CAN ID（0 = 该槽位未使用） */
    uint16_t mst_id;              /* 反馈帧 ID（Master ID，调试助手设置，默认 0） */
    float    p_min;               /* 位置范围（有符号编码只用 p_max 做满量程，p_min 默认 = -p_max） */
    float    p_max;
    float    v_min;               /* 速度范围（同上） */
    float    v_max;
    float    t_min;               /* 转矩范围（同上） */
    float    t_max;
    float    kp_min;              /* MIT 模式 Kp 范围（无符号 0~500） */
    float    kp_max;
    float    kd_min;              /* MIT 模式 Kd 范围（无符号 0~5） */
    float    kd_max;
    dm_motor_measure_t measure;   /* 反馈数据（只读） */
} dm_motor_t;

/* DM 电机槽位表：用 app_dm_motor_init() 填 hfdcan / can_id / mst_id / 范围 */
extern dm_motor_t app_dm_motor[APP_DM_MOTOR_NUM];

/**
  * @brief          初始化一台 DM 电机（填默认映射范围、绑定总线与 ID）
  * @param[in,out]  motor:  电机配置指针
  * @param[in]      hfdcan: 挂载的 FDCAN 句柄（&hfdcan1 / &hfdcan2）
  * @param[in]      can_id: 电机 CAN ID（调试助手里设的 ID；0 = 不使用该槽位；
  *                         必须 ≤ 0x7FF，超范围按未使用处理）
  * @param[in]      mst_id: 反馈帧 ID（Master ID，调试助手设置，默认 0）
  * @retval         none
  */
extern void app_dm_motor_init(dm_motor_t *motor, FDCAN_HandleTypeDef *hfdcan,
                              uint16_t can_id, uint16_t mst_id);

/**
  * @brief          使能 DM 电机（数据 0xFF...FF,0xFC，帧 ID = CAN ID + mode）
  * @param[in]      motor: 电机配置指针
  * @param[in]      mode:  目标模式 APP_DM_MODE_MIT / APP_DM_MODE_POS_VEL
  * @retval         none
  */
extern void app_dm_enable(dm_motor_t *motor, uint16_t mode);

/**
  * @brief          失能 DM 电机（数据 0xFF...FF,0xFD，帧 ID = CAN ID + mode）
  * @param[in]      motor: 电机配置指针
  * @param[in]      mode:  目标模式 APP_DM_MODE_MIT / APP_DM_MODE_POS_VEL
  * @retval         none
  */
extern void app_dm_disable(dm_motor_t *motor, uint16_t mode);

/**
  * @brief          MIT 模式控制（帧 ID = CAN ID）
  * @param[in]      motor: 电机配置指针
  * @param[in]      p_des: 位置给定（rad，范围 motor->p_min ~ motor->p_max）
  * @param[in]      v_des: 速度给定（rad/s，范围 motor->v_min ~ motor->v_max）
  * @param[in]      kp:    位置比例系数（范围 motor->kp_min ~ motor->kp_max）
  * @param[in]      kd:    位置微分系数（范围 motor->kd_min ~ motor->kd_max）
  * @param[in]      t_ff:  转矩前馈（N·m，范围 motor->t_min ~ motor->t_max）
  * @retval         none
  */
extern void app_dm_cmd_mit(dm_motor_t *motor, float p_des, float v_des, float kp, float kd, float t_ff);

/**
  * @brief          位置速度模式控制（帧 ID = 0x100 + CAN ID）
  * @param[in]      motor: 电机配置指针
  * @param[in]      p_des: 位置给定（rad，float 低位在前）
  * @param[in]      v_des: 速度给定（rad/s，float 低位在前）
  * @retval         none
  */
extern void app_dm_cmd_pos_vel(dm_motor_t *motor, float p_des, float v_des);

/**
  * @brief          读取 DM 电机反馈数据（只读指针）
  * @param[in]      motor: 电机配置指针
  * @retval         反馈数据指针；motor 为 NULL 时返回 NULL
  */
extern const dm_motor_measure_t *app_dm_get_measure(const dm_motor_t *motor);

/* ==================== 3. 变量声明 ==================== */

/* 设备配置表实例（默认值见 app_can.c，运行时可重新赋值 .hcan） */
extern can_tx_device_t can_dev_chassis;   /* 底盘 3508 四合一 */
extern can_tx_device_t can_dev_friction;  /* 摩擦轮 */
extern can_tx_device_t can_dev_yaw;       /* yaw 6020 */
extern can_tx_device_t can_dev_pitch;     /* pitch 6020 */
extern can_tx_device_t can_dev_shoot;     /* 拨盘 2006 */

/* ==================== 4. 函数声明 ==================== */
/**
  * @brief          生成标准帧滤波器配置（填充外部创建的结构体，不创建）
  * @param[out]     filter: 外部创建的滤波器配置结构体指针
  * @param[in]      id:    期望接收的标准帧 ID（如 0x201）
  * @param[in]      mask:  掩码，位为 1 表示"必须匹配该位"，位为 0 表示"该位不关心"
  * @param[in]      bank:  FDCAN 实例内的滤波器序号（0 ~ StdFiltersNbr-1）；
  *                        CAN2 按 dji 版惯例写 14~27 时由 app_can2_filter_init 折算
  * @retval         none
  */
extern void can_filter_std(FDCAN_FilterTypeDef *filter, uint32_t id, uint32_t mask, uint32_t bank);

/**
  * @brief          (方案C) 配置并启动 FDCAN1
  * @param[in]      filter: 外部创建并填充好的滤波器配置指针
  * @retval         none
  */
extern void app_can1_filter_init(FDCAN_FilterTypeDef *filter);

/**
  * @brief          (方案C) 配置并启动 FDCAN2
  * @param[in]      filter: 外部创建并填充好的滤波器配置指针
  * @retval         none
  */
extern void app_can2_filter_init(FDCAN_FilterTypeDef *filter);

/**
  * @brief          (方案C) 按设备配置发送底盘四电机电流
  * @param[in]      dev: 设备配置指针, 电流范围 [-16384,16384]
  * @retval         none
  */
extern void app_can_cmd_chassis(can_tx_device_t *dev, int16_t m1, int16_t m2, int16_t m3, int16_t m4);

/**
  * @brief          (方案C) 按设备配置发送 yaw 电流
  * @param[in]      dev: 设备配置指针, 电流范围 [-30000,30000]
  * @retval         none
  */
extern void app_can_cmd_yaw(can_tx_device_t *dev, int16_t yaw);

/**
  * @brief          (方案C) 按设备配置发送 pitch 电流
  * @param[in]      dev: 设备配置指针, 电流范围 [-30000,30000]
  * @retval         none
  */
extern void app_can_cmd_pitch(can_tx_device_t *dev, int16_t pitch);

/**
  * @brief          (方案C) 按设备配置发送摩擦轮 + 拨盘
  * @param[in]      dev:   设备配置指针
  * @param[in]      m1:    摩擦轮电机 1（3508），电流范围 [-16384,16384]
  * @param[in]      m2:    摩擦轮电机 2（3508），电流范围 [-16384,16384]
  * @param[in]      shoot: 拨盘电机；若为 M3508 电流范围 [-16384,16384]，若为 2006 则 [-10000,10000]
  * @retval         none
  */
extern void app_can_cmd_shoot(can_tx_device_t *dev, int16_t m1, int16_t m2, int16_t shoot);

/* ---------- 解包函数（设备取数：返回指定总线上对应设备的电机数据，槽位 = ID - 0x201） ---------- */

/**
  * @brief          return the yaw 6020 motor data point on the given bus
  * @param[in]      bus: APP_CAN1 / APP_CAN2
  * @retval         motor data point（未收到帧时为全 0 数据）
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(app_can_bus_e bus);

/**
  * @brief          return the pitch 6020 motor data point on the given bus
  * @param[in]      bus: APP_CAN1 / APP_CAN2
  * @retval         motor data point（未收到帧时为全 0 数据）
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(app_can_bus_e bus);

/**
  * @brief          return the trigger 2006 motor data point on the given bus
  * @param[in]      bus: APP_CAN1 / APP_CAN2
  * @retval         motor data point（未收到帧时为全 0 数据）
  */
extern const motor_measure_t *get_trigger_motor_measure_point(app_can_bus_e bus);

/**
  * @brief          return the chassis 3508 motor data point on the given bus
  * @param[in]      bus: APP_CAN1 / APP_CAN2
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point（未收到帧时为全 0 数据）
  */
extern const motor_measure_t *get_chassis_motor_measure_point(app_can_bus_e bus, uint8_t i);

/**
  * @brief          return the friction 3508 motor data point on the given bus
  * @param[in]      bus: APP_CAN1 / APP_CAN2
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point（未收到帧时为全 0 数据）
  */
extern const motor_measure_t *get_friction_motor_measure_point(app_can_bus_e bus, uint8_t i);

#endif
