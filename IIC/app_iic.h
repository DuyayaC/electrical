#ifndef APP_IIC_H
#define APP_IIC_H

#include "i2c.h"
#include "main.h"

#include <stdint.h>

/* ==================== 模块说明 ====================
 * 本模块只处理电池架 2 一个设备的数据（只读，DMA）。
 * 器件由 CubeMX 初始化的 I2C 句柄驱动，绑定一次即可反复读取。
 *
 * 用法（按功能顺序）：
 *   app_iic_init(&hi2c1);                                    // 1) 绑定 I2C
 *   app_iic_meas_async(&bat, app_iic_battery_callback, NULL); // 2) 发起 DMA 读取
 *   // 3) 回调触发后 bat 里就是物理量
 *   float v = (float)bat.voltage_mv / 1000.0f;
 * ================================================== */

/* ==================== 1. 类型定义 ==================== */

/* 器件参数（手册：从机地址 0x41(7bit)，速率 100 kHz） */
#define APP_IIC_BATTERY_ADDR        (0x41U << 1) /**< HAL 使用的 8bit 地址（含 R/W 位） */
#define APP_IIC_BATTERY_ADDR_7BIT   (0x41U)      /**< 手册给出的 7bit 地址 */
#define APP_IIC_RETRY               (2U)         /**< 启动失败时的重试次数 */

/* -------- 静态数据寄存器 -------- */
#define APP_IIC_REG_WHO_AM_I            (0x1FU) /**< uint8_t  器件 ID，固定 0x41 */
#define APP_IIC_REG_DESIGNED_CAPACITY   (0x20U) /**< uint32_t 设计容量 (mAh) */
#define APP_IIC_REG_LOOP_TIMES          (0x24U) /**< uint16_t 循环次数 */
#define APP_IIC_REG_PRODUCTION_DATE     (0x26U) /**< uint16_t 生产日期（位域） */
#define APP_IIC_REG_BATTERY_LIFE        (0x28U) /**< uint8_t  电池寿命 (0~100 %) */

/* -------- 动态数据寄存器 -------- */
#define APP_IIC_REG_CURRENT_VOLTAGE     (0x29U) /**< int32_t  瞬时电压 (mV) */
#define APP_IIC_REG_CURRENT_CURRENT     (0x2DU) /**< int32_t  瞬时电流 (mA) */
#define APP_IIC_REG_TEMPERATURE         (0x31U) /**< int16_t  瞬时温度 (精度 0.1 ℃) */
#define APP_IIC_REG_CAPACITY_PRECENT    (0x33U) /**< uint8_t  剩余电量 (0~100 %) */
#define APP_IIC_REG_INTERNAL_STATE      (0x34U) /**< uint8_t  内部状态（位域） */
#define APP_IIC_REG_ERROR_STATE         (0x35U) /**< uint8_t  错误状态（位域） */

/** 一次连续读取的首寄存器与长度：0x1F ~ 0x35 共 23 字节（手册中地址连续） */
#define APP_IIC_BLOCK_FIRST_REG         APP_IIC_REG_WHO_AM_I
#define APP_IIC_BLOCK_LEN               ((uint16_t)((APP_IIC_REG_ERROR_STATE - APP_IIC_REG_WHO_AM_I) + 1U))

/** WHO_AM_I 期望值 */
#define APP_IIC_WHO_AM_I_VALUE          (0x41U)

/* 位域：INTERNAL_STATE (0x34) */
#define APP_IIC_INTERNAL_OUTPUT_Pos          (0U)
#define APP_IIC_INTERNAL_OUTPUT_Msk          (0x01U << APP_IIC_INTERNAL_OUTPUT_Pos)
#define APP_IIC_INTERNAL_OUTPUT_HIGH         (0x01U) /**< 大电流(>0.2A)，不会自动断输出 */
#define APP_IIC_INTERNAL_OUTPUT_LOW          (0x00U) /**< 0.2A 以下，开机 20 分钟后自动断输出 */

#define APP_IIC_INTERNAL_COMM_Pos            (1U)
#define APP_IIC_INTERNAL_COMM_Msk            (0x01U << APP_IIC_INTERNAL_COMM_Pos)
#define APP_IIC_INTERNAL_COMM_NORMAL         (0x01U) /**< 1: 通信正常 */
#define APP_IIC_INTERNAL_COMM_ABNORMAL       (0x00U) /**< 0: 异常（电池未装好/未开机） */

/* 位域：ERROR_STATE (0x35)，1 表示发生该错误 */
#define APP_IIC_ERROR_SHORT_Pos              (0U) /**< 放电短路 */
#define APP_IIC_ERROR_OVER_DISCHARGE_Pos     (1U) /**< 过放 */
#define APP_IIC_ERROR_CURRENT_EXCEED_Pos     (2U) /**< 电流超限 */
#define APP_IIC_ERROR_OVERHEAT_Pos           (3U) /**< 电池过热 */
#define APP_IIC_ERROR_VOLTAGE_LOW_Pos        (4U) /**< 电池电压过低 */
#define APP_IIC_ERROR_CELL_Pos               (5U) /**< 电芯错误 */
#define APP_IIC_ERROR_SELF_CHECK_Pos         (6U) /**< 自检错误 */
#define APP_IIC_ERROR_RESERVED_Pos           (7U) /**< 保留 */

#define APP_IIC_ERROR_SHORT_Msk              (0x01U << APP_IIC_ERROR_SHORT_Pos)
#define APP_IIC_ERROR_OVER_DISCHARGE_Msk     (0x01U << APP_IIC_ERROR_OVER_DISCHARGE_Pos)
#define APP_IIC_ERROR_CURRENT_EXCEED_Msk     (0x01U << APP_IIC_ERROR_CURRENT_EXCEED_Pos)
#define APP_IIC_ERROR_OVERHEAT_Msk           (0x01U << APP_IIC_ERROR_OVERHEAT_Pos)
#define APP_IIC_ERROR_VOLTAGE_LOW_Msk        (0x01U << APP_IIC_ERROR_VOLTAGE_LOW_Pos)
#define APP_IIC_ERROR_CELL_Msk               (0x01U << APP_IIC_ERROR_CELL_Pos)
#define APP_IIC_ERROR_SELF_CHECK_Msk         (0x01U << APP_IIC_ERROR_SELF_CHECK_Pos)

/** PRODUCTION_DATE 位域：15:9 年(相对 1980) / 8:5 月 / 4:0 日 */
#define APP_IIC_DATE_YEAR_OFFSET             (1980U)
#define APP_IIC_DATE_GET_YEAR(raw)           ((uint16_t)(((raw) >> 9) + APP_IIC_DATE_YEAR_OFFSET))
#define APP_IIC_DATE_GET_MONTH(raw)          ((uint8_t)(((raw) >> 5) & 0x0FU))
#define APP_IIC_DATE_GET_DAY(raw)            ((uint8_t)((raw) & 0x1FU))

/** 驱动状态码 */
typedef enum
{
    APP_IIC_OK = 0,        /**< 成功 / 已启动 */
    APP_IIC_BUSY,          /**< DMA 传输进行中 */
    APP_IIC_ERROR_PARAM,   /**< 参数错误 */
    APP_IIC_ERROR_NACK,    /**< 从机无应答（未接 / 未开机 / 地址错误） */
    APP_IIC_ERROR_TIMEOUT, /**< 总线超时 */
    APP_IIC_ERROR_BUS,     /**< 总线错误 (BERR/ARLO) */
    APP_IIC_ERROR_HAL,     /**< 其它 HAL 错误（含 DMA 未配置） */
    APP_IIC_ERROR_ID,      /**< WHO_AM_I 校验失败 */
} app_iic_status_e;

/** 电池架数据（已换算为物理量） */
typedef struct
{
    uint8_t  who_am_i;          /**< 0x1F  器件 ID，正常为 0x41 */
    uint32_t designed_capacity; /**< 0x20  设计容量 (mAh) */
    uint16_t loop_times;        /**< 0x24  充电循环次数 */
    uint16_t production_date;   /**< 0x26  生产日期原始值 */
    uint8_t  battery_life;      /**< 0x28  电池寿命 (0~100 %) */

    int32_t  voltage_mv;        /**< 0x29  电压 (mV) */
    int32_t  current_ma;        /**< 0x2D  电流 (mA) */
    int16_t  temperature_raw;   /**< 0x31  温度原始值 (0.1 ℃) */
    float    temperature_c;     /**<        温度 (℃) */
    uint8_t  capacity_percent;  /**< 0x33  剩余电量 (0~100 %) */
    uint8_t  internal_state;    /**< 0x34  内部状态位域 */
    uint8_t  error_state;       /**< 0x35  错误状态位域 */

    uint16_t year;              /**<        生产年份（由 production_date 解析） */
    uint8_t  month;             /**<        生产月份 */
    uint8_t  day;               /**<        生产日 */
} app_iic_battery_data_t;

/** 上层回调：一次读取完成后调用（在 DMA 完成中断上下文里，不要放耗时操作） */
typedef void (*app_iic_callback_t)(app_iic_status_e status, app_iic_battery_data_t *data, void *user_ctx);

/* ==================== 2. 函数声明 ==================== */

/**
  * @brief          绑定 I2C 句柄（初始化）
  * @param[in]      hi2c: 已由 CubeMX 初始化好的 I2C 句柄（如 &hi2c1）
  * @retval         none
  * @note           要求 CubeMX 中已为本 I2C 添加 I2Cx_RX 的 DMA 通道；
  *                 传 NULL 或重复调用只是重新绑定，不报错。
  */
extern void app_iic_init(I2C_HandleTypeDef *hi2c);

/**
  * @brief          读取全部数据（DMA，非阻塞）
  * @param[out]     data: 结果缓存，由调用者提供，生命周期需保持到回调触发
  * @param[in]      cb:   完成回调，可为 NULL（数据在回调前已解析进 data）
  * @param[in]      user_ctx: 回调透传参数
  * @retval         APP_IIC_OK 传输已启动；其它为立即失败的原因
  * @note           回调用前会校验 WHO_AM_I，不是 0x41 则回调收到 APP_IIC_ERROR_ID。
  */
extern app_iic_status_e app_iic_meas_async(app_iic_battery_data_t *data,
                                           app_iic_callback_t cb, void *user_ctx);

/**
  * @brief          读取任意寄存器（DMA，非阻塞）
  * @param[in]      reg: 寄存器地址 (8bit)
  * @param[out]     buf: 目标缓存
  * @param[in]      len: 读取字节数（本器件单次最长 255 字节）
  * @param[in]      cb:  完成回调，可为 NULL
  * @param[in]      user_ctx: 回调透传参数
  * @retval         APP_IIC_OK 传输已启动；其它为立即失败的原因
  * @note           电池架所有寄存器都是只读的，因此没有写接口
  */
extern app_iic_status_e app_iic_reg_async(uint8_t reg, uint8_t *buf, uint16_t len,
                                          app_iic_callback_t cb, void *user_ctx);

/**
  * @brief          一键读取 WHO_AM_I（DMA，非阻塞；在线检测用）
  * @param[out]     id:  读出的器件 ID
  * @param[in]      cb:  完成回调，可为 NULL
  * @param[in]      user_ctx: 回调透传参数
  * @retval         APP_IIC_OK 传输已启动；其它为立即失败的原因
  */
extern app_iic_status_e app_iic_who_am_i_async(uint8_t *id,
                                               app_iic_callback_t cb, void *user_ctx);

/* ---------- 中断回调（HAL 转发：把 HAL 的 I2C 完成/错误事件交给本模块） ---------- */

/**
  * @brief          转发 HAL_I2C_MemRxCpltCallback（读完成）
  * @param[in]      hi2c: HAL 传入的 I2C 句柄
  * @retval         none
  * @note           在 main.c / stm32h7xx_it.c 里实现：
  *                   void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
  *                   { app_iic_on_i2c_cplt(hi2c); }
  */
extern void app_iic_on_i2c_cplt(I2C_HandleTypeDef *hi2c);

/**
  * @brief          转发 HAL_I2C_ErrorCallback（I2C 错误）
  * @param[in]      hi2c: HAL 传入的 I2C 句柄
  * @retval         none
  * @note           在 main.c / stm32h7xx_it.c 里实现：
  *                   void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
  *                   { app_iic_on_i2c_error(hi2c); }
  */
extern void app_iic_on_i2c_error(I2C_HandleTypeDef *hi2c);

/* ---------- 解包函数（设备取数：把原始字节解析成物理量） ---------- */

/**
  * @brief          解包电池架数据（23 字节原始数据 -> 物理量）
  * @param[in]      raw:  长度 APP_IIC_BLOCK_LEN、首字节为寄存器 0x1F 的原始缓存
  * @param[out]     data: 目标结构体
  * @retval         none
  * @note           结构体带对齐填充，不能直接当 DMA 缓存用，模块内部用紧凑 raw 缓存接收，
  *                 传输完成后自动调用本函数。想自己收原始数据时也可以直接用这个接口。
  */
extern void app_iic_decode_battery(const uint8_t *raw, app_iic_battery_data_t *data);

/**
  * @brief          return the error state string（错误状态位解码）
  * @param[in]      error_state: APP_IIC_REG_ERROR_STATE 读回的值
  * @retval         人类可读字符串（静态常量，不要 free）
  */
extern const char *app_iic_get_error_string(uint8_t error_state);

/* ==================== CubeMX 配置提醒 ====================
 * 1) I2C: 100 kHz / 7bit 地址 / 从机地址 0x41
 * 2) I2C -> DMA Settings:
 *      - 只需要添加 I2Cx_RX（Peripheral To Memory）—— HAL_I2C_Mem_Read_DMA 只用 hdmarx，
 *        没有它函数会直接返回 HAL_ERROR 并置 HAL_I2C_ERROR_DMA_PARAM。
 *        因为只读不写，I2Cx_TX 这一路可以不加。
 * 3) NVIC: DMA 通道中断 + I2C event/error 中断都要使能。
 * 4) 硬件: SDA/SCL 各 4.7 kΩ 上拉到 3.3 V（电池架内部无上拉）。
 * ======================================================== */

#endif
