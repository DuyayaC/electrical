/**
 * @file          app_iic.c
 * @brief         电池架 2 的 I2C 数据读取（只读，DMA）
 *
 * 自包含模块：仅依赖 STM32 HAL 的 i2c.h/main.h，只处理电池架一个设备。
 *
 * 器件：RoboMaster 电池架 2
 *   从机地址 0x41(7bit)，速率 100 kHz，寄存器地址 8bit，数据小端。
 *   10 个寄存器在地址空间上连续（0x1F ~ 0x35，共 23 字节），
 *   因此一次「发寄存器地址 + 连读 23 字节」就能拿回全部数据，DMA 只搬一趟。
 *   所有寄存器都是只读的，所以本模块只有读取接口，没有写接口。
 *
 * 用到的 HAL 库函数
 *   HAL_I2C_Mem_Read_DMA()              读寄存器（非阻塞，DMA 搬运）
 *   HAL_I2C_DeInit() / HAL_I2C_Init()   仅用于出错后复位外设
 *
 * 为什么需要 app_iic_raw
 *   app_iic_battery_data_t 里有对齐填充（uint8_t 后面紧跟 uint32_t 会插 3 字节空洞），
 *   直接把 DMA 指向该结构体会让「寄存器顺序」和「结构体偏移」错位。
 *   所以 DMA 永远落到紧凑的 app_iic_raw[23]，完成后由 app_iic_decode_battery() 解析。
 */

#include "app_iic.h"
#include <string.h>

/* ==================== 0. 本文件函数一览 ====================
 * 分组            函数                          作用
 * 内部工具        app_iic_rd_u16/u32            小端字节拼接
 *                 app_iic_hal_to_status()       HAL 返回值 -> 本模块状态码
 *                 app_iic_recover()             出错后复位 I2C 外设
 *                 app_iic_read_dma()            启动一次 DMA 读
 *                 app_iic_finish_meas()         整块读取收尾（解包 + 校验 + 回调）
 *                 app_iic_finish_reg()          单寄存器读取收尾（回调）
 * 初始化          app_iic_init()                绑定 I2C 句柄
 * 读取接口        app_iic_meas_async()          读全部数据（23 字节）
 *                 app_iic_reg_async()           读任意寄存器
 *                 app_iic_who_am_i_async()      只读 WHO_AM_I（在线检测）
 * 中断回调        app_iic_on_i2c_cplt()         转发 HAL 读完成
 *                 app_iic_on_i2c_error()        转发 HAL 错误
 * 解包            app_iic_decode_battery()      23 字节原始数据 -> 物理量
 *                 app_iic_get_error_string()    错误状态位 -> 字符串
 * ========================================================== */

/* ==================== 1. 变量 ==================== */

/* 绑定的 I2C 句柄（由 app_iic_init 设置） */
static I2C_HandleTypeDef *app_iic_hi2c = NULL;

/* DMA 落地缓存：紧凑、顺序与寄存器一致（偏移 0 对应寄存器 0x1F） */
static uint8_t app_iic_raw[APP_IIC_BLOCK_LEN];

/* 传输状态：同一时刻只允许一笔传输 */
typedef enum
{
    APP_IIC_ST_IDLE = 0,      /* 空闲，可接受新请求 */
    APP_IIC_ST_BLOCK_BUSY,    /* 整块数据读取中 */
    APP_IIC_ST_REG_BUSY,      /* 单寄存器读取中 */
} app_iic_state_e;

static volatile app_iic_state_e app_iic_state = APP_IIC_ST_IDLE;

static uint8_t                *app_iic_reg_buf;   /* 单寄存器读的目标缓存（回调里回传） */
static app_iic_battery_data_t *app_iic_user;      /* 整块读取时的用户结构体（解析目标） */
static app_iic_callback_t      app_iic_cb;        /* 本次传输的完成回调，可为 NULL */
static void                   *app_iic_ctx;       /* 回调透传参数 */
static app_iic_status_e        app_iic_result;    /* 最近一次传输结果 */

/* 整块缓存中每个字段的偏移 = 寄存器地址 - 首寄存器地址 */
#define APP_IIC_OFF(field)  ((uint16_t)((field) - APP_IIC_BLOCK_FIRST_REG))
#define APP_IIC_OFF_WHO_AM_I        APP_IIC_OFF(APP_IIC_REG_WHO_AM_I)
#define APP_IIC_OFF_CAPACITY        APP_IIC_OFF(APP_IIC_REG_DESIGNED_CAPACITY)
#define APP_IIC_OFF_LOOP_TIMES      APP_IIC_OFF(APP_IIC_REG_LOOP_TIMES)
#define APP_IIC_OFF_PROD_DATE       APP_IIC_OFF(APP_IIC_REG_PRODUCTION_DATE)
#define APP_IIC_OFF_BATTERY_LIFE    APP_IIC_OFF(APP_IIC_REG_BATTERY_LIFE)
#define APP_IIC_OFF_VOLTAGE         APP_IIC_OFF(APP_IIC_REG_CURRENT_VOLTAGE)
#define APP_IIC_OFF_CURRENT         APP_IIC_OFF(APP_IIC_REG_CURRENT_CURRENT)
#define APP_IIC_OFF_TEMPERATURE     APP_IIC_OFF(APP_IIC_REG_TEMPERATURE)
#define APP_IIC_OFF_CAPACITY_PCT    APP_IIC_OFF(APP_IIC_REG_CAPACITY_PRECENT)
#define APP_IIC_OFF_INTERNAL_STATE  APP_IIC_OFF(APP_IIC_REG_INTERNAL_STATE)
#define APP_IIC_OFF_ERROR_STATE     APP_IIC_OFF(APP_IIC_REG_ERROR_STATE)

/* ==================== 2. 内部工具 ==================== */

/**
  * @brief          小端 2 字节 -> uint16_t（手册：数据均小端存放）
  * @param[in]      p: 指向 2 字节原始数据的指针
  * @retval         拼好的 16bit 数值
  */
static inline uint16_t app_iic_rd_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

/**
  * @brief          小端 4 字节 -> uint32_t（手册：数据均小端存放）
  * @param[in]      p: 指向 4 字节原始数据的指针
  * @retval         拼好的 32bit 数值
  */
static inline uint32_t app_iic_rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/**
  * @brief          把 HAL 返回值/错误码翻译成本模块状态码
  * @param[in]      st: HAL 返回值
  * @retval         本模块状态码
  */
static app_iic_status_e app_iic_hal_to_status(HAL_StatusTypeDef st)
{
    if (st == HAL_OK)
    {
        return APP_IIC_OK;
    }
    if (st == HAL_BUSY)
    {
        return APP_IIC_BUSY;
    }
    if (st == HAL_TIMEOUT)
    {
        return APP_IIC_ERROR_TIMEOUT;
    }

    /* HAL_ERROR：再看 ErrorCode 细分 */
    if (app_iic_hi2c != NULL)
    {
        uint32_t err = app_iic_hi2c->ErrorCode;

        if ((err & HAL_I2C_ERROR_AF) != 0U)
        {
            return APP_IIC_ERROR_NACK;   /* 从机无应答 / 未接 / 未开机 */
        }
        if ((err & (HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_ARLO)) != 0U)
        {
            return APP_IIC_ERROR_BUS;
        }
        /* 常见于 CubeMX 里忘加 I2Cx_RX 的 DMA 通道 */
        if ((err & HAL_I2C_ERROR_DMA_PARAM) != 0U)
        {
            return APP_IIC_ERROR_HAL;
        }
    }
    return APP_IIC_ERROR_HAL;
}

/**
  * @brief          错误后的外设复位，避免总线卡死
  * @retval         none
  * @note           HAL_I2C_DeInit/Init 是同步调用，比较重。若你的工程在 1 kHz 控制中断里
  *                 调用本模块，建议把它换成「只清错误标志 + 打错误计数」的轻量处理。
  */
static void app_iic_recover(void)
{
    if (app_iic_hi2c != NULL)
    {
        (void)HAL_I2C_DeInit(app_iic_hi2c);
        (void)HAL_I2C_Init(app_iic_hi2c);
    }
}

/**
  * @brief          启动一次 DMA 读（全部寄存器或指定寄存器）
  * @param[in]      reg: 寄存器地址
  * @param[out]     buf: 目标缓存
  * @param[in]      len: 读取字节数
  * @retval         APP_IIC_OK 表示已启动；其它为立即失败的原因
  * @note           I2C 句柄未绑定、buf 为空或 len 为 0 时返回 APP_IIC_ERROR_PARAM。
  */
static app_iic_status_e app_iic_read_dma(uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef st;

    if ((app_iic_hi2c == NULL) || (buf == NULL) || (len == 0U))
    {
        return APP_IIC_ERROR_PARAM;
    }

    app_iic_hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
    st = HAL_I2C_Mem_Read_DMA(app_iic_hi2c, APP_IIC_BATTERY_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len);
    return app_iic_hal_to_status(st);
}

/**
  * @brief          整块读取收尾：解析数据 + 校验 WHO_AM_I + 回调
  * @param[in]      status: 本次传输结果
  * @retval         none
  * @note           只在传输成功时解包；WHO_AM_I 不是 0x41 则把结果改成
  *                 APP_IIC_ERROR_ID 再回调。回调在中断上下文里执行。
  */
static void app_iic_finish_meas(app_iic_status_e status)
{
    app_iic_callback_t      cb   = app_iic_cb;
    void                   *ctx  = app_iic_ctx;
    app_iic_battery_data_t *user = app_iic_user;

    app_iic_state = APP_IIC_ST_IDLE;
    app_iic_cb    = NULL;
    app_iic_ctx   = NULL;
    app_iic_user  = NULL;

    if ((status == APP_IIC_OK) && (user != NULL))
    {
        app_iic_decode_battery(app_iic_raw, user);
        if (user->who_am_i != APP_IIC_WHO_AM_I_VALUE)
        {
            status = APP_IIC_ERROR_ID;   /* 数据拿到了但不是电池架 */
        }
    }

    app_iic_result = status;
    if (cb != NULL)
    {
        cb(status, user, ctx);
    }
}

/**
  * @brief          单寄存器读取收尾：数据已在调用者自己的 buf 里，原样回传
  * @param[in]      status: 本次传输结果
  * @retval         none
  * @note           回调第二个形参是本模块给调用者传回的 buf（按字节缓冲区用）。
  */
static void app_iic_finish_reg(app_iic_status_e status)
{
    app_iic_callback_t cb  = app_iic_cb;
    void              *ctx = app_iic_ctx;
    uint8_t           *buf = app_iic_reg_buf;

    app_iic_state = APP_IIC_ST_IDLE;
    app_iic_result = status;
    app_iic_cb    = NULL;
    app_iic_ctx   = NULL;
    app_iic_user  = NULL;

    if (cb != NULL)
    {
        cb(status, (app_iic_battery_data_t *)buf, ctx);
    }
}

/* ==================== 3. 初始化 ==================== */

/**
  * @brief          绑定 I2C 句柄（初始化）
  * @param[in]      hi2c: 已由 CubeMX 初始化好的 I2C 句柄
  * @retval         none
  * @note           绑定后清零传输状态（舍弃未完成的传输），可在运行时重新绑定换总线。
  */
void app_iic_init(I2C_HandleTypeDef *hi2c)
{
    app_iic_hi2c  = hi2c;
    app_iic_state = APP_IIC_ST_IDLE;
    app_iic_cb    = NULL;
    app_iic_ctx   = NULL;
    app_iic_user  = NULL;
    app_iic_result = APP_IIC_OK;
}

/* ==================== 4. 读取接口 ==================== */

/**
  * @brief          读取全部数据（DMA，非阻塞）
  * @param[out]     data: 结果缓存
  * @param[in]      cb:   完成回调，可为 NULL
  * @param[in]      user_ctx: 回调透传参数
  * @retval         APP_IIC_OK 传输已启动；其它为立即失败的原因
  * @note           data 为 NULL 返回 APP_IIC_ERROR_PARAM；上一笔还没结束时返回 APP_IIC_BUSY。
  */
app_iic_status_e app_iic_meas_async(app_iic_battery_data_t *data,
                                    app_iic_callback_t cb, void *user_ctx)
{
    app_iic_status_e st = APP_IIC_ERROR_HAL;
    uint8_t tries;

    if (data == NULL)
    {
        return APP_IIC_ERROR_PARAM;
    }
    if (app_iic_state != APP_IIC_ST_IDLE)
    {
        return APP_IIC_BUSY;
    }

    app_iic_user   = data;
    app_iic_cb     = cb;
    app_iic_ctx    = user_ctx;
    app_iic_result = APP_IIC_BUSY;
    app_iic_state  = APP_IIC_ST_BLOCK_BUSY;

    memset(data, 0, sizeof(*data));

    /* 启动失败（例如总线被占、DMA 没配好）时复位外设后重试 */
    for (tries = APP_IIC_RETRY; tries > 0U; tries--)
    {
        st = app_iic_read_dma(APP_IIC_BLOCK_FIRST_REG, app_iic_raw, APP_IIC_BLOCK_LEN);
        if (st == APP_IIC_OK)
        {
            return APP_IIC_OK;   /* 已启动，等 DMA 完成中断 */
        }
        app_iic_recover();
    }

    app_iic_state = APP_IIC_ST_IDLE;
    app_iic_cb    = NULL;
    app_iic_ctx   = NULL;
    app_iic_user  = NULL;
    return st;
}

/**
  * @brief          读取任意寄存器（DMA，非阻塞）
  * @param[in]      reg: 寄存器地址 (8bit)
  * @param[out]     buf: 目标缓存
  * @param[in]      len: 读取字节数
  * @param[in]      cb:  完成回调，可为 NULL
  * @param[in]      user_ctx: 回调透传参数
  * @retval         APP_IIC_OK 传输已启动；其它为立即失败的原因
  */
app_iic_status_e app_iic_reg_async(uint8_t reg, uint8_t *buf, uint16_t len,
                                   app_iic_callback_t cb, void *user_ctx)
{
    app_iic_status_e st;

    if ((buf == NULL) || (len == 0U))
    {
        return APP_IIC_ERROR_PARAM;
    }
    if (app_iic_state != APP_IIC_ST_IDLE)
    {
        return APP_IIC_BUSY;
    }

    app_iic_reg_buf = buf;
    app_iic_user    = NULL;
    app_iic_cb      = cb;
    app_iic_ctx     = user_ctx;
    app_iic_result  = APP_IIC_BUSY;
    app_iic_state   = APP_IIC_ST_REG_BUSY;

    st = app_iic_read_dma(reg, buf, len);
    if (st != APP_IIC_OK)
    {
        app_iic_state = APP_IIC_ST_IDLE;
        app_iic_cb    = NULL;
        app_iic_ctx   = NULL;
    }
    return st;
}

/**
  * @brief          一键读取 WHO_AM_I（DMA，非阻塞；在线检测用）
  * @param[out]     id:  读出的器件 ID
  * @param[in]      cb:  完成回调，可为 NULL
  * @param[in]      user_ctx: 回调透传参数
  * @retval         APP_IIC_OK 传输已启动；其它为立即失败的原因
  * @note           等价于 app_iic_reg_async(APP_IIC_REG_WHO_AM_I, id, 1, cb, user_ctx)
  */
app_iic_status_e app_iic_who_am_i_async(uint8_t *id,
                                        app_iic_callback_t cb, void *user_ctx)
{
    return app_iic_reg_async(APP_IIC_REG_WHO_AM_I, id, 1U, cb, user_ctx);
}

/* ==================== 5. 中断回调 ====================
 * 使用 HAL 标准完成回调 HAL_I2C_MemRxCpltCallback（读完成自动触发）。
 * 说明：HAL_I2C_Mem_Read_DMA() 只用 hdmarx 搬数据，寄存器地址那一个字节由 HAL
 *       自己在中断里发出，全过程不会触发 MemTxCplt，因此只需转发下面两个。
 * ==================================================== */

/**
  * @brief          转发 HAL_I2C_MemRxCpltCallback（读完成）
  * @param[in]      hi2c: HAL 传入的 I2C 句柄
  * @retval         none
  * @note           句柄不是本模块绑定的、或当前没有传输在跑时直接忽略（安全空转）。
  */
void app_iic_on_i2c_cplt(I2C_HandleTypeDef *hi2c)
{
    if ((app_iic_hi2c == NULL) || (hi2c != app_iic_hi2c))
    {
        return;   /* 不是本模块的 I2C */
    }

    if (app_iic_state == APP_IIC_ST_BLOCK_BUSY)
    {
        app_iic_finish_meas(APP_IIC_OK);
    }
    else if (app_iic_state == APP_IIC_ST_REG_BUSY)
    {
        app_iic_finish_reg(APP_IIC_OK);
    }
    else
    {
        /* IDLE：重复回调，忽略 */
    }
}

/**
  * @brief          转发 HAL_I2C_ErrorCallback（I2C 错误）
  * @param[in]      hi2c: HAL 传入的 I2C 句柄
  * @retval         none
  * @note           会先复位 I2C 外设（app_iic_recover），再把失败结果回调给上层，
  *                 因此出错后驱动立即可用于下一次读取。
  */
void app_iic_on_i2c_error(I2C_HandleTypeDef *hi2c)
{
    app_iic_status_e st;

    if ((app_iic_hi2c == NULL) || (hi2c != app_iic_hi2c) || (app_iic_state == APP_IIC_ST_IDLE))
    {
        return;
    }

    st = app_iic_hal_to_status(HAL_ERROR);
    app_iic_recover();

    if (app_iic_state == APP_IIC_ST_BLOCK_BUSY)
    {
        app_iic_finish_meas(st);
    }
    else
    {
        app_iic_finish_reg(st);
    }
}

/* ==================== 6. 解包函数 ==================== */

/**
  * @brief          解包电池架数据（23 字节原始数据 -> 物理量）
  * @param[in]      raw:  长度 APP_IIC_BLOCK_LEN、首字节为寄存器 0x1F 的原始缓存
  * @param[out]     data: 目标结构体
  * @retval         none
  * @note           本函数只读 raw、只写 data，不做任何 I2C 操作，可独立调用（如单元测试）。
  */
void app_iic_decode_battery(const uint8_t *raw, app_iic_battery_data_t *data)
{
    uint16_t date_raw;

    if ((raw == NULL) || (data == NULL))
    {
        return;
    }

    date_raw = app_iic_rd_u16(&raw[APP_IIC_OFF_PROD_DATE]);

    data->who_am_i          = raw[APP_IIC_OFF_WHO_AM_I];
    data->designed_capacity = app_iic_rd_u32(&raw[APP_IIC_OFF_CAPACITY]);
    data->loop_times        = app_iic_rd_u16(&raw[APP_IIC_OFF_LOOP_TIMES]);
    data->production_date   = date_raw;
    data->battery_life      = raw[APP_IIC_OFF_BATTERY_LIFE];

    data->voltage_mv        = (int32_t)app_iic_rd_u32(&raw[APP_IIC_OFF_VOLTAGE]);
    data->current_ma        = (int32_t)app_iic_rd_u32(&raw[APP_IIC_OFF_CURRENT]);
    data->temperature_raw   = (int16_t)app_iic_rd_u16(&raw[APP_IIC_OFF_TEMPERATURE]);
    data->temperature_c     = (float)data->temperature_raw * 0.1f;
    data->capacity_percent  = raw[APP_IIC_OFF_CAPACITY_PCT];
    data->internal_state    = raw[APP_IIC_OFF_INTERNAL_STATE];
    data->error_state       = raw[APP_IIC_OFF_ERROR_STATE];

    data->year  = APP_IIC_DATE_GET_YEAR(date_raw);
    data->month = APP_IIC_DATE_GET_MONTH(date_raw);
    data->day   = APP_IIC_DATE_GET_DAY(date_raw);
}

/**
  * @brief          return the error state string（错误状态位解码）
  * @param[in]      error_state: APP_IIC_REG_ERROR_STATE 读回的值
  * @retval         人类可读字符串（静态常量，不要 free）
  * @note           多个错误位同时置位时，只返回优先级最高的那个；无错误返回 "no error"。
  */
const char *app_iic_get_error_string(uint8_t error_state)
{
    if ((error_state & APP_IIC_ERROR_SELF_CHECK_Msk) != 0U)     { return "self-check error"; }
    if ((error_state & APP_IIC_ERROR_CELL_Msk) != 0U)           { return "battery cell error"; }
    if ((error_state & APP_IIC_ERROR_VOLTAGE_LOW_Msk) != 0U)    { return "battery voltage low"; }
    if ((error_state & APP_IIC_ERROR_OVERHEAT_Msk) != 0U)       { return "battery overheated"; }
    if ((error_state & APP_IIC_ERROR_CURRENT_EXCEED_Msk) != 0U) { return "battery current exceeded"; }
    if ((error_state & APP_IIC_ERROR_OVER_DISCHARGE_Msk) != 0U) { return "battery over discharged"; }
    if ((error_state & APP_IIC_ERROR_SHORT_Msk) != 0U)          { return "short-circuit during discharge"; }
    return "no error";
}
