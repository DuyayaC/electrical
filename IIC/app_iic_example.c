/**
 * @file          app_iic_example.c
 * @brief         RoboMaster 电池架 2 (I2C + DMA) 在 STM32H723 上的使用示例与移植说明
 *
 * ============================ 一、CubeMX 配置 ============================
 *
 * 1) I2C1（或任意 I2C）
 *    - Mode              : I2C
 *    - Speed Mode        : Standard Mode
 *    - I2C Clock Speed   : 100000 Hz      (手册要求 100 kHz)
 *
 * 2) I2C1 -> DMA Settings  (关键：不配 DMA 时 HAL 会直接返回错误)
 *    - Add -> "I2C1_RX"  (方向 Peripheral To Memory)
 *    - Priority : Medium 或 High
 *    - 因为只读不写，I2C1_TX 这一路不需要加
 *
 *    说明（对照 STM32H7 HAL 源码 stm32h7xx_hal_i2c.c）：
 *      * HAL_I2C_Mem_Read_DMA() 只用 hi2c->hdmarx 搬数据；
 *        寄存器地址那一个字节是 HAL 用中断（I2C_XFER_TX_IT）发出去的，不占 DMA。
 *      * 若 hdmarx == NULL，函数直接返回 HAL_ERROR 并置 HAL_I2C_ERROR_DMA_PARAM。
 *
 * 3) NVIC
 *    - DMA 通道中断        : Enable
 *    - I2C1 event/error IRQ: Enable（HAL 需要靠它发寄存器地址并收尾）
 *
 * 4) 硬件
 *    - SDA / SCL 各接一个 4.7 kΩ 上拉到 3.3 V（电池架内部没有上拉，手册明确要求）
 *    - 电池架接口从上到下：A(GND) / B(SDA) / C(SCL)
 *    - 通信速率 100 kHz，从机地址 0x41（7 bit）
 *
 * ============================ 二、中断转发 ============================
 *
 * 在 main.c 或专门的回调文件里加上（只需要这两个）：
 *
 *   void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) { app_iic_on_i2c_cplt(hi2c); }
 *   void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)     { app_iic_on_i2c_error(hi2c); }
 *
 * ============================ 三、调用方式 ============================
 */

#include "app_iic.h"

/* ------------------------------------------------------------------ */
/* 推荐用法：DMA + 周期轮询发起，全程非阻塞                              */
/* ------------------------------------------------------------------ */

static app_iic_battery_data_t    g_battery;
static volatile uint8_t          g_battery_ready     = 0;
static volatile app_iic_status_e g_battery_status    = APP_IIC_OK;
static volatile uint32_t         g_battery_err_count = 0;

/** 完成回调运行在 DMA 完成中断里，只做轻量记录 */
static void app_iic_battery_callback(app_iic_status_e status, app_iic_battery_data_t *data, void *user_ctx)
{
    (void)data;       /* 数据已解析进 g_battery，这里只记录状态 */
    (void)user_ctx;

    g_battery_status = status;
    if (status == APP_IIC_OK)
    {
        g_battery_ready = 1;
    }
    else
    {
        g_battery_err_count++;
    }
}

void battery_example_init(void)
{
    app_iic_init(&hi2c1);        /* hi2c1 由 CubeMX 生成 */
}

/**
 * @brief 放在 100 ms 周期任务里调用
 * @note  返回 APP_IIC_BUSY 说明上一帧还没读完，直接跳过，下个周期再来。
 */
void battery_example_poll_100ms(void)
{
    if (g_battery_ready)
    {
        g_battery_ready = 0;

        /* 这里已在任务上下文，可以放心 printf / 赋给全局供其它模块使用 */
        float volt = (float)g_battery.voltage_mv / 1000.0f;
        float curr = (float)g_battery.current_ma / 1000.0f;
        (void)volt;
        (void)curr;

        if (g_battery.error_state != 0U)
        {
            (void)app_iic_get_error_string(g_battery.error_state);
        }
    }

    (void)app_iic_meas_async(&g_battery, app_iic_battery_callback, NULL);
}

/** 供其它模块读取最新数据 */
const app_iic_battery_data_t *battery_example_get(void)
{
    return &g_battery;
}

/* ------------------------------------------------------------------ */
/* 只读 WHO_AM_I 做在线检测（同样是 DMA，读 1 字节）                     */
/* ------------------------------------------------------------------ */

static volatile uint8_t g_id_probe_done = 0;
static uint8_t          g_id_buf        = 0;

static void app_iic_id_callback(app_iic_status_e status, app_iic_battery_data_t *data, void *ctx)
{
    (void)data;
    (void)ctx;
    g_id_probe_done = (status == APP_IIC_OK) ? 1U : 2U;
}

void battery_probe_start(void)
{
    g_id_probe_done = 0;
    (void)app_iic_who_am_i_async(&g_id_buf, app_iic_id_callback, NULL);
}

/** @return 1 在线，0 离线或结果未就绪（下个周期再问） */
uint8_t battery_online(void)
{
    if (g_id_probe_done == 0U)
    {
        battery_probe_start();
        return 0;
    }
    if (g_id_probe_done == 2U)
    {
        g_id_probe_done = 0;
        return 0;                     /* 读失败：电池没接 / 没开机 */
    }
    g_id_probe_done = 0;
    return (g_id_buf == APP_IIC_WHO_AM_I_VALUE) ? 1U : 0U;
}

/* ------------------------------------------------------------------ */
/* 换总线：运行时重新绑定即可，不用改代码                                */
/* ------------------------------------------------------------------ */

void battery_switch_to_i2c2(void)
{
    app_iic_init(&hi2c2);            /* 例：从 I2C1 换到 I2C2 */
}

/* ------------------------------------------------------------------ */
/* 接到 RoboMaster 标准库的 device/detect 框架                          */
/*          （把 app_iic.h 里的 APP_IIC_USE_RM_DEVICE 置 1 后使用）      */
/* ------------------------------------------------------------------ */
#if APP_IIC_USE_RM_DEVICE

#include "detect_task.h"

#define APP_IIC_BATTERY_DEV_ID 0x40   /* device id 必须小于 0x80，按自己工程的实际占用挑一个 */

static app_iic_battery_data_t s_rm_data;

static void app_iic_rm_callback(app_iic_status_e status, app_iic_battery_data_t *data, void *ctx)
{
    (void)ctx;
    (void)data;
    if (status == APP_IIC_OK)
    {
        detect_hook(APP_IIC_BATTERY_DEV_ID);   /* 喂狗：告诉检测任务设备在线 */
    }
}

void battery_rm_init(void)
{
    app_iic_init(&hi2c1);
}

void battery_rm_poll(void)
{
    (void)app_iic_meas_async(&s_rm_data, app_iic_rm_callback, NULL);
}

const app_iic_battery_data_t *battery_rm_get(void)
{
    return &s_rm_data;
}

#endif /* APP_IIC_USE_RM_DEVICE */
