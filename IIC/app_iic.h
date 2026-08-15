#ifndef APP_IIC_H
#define APP_IIC_H

#include "iic.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

/* ============================ 使用说明（FreeRTOS） ============================
 *
 * [1] CubeMX 配置（必需）：
 *     - 开启 I2C（如 I2C1），速率选 Fast Mode 400kHz 以内；
 *     - DMA 设置里为 I2C1_RX / I2C1_TX 添加 DMA 通道；
 *     - F1 系列建议同时使能 I2C 事件中断（NVIC 勾选 I2C1 event interrupt），
 *       否则 DMA 传输完成事件可能得不到及时处理、状态卡在 BUSY；
 *     - 注意：FreeRTOS 占用 SysTick 时，若还需 HAL_GetTick 计时（如
 *       iic_is_device_ready 的超时），请在 CubeMX 中给 HAL 单独配置 TIM 时基。
 *
 * [2] 定义设备并初始化（dev_addr 为 7 位地址，不含 R/W 位）：
 *     iic_dev_t imu = {0};
 *     iic_dev_init(&imu, &hi2c1, 0x18, 100);   // 例：BMI088 加速度计 7 位地址 0x18
 *
 * [3] 本库是纯异步的：发起传输后立即返回，不提供任何忙等/阻塞等待 API，
 *     DMA 完成后有两种取数方式（任选其一）：
 *
 *     a) 回调驱动（推荐，最简单）：完成回调在 DMA 中断上下文执行，
 *        只做置标志/发队列这类轻量操作，数据处理放到任务里。
 *     static void imu_done(iic_dev_t *dev) { imu_flag = 1; }  // ISR 上下文！
 *
 *     b) 任务通知驱动：传输前设置当前任务句柄，完成时 ISR 会
 *        vTaskNotifyGiveFromISR 通知任务，任务用 ulTaskNotifyTake 睡眠等待
 *        （非忙等，期间让出 CPU，其他任务照常运行）。
 *
 *     // ===== 任务内典型用法（任务通知 + 定时连续读取） =====
 *     void IMU_Task(void *arg)
 *     {
 *         static uint8_t accel_raw[6];
 *         iic_dev_t imu;
 *         iic_dev_init(&imu, &hi2c1, 0x18, 100);
 *         iic_dma_set_notify_task(&imu, xTaskGetCurrentTaskHandle());
 *
 *         for (;;)
 *         {
 *             if (iic_reg_read_dma(&imu, 0x02, accel_raw, 6, NULL, NULL) == IIC_OK)
 *             {
 *                 ulTaskNotifyTake(pdTRUE, 50);   // 最多等 50 tick，不忙等
 *                 /* 此时 accel_raw 已更新，做数据处理  }
 *             }
 *             vTaskDelay(pdMS_TO_TICKS(5));
 *         }
 *     }
 *
 * [4] 调试：检测设备在线 / 扫描总线所有 7 位地址
 *     iic_is_device_ready(&imu);                   // IIC_OK = 在线
 *     uint32_t found = iic_bus_scan(&hi2c1);       // bit[addr]=1 表示该地址有应答
 *
 * 注意：若传感器不支持重复起始（RESTART），F1 系列 DMA 读可能出错。
 * ================================================================== */

/* ==================== 1. 状态 / 类型定义 ==================== */

/* I2C 操作返回值 */
typedef enum
{
    IIC_OK      = 0,   /* 成功 */
    IIC_BUSY    = 1,   /* 总线忙 / DMA 传输中 */
    IIC_TIMEOUT = 2,   /* 超时 */
    IIC_ERROR   = 3,   /* 通信错误（NACK / 硬件错误） */
} iic_status_e;

/* DMA 传输方向（内部使用） */
typedef enum
{
    IIC_DMA_TX = 0,
    IIC_DMA_RX = 1,
} iic_dma_dir_e;

/* IIC 设备描述结构 */
typedef struct
{
    I2C_HandleTypeDef *hi2c;  /* HAL I2C 句柄（CubeMX 生成的 hi2c1 / hi2c2 ...） */
    uint16_t dev_addr;        /* 7 位设备地址（不含 R/W 位，例：BMI088 加速度计为 0x18） */
    uint32_t timeout;         /* 超时时间，单位 ms（iic_is_device_ready 使用） */
    TaskHandle_t notify_task; /* FreeRTOS 任务句柄：DMA 完成/出错时用 ISR 通知该任务，
                               * 传 NULL 则只走回调，不通知任务 */
} iic_dev_t;

/* DMA 完成 / 错误回调（在 DMA 中断上下文执行，应尽量简短：
 * 只置标志、发队列/信号量，数据处理放到任务里） */
typedef void (*iic_dma_cb_t)(iic_dev_t *dev);

/* DMA 通道数：同时支持的最大 I2C 总线数量 */
#define IIC_DMA_CH_NUM 4

/* DMA 通道注册表条目（内部使用）：每个 I2C 外设（总线）占用一个通道 */
typedef struct
{
    I2C_HandleTypeDef *hi2c;  /* 总线句柄（&hi2c1 / &hi2c2 ...） */
    iic_dev_t *dev;           /* 当前发起传输的设备（回调/任务通知参数） */
    iic_dma_cb_t cb_tx;       /* 发送完成回调 */
    iic_dma_cb_t cb_rx;       /* 接收完成回调 */
    iic_dma_cb_t cb_err;      /* 错误回调 */
    uint8_t used;             /* 通道占用标志 */
} iic_dma_ch_t;

/* ==================== 2. 初始化 / 通用 ==================== */

/**
  * @brief          初始化一个 I2C 设备
  * @param[in]      dev:      设备结构体指针
  * @param[in]      hi2c:     HAL I2C 句柄（&hi2c1 等）
  * @param[in]      dev_addr: 7 位设备地址
  * @param[in]      timeout:  超时 ms（传 0 则用默认 100ms）
  * @retval         none
  */
extern void iic_dev_init(iic_dev_t *dev, I2C_HandleTypeDef *hi2c,
                         uint16_t dev_addr, uint32_t timeout);

/**
  * @brief          检测设备是否在线（发送地址并等待 ACK）
  * @retval         IIC_OK: 在线；IIC_ERROR: 无应答
  */
extern iic_status_e iic_is_device_ready(iic_dev_t *dev);

/**
  * @brief          扫描总线上所有 7 位地址（调试用）
  * @param[in]      hi2c: 要扫描的 I2C 句柄
  * @retval         uint32_t 位图，bit[addr]=1 表示地址 addr 有设备应答
  */
extern uint32_t iic_bus_scan(I2C_HandleTypeDef *hi2c);

/* ==================== 3. DMA 模式（异步非阻塞） ==================== */

/**
  * @brief          DMA 写多个寄存器（异步，reg 由 CPU 先发、数据由 DMA 发）
  * @param[in]      dev:      设备指针
  * @param[in]      reg:      起始寄存器地址
  * @param[in]      buf:      待发送数据（DMA 期间必须保持有效）
  * @param[in]      len:      数据长度
  * @param[in]      cplt_cb:  完成回调（可传 NULL，仅用任务通知取数）
  * @param[in]      err_cb:   错误回调（可传 NULL）
  * @retval         IIC_OK: 已启动；其余见 iic_status_e
  */
extern iic_status_e iic_reg_write_dma(iic_dev_t *dev, uint8_t reg, uint8_t *buf, uint16_t len,
                                      iic_dma_cb_t cplt_cb, iic_dma_cb_t err_cb);

/**
  * @brief          DMA 读多个寄存器（异步，reg 由 CPU 先发、数据由 DMA 收）
  *                 参数同 iic_reg_write_dma
  */
extern iic_status_e iic_reg_read_dma(iic_dev_t *dev, uint8_t reg, uint8_t *buf, uint16_t len,
                                     iic_dma_cb_t cplt_cb, iic_dma_cb_t err_cb);

/**
  * @brief          DMA 写存储器（异步，适用 EEPROM，mem_add_size 传
  *                 I2C_MEMADD_SIZE_8BIT / I2C_MEMADD_SIZE_16BIT）
  */
extern iic_status_e iic_mem_write_dma(iic_dev_t *dev, uint16_t mem_addr, uint16_t mem_add_size,
                                      uint8_t *buf, uint16_t len,
                                      iic_dma_cb_t cplt_cb, iic_dma_cb_t err_cb);

/**
  * @brief          DMA 读存储器（异步，参数同 iic_mem_write_dma）
  */
extern iic_status_e iic_mem_read_dma(iic_dev_t *dev, uint16_t mem_addr, uint16_t mem_add_size,
                                     uint8_t *buf, uint16_t len,
                                     iic_dma_cb_t cplt_cb, iic_dma_cb_t err_cb);

/* ==================== 4. FreeRTOS 集成 ==================== */

/**
  * @brief          设置 DMA 完成/出错时要通知的 FreeRTOS 任务
  *                 传输结束时 ISR 内 vTaskNotifyGiveFromISR 通知该任务，
  *                 任务侧可用 ulTaskNotifyTake(pdTRUE, timeout) 睡眠等待（非忙等）
  * @param[in]      dev:  设备指针
  * @param[in]      task: 任务句柄（传 NULL 取消通知）
  * @retval         none
  */
extern void iic_dma_set_notify_task(iic_dev_t *dev, TaskHandle_t task);

/* ==================== 5. DMA 状态管理 ==================== */

/**
  * @brief          查询 DMA 传输是否完成（非阻塞）
  * @retval         IIC_OK: 空闲；IIC_BUSY: 传输中
  */
extern iic_status_e iic_dma_is_busy(iic_dev_t *dev);

/**
  * @brief          中止当前 DMA 传输（非阻塞）
  * @retval         IIC_OK: 已中止/本就空闲；IIC_ERROR: 中止失败
  */
extern iic_status_e iic_dma_abort(iic_dev_t *dev);

#endif