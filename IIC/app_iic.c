/**
 * @file          app_iic.c
 * @brief         STM32 HAL I2C DMA 驱动（FreeRTOS 友好）：寄存器/存储器异步读写
 *
 * ============================ 实现说明 ============================
 * [1] DMA 模式：封装 HAL_I2C_Mem_Write_DMA / HAL_I2C_Mem_Read_DMA，
 *     内部维护一张按 I2C 总线句柄索引的通道注册表，把 HAL 弱回调
 *     (HAL_I2C_MemTxCpltCallback 等) 路由到用户传入的回调函数。
 * [2] 纯异步：库内没有任何忙等/阻塞等待 API，传输完成通过两种途径
 *     通知用户 —— 用户回调（ISR 上下文）或 FreeRTOS 任务通知
 *     （vTaskNotifyGiveFromISR，需先 iic_dma_set_notify_task 登记任务）。
 * [3] 并发安全：通道注册表的分配/查找/释放用临界区
 *     (taskENTER_CRITICAL / taskEXIT_CRITICAL) 保护，
 *     任务上下文与 DMA 中断上下文可安全并发。
 * [4] 辅助功能：iic_is_device_ready / iic_bus_scan 供初始化与调试。
 * ==================================================================
 */

#include "app_iic.h"

/* ==================== 1. DMA 通道注册表 ==================== */
/* 每个 I2C 外设（总线）占用一个通道，最多支持 IIC_DMA_CH_NUM 条总线。
 * 通道表的分配/查找/释放均需在临界区（taskENTER_CRITICAL）内进行，
 * 保证任务上下文与 DMA 中断上下文并发安全。
 * （iic_dma_ch_t / IIC_DMA_CH_NUM 定义在 app_iic.h） */

static iic_dma_ch_t iic_dma_ch[IIC_DMA_CH_NUM];

/* 按总线句柄查找已注册通道（调用方需处于临界区） */
static iic_dma_ch_t *iic_dma_find_ch(I2C_HandleTypeDef *hi2c)
{
    uint8_t i;
    for (i = 0; i < IIC_DMA_CH_NUM; i++)
    {
        if (iic_dma_ch[i].used && (iic_dma_ch[i].hi2c == hi2c))
            return &iic_dma_ch[i];
    }
    return NULL;
}

/* 分配（或复用）一条总线通道，失败返回 NULL（内部含临界区） */
static iic_dma_ch_t *iic_dma_alloc_ch(I2C_HandleTypeDef *hi2c)
{
    uint8_t i;
    iic_dma_ch_t *ch;

    taskENTER_CRITICAL();
    ch = iic_dma_find_ch(hi2c);
    if (ch == NULL)
    {
        for (i = 0; i < IIC_DMA_CH_NUM; i++)
        {
            if (!iic_dma_ch[i].used)
            {
                iic_dma_ch[i].used   = 1;
                iic_dma_ch[i].hi2c   = hi2c;
                iic_dma_ch[i].dev    = NULL;
                iic_dma_ch[i].cb_tx  = NULL;
                iic_dma_ch[i].cb_rx  = NULL;
                iic_dma_ch[i].cb_err = NULL;
                ch = &iic_dma_ch[i];
                break;
            }
        }
    }
    taskEXIT_CRITICAL();
    return ch;
}

/* 释放通道（调用方需处于临界区） */
static void iic_dma_release_ch(iic_dma_ch_t *ch)
{
    ch->used   = 0;
    ch->dev    = NULL;
    ch->cb_tx  = NULL;
    ch->cb_rx  = NULL;
    ch->cb_err = NULL;
}

/* HAL 状态码 -> 本库状态码 */
static iic_status_e iic_map_hal_status(HAL_StatusTypeDef st)
{
    if (st == HAL_OK)
        return IIC_OK;
    if (st == HAL_BUSY)
        return IIC_BUSY;
    if (st == HAL_TIMEOUT)
        return IIC_TIMEOUT;
    return IIC_ERROR;
}

/* ==================== 2. 初始化 / 通用 ==================== */

void iic_dev_init(iic_dev_t *dev, I2C_HandleTypeDef *hi2c,
                  uint16_t dev_addr, uint32_t timeout)
{
    dev->hi2c        = hi2c;
    dev->dev_addr    = dev_addr;
    dev->timeout     = (timeout == 0) ? 100u : timeout;
    dev->notify_task = NULL;   /* 默认不通知任务，仅走回调 */
}

iic_status_e iic_is_device_ready(iic_dev_t *dev)
{
    if (HAL_I2C_IsDeviceReady(dev->hi2c, (uint16_t)(dev->dev_addr << 1),
                              2, dev->timeout) == HAL_OK)
    {
        return IIC_OK;
    }
    return IIC_ERROR;
}

uint32_t iic_bus_scan(I2C_HandleTypeDef *hi2c)
{
    uint32_t found = 0;
    uint8_t addr;
    for (addr = 1; addr < 128; addr++)
    {
        if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), 2, 5) == HAL_OK)
            found |= (1UL << addr);
    }
    return found;
}

/* ==================== 3. DMA 模式（异步非阻塞） ==================== */

iic_status_e iic_reg_write_dma(iic_dev_t *dev, uint8_t reg, uint8_t *buf, uint16_t len,
                               iic_dma_cb_t cplt_cb, iic_dma_cb_t err_cb)
{
    HAL_StatusTypeDef st;
    iic_dma_ch_t *ch;

    if (HAL_I2C_GetState(dev->hi2c) != HAL_I2C_STATE_READY)
        return IIC_BUSY;

    ch = iic_dma_alloc_ch(dev->hi2c);
    if (ch == NULL)
        return IIC_BUSY;

    ch->dev    = dev;
    ch->cb_tx  = cplt_cb;
    ch->cb_rx  = NULL;
    ch->cb_err = err_cb;

    st = HAL_I2C_Mem_Write_DMA(dev->hi2c, (uint16_t)(dev->dev_addr << 1), reg,
                               I2C_MEMADD_SIZE_8BIT, buf, len);
    if (st != HAL_OK)
    {
        iic_dma_release_ch(ch);
        return iic_map_hal_status(st);
    }
    return IIC_OK;
}

iic_status_e iic_reg_read_dma(iic_dev_t *dev, uint8_t reg, uint8_t *buf, uint16_t len,
                              iic_dma_cb_t cplt_cb, iic_dma_cb_t err_cb)
{
    HAL_StatusTypeDef st;
    iic_dma_ch_t *ch;

    if (HAL_I2C_GetState(dev->hi2c) != HAL_I2C_STATE_READY)
        return IIC_BUSY;

    ch = iic_dma_alloc_ch(dev->hi2c);
    if (ch == NULL)
        return IIC_BUSY;

    ch->dev    = dev;
    ch->cb_tx  = NULL;
    ch->cb_rx  = cplt_cb;
    ch->cb_err = err_cb;

    st = HAL_I2C_Mem_Read_DMA(dev->hi2c, (uint16_t)(dev->dev_addr << 1), reg,
                              I2C_MEMADD_SIZE_8BIT, buf, len);
    if (st != HAL_OK)
    {
        iic_dma_release_ch(ch);
        return iic_map_hal_status(st);
    }
    return IIC_OK;
}

iic_status_e iic_mem_write_dma(iic_dev_t *dev, uint16_t mem_addr, uint16_t mem_add_size,
                               uint8_t *buf, uint16_t len,
                               iic_dma_cb_t cplt_cb, iic_dma_cb_t err_cb)
{
    HAL_StatusTypeDef st;
    iic_dma_ch_t *ch;

    if (HAL_I2C_GetState(dev->hi2c) != HAL_I2C_STATE_READY)
        return IIC_BUSY;

    ch = iic_dma_alloc_ch(dev->hi2c);
    if (ch == NULL)
        return IIC_BUSY;

    ch->dev    = dev;
    ch->cb_tx  = cplt_cb;
    ch->cb_rx  = NULL;
    ch->cb_err = err_cb;

    st = HAL_I2C_Mem_Write_DMA(dev->hi2c, (uint16_t)(dev->dev_addr << 1), mem_addr,
                               mem_add_size, buf, len);
    if (st != HAL_OK)
    {
        iic_dma_release_ch(ch);
        return iic_map_hal_status(st);
    }
    return IIC_OK;
}

iic_status_e iic_mem_read_dma(iic_dev_t *dev, uint16_t mem_addr, uint16_t mem_add_size,
                              uint8_t *buf, uint16_t len,
                              iic_dma_cb_t cplt_cb, iic_dma_cb_t err_cb)
{
    HAL_StatusTypeDef st;
    iic_dma_ch_t *ch;

    if (HAL_I2C_GetState(dev->hi2c) != HAL_I2C_STATE_READY)
        return IIC_BUSY;

    ch = iic_dma_alloc_ch(dev->hi2c);
    if (ch == NULL)
        return IIC_BUSY;

    ch->dev    = dev;
    ch->cb_tx  = NULL;
    ch->cb_rx  = cplt_cb;
    ch->cb_err = err_cb;

    st = HAL_I2C_Mem_Read_DMA(dev->hi2c, (uint16_t)(dev->dev_addr << 1), mem_addr,
                              mem_add_size, buf, len);
    if (st != HAL_OK)
    {
        iic_dma_release_ch(ch);
        return iic_map_hal_status(st);
    }
    return IIC_OK;
}

/* ==================== 4. FreeRTOS 集成 ==================== */

void iic_dma_set_notify_task(iic_dev_t *dev, TaskHandle_t task)
{
    taskENTER_CRITICAL();
    dev->notify_task = task;
    taskEXIT_CRITICAL();
}

/* ==================== 5. DMA 状态管理（均非阻塞） ==================== */

iic_status_e iic_dma_is_busy(iic_dev_t *dev)
{
    return (HAL_I2C_GetState(dev->hi2c) == HAL_I2C_STATE_READY) ? IIC_OK : IIC_BUSY;
}

iic_status_e iic_dma_abort(iic_dev_t *dev)
{
    if (HAL_I2C_GetState(dev->hi2c) == HAL_I2C_STATE_READY)
        return IIC_OK;

    if (HAL_I2C_DMAStop(dev->hi2c) == HAL_OK)
        return IIC_OK;
    return IIC_ERROR;
}

/* ==================== 6. HAL 回调路由 ==================== */
/* 完成回调在 DMA 中断上下文触发，执行要快：
 * 临界区内释放通道 -> 临界区外 vTaskNotifyGiveFromISR 通知任务（可选）
 * -> 调用用户回调。先释放通道可让用户立即发起下一次传输。 */

static void iic_dma_dispatch_cplt(I2C_HandleTypeDef *hi2c, uint8_t dir)
{
    iic_dma_ch_t *ch;
    iic_dma_cb_t cb;
    iic_dev_t *dev;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    taskENTER_CRITICAL();
    ch = iic_dma_find_ch(hi2c);
    if (ch == NULL)
    {
        taskEXIT_CRITICAL();
        return;
    }
    cb  = (dir == IIC_DMA_TX) ? ch->cb_tx : ch->cb_rx;
    dev = ch->dev;
    iic_dma_release_ch(ch);
    taskEXIT_CRITICAL();

    /* FreeRTOS：通知等待该传输完成的任务（若有登记） */
    if (dev->notify_task != NULL)
        vTaskNotifyGiveFromISR(dev->notify_task, &xHigherPriorityTaskWoken);

    if (cb != NULL)
        cb(dev);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void iic_dma_dispatch_err(I2C_HandleTypeDef *hi2c)
{
    iic_dma_ch_t *ch;
    iic_dma_cb_t cb;
    iic_dev_t *dev;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    taskENTER_CRITICAL();
    ch = iic_dma_find_ch(hi2c);
    if (ch == NULL)
    {
        taskEXIT_CRITICAL();
        return;
    }
    cb  = ch->cb_err;
    dev = ch->dev;
    iic_dma_release_ch(ch);
    taskEXIT_CRITICAL();

    /* 出错也通知任务，任务侧可检查状态重新发起传输 */
    if (dev->notify_task != NULL)
        vTaskNotifyGiveFromISR(dev->notify_task, &xHigherPriorityTaskWoken);

    if (cb != NULL)
        cb(dev);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* 寄存器型（Mem 方式）回调 */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    iic_dma_dispatch_cplt(hi2c, IIC_DMA_TX);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    iic_dma_dispatch_cplt(hi2c, IIC_DMA_RX);
}

/* 纯主机 Master 方式回调（本库未用，保留以防其他模块使用） */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    iic_dma_dispatch_cplt(hi2c, IIC_DMA_TX);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    iic_dma_dispatch_cplt(hi2c, IIC_DMA_RX);
}

/* 错误回调（中断 / DMA 模式下出错时触发） */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    iic_dma_dispatch_err(hi2c);
}