# RoboMaster 电池架 2 —— I2C 数据读取（STM32H723 + DMA，只读）

代码依据《RoboMaster 电池架 2 使用说明》（PDF 第 6~8 页中文版，即第 14~16 页英文版）编写。
命名对齐同目录下的 `CAN/app_can.c` / `app_can.h`（`app_iic_*` 前缀 + Doxygen 注释风格）。
本模块**只处理电池架一个设备**，只读数据，因此没有设备配置表，也没有写接口。

## 一、文件说明

| 文件 | 作用 |
| --- | --- |
| `app_iic.h` | 器件参数、寄存器定义、数据结构、接口声明 |
| `app_iic.c` | 驱动实现：只用 `HAL_I2C_Mem_Read_DMA()`，一次连续读取 23 字节 |
| `app_iic_example.c` | CubeMX 配置步骤 + 调用示例 |

把 `app_iic.c` 加入 CubeMX 工程的编译列表即可（`app_iic_example.c` 只是参考，不要一起编）。

## 二、对外接口

绑定一次，之后反复发起读取即可：

```c
void             app_iic_init(I2C_HandleTypeDef *hi2c);       /* 绑定 I2C */

/* 读取（全部 DMA，非阻塞） */
app_iic_status_e app_iic_meas_async(app_iic_battery_data_t *data,
                                    app_iic_callback_t cb, void *user_ctx);   /* 读全部数据 */
app_iic_status_e app_iic_reg_async(uint8_t reg, uint8_t *buf, uint16_t len,
                                   app_iic_callback_t cb, void *user_ctx);    /* 读任意寄存器 */
app_iic_status_e app_iic_who_am_i_async(uint8_t *id,
                                        app_iic_callback_t cb, void *user_ctx);/* 在线检测 */

/* HAL 中断转发 */
void             app_iic_on_i2c_cplt(I2C_HandleTypeDef *hi2c);
void             app_iic_on_i2c_error(I2C_HandleTypeDef *hi2c);

/* 解包 */
void             app_iic_decode_battery(const uint8_t *raw, app_iic_battery_data_t *data);
const char      *app_iic_get_error_string(uint8_t error_state);
```

没有任何阻塞式接口，也没有写接口。

## 三、手册里的通信参数

| 项目 | 值 |
| --- | --- |
| 从机地址 | **0x41**（7 bit） |
| 通信速率 | **100 kHz** |
| 上拉电阻 | 电池架**内部没有上拉**，主控必须在 SDA/SCL 上各加 **4.7 kΩ** 上拉到 3.3 V |
| 寄存器地址宽度 | 8 bit |
| 字节序 | 小端（Little-Endian） |
| 接口丝印（从上到下） | A = GND，B = SDA，C = SCL |

> 手册原文警告：不加外部上拉，总线可能死锁或通信失败。

## 四、寄存器表（手册原表，全部只读）

| 宏 | 地址 | 数据类型 | 说明 |
| --- | --- | --- | --- |
| `APP_IIC_REG_WHO_AM_I` | 0x1F | uint8_t | 器件 ID，固定 **0x41** |
| `APP_IIC_REG_DESIGNED_CAPACITY` | 0x20 | uint32_t | 设计容量 (mAh) |
| `APP_IIC_REG_LOOP_TIMES` | 0x24 | uint16_t | 循环次数 |
| `APP_IIC_REG_PRODUCTION_DATE` | 0x26 | uint16_t | 生产日期（位域） |
| `APP_IIC_REG_BATTERY_LIFE` | 0x28 | uint8_t | 电池寿命 (0~100 %) |
| `APP_IIC_REG_CURRENT_VOLTAGE` | 0x29 | int32_t | 瞬时电压 (mV) |
| `APP_IIC_REG_CURRENT_CURRENT` | 0x2D | int32_t | 瞬时电流 (mA) |
| `APP_IIC_REG_TEMPERATURE` | 0x31 | int16_t | 瞬时温度，精度 **0.1 ℃** |
| `APP_IIC_REG_CAPACITY_PRECENT` | 0x33 | uint8_t | 剩余电量 (0~100 %) |
| `APP_IIC_REG_INTERNAL_STATE` | 0x34 | uint8_t | 内部状态（位域） |
| `APP_IIC_REG_ERROR_STATE` | 0x35 | uint8_t | 错误状态（位域） |

地址连续（0x1F~0x35，共 23 字节），所以驱动用一次「发寄存器地址 + 连读 23 字节」取回全部数据，
DMA 只搬一趟（`APP_IIC_BLOCK_LEN` = 23）。

## 五、位域详解

### `PRODUCTION_DATE` (0x26)

| 位 | 含义 |
| --- | --- |
| 15:9 | 年份偏移（**相对 1980**，实际年份 = 该值 + 1980） |
| 8:5 | 月份 |
| 4:0 | 日 |

（也可直接用宏：`APP_IIC_DATE_GET_YEAR/MONTH/DAY`）

### `INTERNAL_STATE` (0x34)

| 位 | 含义 | 取值 |
| --- | --- | --- |
| 7:2 | 保留 | N/A |
| 1 | 与电池的通信状态 | 0 = 通信异常（电池未装好 / 未开机，**默认值**）<br>1 = 通信正常 |
| 0 | 电池电流输出状态 | 0 = 小电流状态（<0.2 A），**开机 20 分钟后会自动关闭输出**（默认值）<br>1 = 大电流状态（>0.2 A），不会自动关闭输出 |

### `ERROR_STATE` (0x35) —— 1 表示发生该错误

| 位 | 含义 |
| --- | --- |
| 7 | 保留 |
| 6 | 自检错误 |
| 5 | 电芯错误 |
| 4 | 电池电压过低 |
| 3 | 电池过热 |
| 2 | 电池电流超限 |
| 1 | 电池过放 |
| 0 | 放电短路 |

## 六、CubeMX 配置要点

1. **I2C**：Standard Mode，Clock Speed = 100000 Hz。
2. **I2C → DMA Settings**：
   - 添加 **`I2Cx_RX`**（Peripheral To Memory）。
     对照 HAL 源码 `stm32h7xx_hal_i2c.c`：`HAL_I2C_Mem_Read_DMA()` 只用 `hi2c->hdmarx` 搬数据，
     `hdmarx == NULL` 时直接返回 `HAL_ERROR` 并置 `HAL_I2C_ERROR_DMA_PARAM`；
     寄存器地址那一个字节由 HAL 自己在中断里发出，**不占 DMA**。
   - 因为只读不写，**`I2Cx_TX` 不需要加**。
3. **NVIC**：使能 DMA 通道中断和 I2C event/error 中断。
4. **中断转发**（只需两个）：
   ```c
   void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) { app_iic_on_i2c_cplt(hi2c); }
   void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)     { app_iic_on_i2c_error(hi2c); }
   ```

## 七、快速上手

```c
#include "app_iic.h"

static app_iic_battery_data_t bat;
static volatile uint8_t bat_ready = 0;

static void bat_cb(app_iic_status_e status, app_iic_battery_data_t *data, void *ctx)
{
    (void)data; (void)ctx;
    bat_ready = (status == APP_IIC_OK) ? 1 : 0;   /* 数据已解析进 bat */
}

void app_init(void)
{
    app_iic_init(&hi2c1);
}

/* 放在 100 ms 周期任务里；返回 APP_IIC_BUSY 说明上一帧还没读完，跳过即可 */
void app_poll_100ms(void)
{
    if (bat_ready) {
        bat_ready = 0;
        /* 这里已在任务上下文，可以 printf / 交给其它模块 */
        float v = (float)bat.voltage_mv / 1000.0f;
        float i = (float)bat.current_ma / 1000.0f;
        (void)v; (void)i;
    }
    (void)app_iic_meas_async(&bat, bat_cb, NULL);
}
```

只判断电池在不在线，读 1 字节即可：

```c
uint8_t id = 0;
if (app_iic_who_am_i_async(&id, my_cb, NULL) == APP_IIC_OK) {
    /* 回调触发后 id == 0x41 表示在线，status == APP_IIC_ERROR_NACK 表示没接或没开机 */
}
```

换总线（换板子 / 改接线不用改代码）：

```c
app_iic_init(&hi2c2);      /* 从 I2C1 换到 I2C2 */
```

## 八、几个容易踩的坑

1. **结构体不能直接给 DMA 用**。`app_iic_battery_data_t` 里 `uint8_t` 后面紧跟 `uint32_t`，
   编译器会插 3 字节填充，逐字节偏移就和寄存器顺序错位了。模块内部用紧凑的
   `app_iic_raw[23]` 接 DMA，再用 `app_iic_decode_battery()` 解析，不要改这个机制。
2. **必须加上拉电阻**，否则总线可能死锁。
3. `APP_IIC_BUSY` 表示上一次读取还没结束，重复调用会被直接拒绝，这是有意为之。
4. 电池没装好或没开机时返回 `APP_IIC_ERROR_NACK`（`WHO_AM_I` 读不到）。
5. 读取成功后驱动会校验 `WHO_AM_I`，不等于 0x41 时回调收到 `APP_IIC_ERROR_ID`。
6. 忘记调用 `app_iic_init()` 就读取，会返回 `APP_IIC_ERROR_PARAM`。
7. 驱动按 `hi2c` 指针区分是不是自己的传输；换绑后旧总线的回调会被忽略。

## 九、验证情况

- 用**真实的 STM32H7 HAL**（`STM32Cube_FW_H7_V1.12.1`，目标 `STM32H723xx`）
  以 `arm-none-eabi-gcc -Wall -Wextra -Wpedantic` 编译，**零警告零错误**。
- 宿主机功能测试覆盖：全部 10 个字段解析（含负电流、0.1 ℃ 温度、日期位域）、
  错误位解码、DMA 完成回调时机、重入保护、NACK 与 ID 校验失败路径、
  单寄存器读 / `who_am_i`、重新绑定换总线（旧总线回调被忽略）、未绑定句柄、
  独立解包 —— 全部通过。

## 十、本目录里的辅助文件（可删）

`.tools/` 只是当初为了解密并抽取那份加密 PDF、以及做编译/功能验证而放的脚手架，
**与单片机工程无关，可以整目录删掉**。
