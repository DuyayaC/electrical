/* Host-side functional test for the single-device app_iic module (read-only DMA). */
#include "app_iic.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

static uint8_t g_regs[0x100];
static int     g_device_present = 1;
static int     g_call_count;
static uint16_t g_last_dev, g_last_mem, g_last_size;

static void put_u16(uint8_t reg, uint16_t v) { g_regs[reg] = (uint8_t)(v & 0xff); g_regs[reg + 1] = (uint8_t)(v >> 8); }
static void put_u32(uint8_t reg, uint32_t v)
{
    g_regs[reg] = (uint8_t)(v & 0xff); g_regs[reg + 1] = (uint8_t)((v >> 8) & 0xff);
    g_regs[reg + 2] = (uint8_t)((v >> 16) & 0xff); g_regs[reg + 3] = (uint8_t)((v >> 24) & 0xff);
}

HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                       uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
    (void)MemAddSize;
    g_last_dev = DevAddress; g_last_mem = MemAddress; g_last_size = Size;
    g_call_count++;
    if (!g_device_present || DevAddress != APP_IIC_BATTERY_ADDR)
    {
        hi2c->ErrorCode = HAL_I2C_ERROR_AF;
        return HAL_ERROR;
    }
    memcpy(pData, &g_regs[MemAddress], Size);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c) { (void)hi2c; return HAL_OK; }
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c) { (void)hi2c; return HAL_OK; }

static int                    g_cb_calls;
static app_iic_status_e       g_cb_status;
static app_iic_battery_data_t g_cb_data;

static void test_cb(app_iic_status_e status, app_iic_battery_data_t *data, void *ctx)
{
    (void)ctx;
    g_cb_calls++;
    g_cb_status = status;
    if (data != NULL) { g_cb_data = *data; }
}

int main(void)
{
    app_iic_battery_data_t d;
    app_iic_status_e st;

    memset(g_regs, 0, sizeof(g_regs));
    g_regs[APP_IIC_REG_WHO_AM_I] = 0x41;
    put_u32(APP_IIC_REG_DESIGNED_CAPACITY, 6000);
    put_u16(APP_IIC_REG_LOOP_TIMES, 137);
    put_u16(APP_IIC_REG_PRODUCTION_DATE, (uint16_t)((45u << 9) | (3u << 5) | 12u));
    g_regs[APP_IIC_REG_BATTERY_LIFE] = 98;
    put_u32(APP_IIC_REG_CURRENT_VOLTAGE, 24137);
    put_u32(APP_IIC_REG_CURRENT_CURRENT, (uint32_t)(-2350));
    put_u16(APP_IIC_REG_TEMPERATURE, (uint16_t)(int16_t)326);
    g_regs[APP_IIC_REG_CAPACITY_PRECENT] = 76;
    g_regs[APP_IIC_REG_INTERNAL_STATE] = (uint8_t)(APP_IIC_INTERNAL_COMM_Msk | APP_IIC_INTERNAL_OUTPUT_HIGH);
    g_regs[APP_IIC_REG_ERROR_STATE] = 0;

    printf("=== 0) app_iic_init ===\n");
    app_iic_init(&hi2c1);

    printf("=== 1) app_iic_meas_async ===\n");
    g_cb_calls = 0; g_cb_status = APP_IIC_BUSY;
    memset(&g_cb_data, 0, sizeof(g_cb_data));
    st = app_iic_meas_async(&d, test_cb, NULL);
    assert(st == APP_IIC_OK);
    assert(g_last_dev == APP_IIC_BATTERY_ADDR);
    assert(g_last_mem == APP_IIC_BLOCK_FIRST_REG);
    assert(g_last_size == 23);
    assert(g_cb_calls == 0);
    app_iic_on_i2c_cplt(&hi2c1);
    assert(g_cb_calls == 1 && g_cb_status == APP_IIC_OK);

    printf("  who_am_i           = 0x%02X      (expect 0x41)\n", g_cb_data.who_am_i);
    printf("  designed_capacity  = %lu mAh   (expect 6000)\n", (unsigned long)g_cb_data.designed_capacity);
    printf("  loop_times         = %u          (expect 137)\n", g_cb_data.loop_times);
    printf("  production_date    = %u -> %u-%02u-%02u (expect 2025-03-12)\n",
           g_cb_data.production_date, g_cb_data.year, g_cb_data.month, g_cb_data.day);
    printf("  battery_life       = %u %%        (expect 98)\n", g_cb_data.battery_life);
    printf("  voltage_mv         = %ld mV     (expect 24137)\n", (long)g_cb_data.voltage_mv);
    printf("  current_ma         = %ld mA    (expect -2350)\n", (long)g_cb_data.current_ma);
    printf("  temperature        = %d / %.1f C (expect 326 / 32.6)\n", g_cb_data.temperature_raw, g_cb_data.temperature_c);
    printf("  capacity_percent   = %u %%        (expect 76)\n", g_cb_data.capacity_percent);
    printf("  internal_state     = 0x%02X        (expect 0x03)\n", g_cb_data.internal_state);
    printf("  error_string       = %s\n", app_iic_get_error_string(g_cb_data.error_state));

    assert(g_cb_data.who_am_i == 0x41);
    assert(g_cb_data.designed_capacity == 6000);
    assert(g_cb_data.loop_times == 137);
    assert(g_cb_data.year == 2025 && g_cb_data.month == 3 && g_cb_data.day == 12);
    assert(g_cb_data.battery_life == 98);
    assert(g_cb_data.voltage_mv == 24137);
    assert(g_cb_data.current_ma == -2350);
    assert(g_cb_data.temperature_raw == 326);
    assert(g_cb_data.temperature_c > 32.59f && g_cb_data.temperature_c < 32.61f);
    assert(g_cb_data.capacity_percent == 76);
    assert(g_cb_data.internal_state == 0x03);
    assert(strcmp(app_iic_get_error_string(g_cb_data.error_state), "no error") == 0);

    printf("=== 2) error bit decoding ===\n");
    assert(strcmp(app_iic_get_error_string(APP_IIC_ERROR_OVERHEAT_Msk), "battery overheated") == 0);
    assert(strcmp(app_iic_get_error_string(APP_IIC_ERROR_CELL_Msk), "battery cell error") == 0);
    assert(strcmp(app_iic_get_error_string(APP_IIC_ERROR_SHORT_Msk), "short-circuit during discharge") == 0);
    assert(strcmp(app_iic_get_error_string(0xFF), "self-check error") == 0);
    printf("  ok\n");

    printf("=== 3) re-entrant request refused ===\n");
    g_cb_calls = 0;
    st = app_iic_meas_async(&d, test_cb, NULL);
    assert(st == APP_IIC_OK);
    st = app_iic_meas_async(&d, test_cb, NULL);
    printf("  second call        = %d (expect %d = APP_IIC_BUSY)\n", (int)st, (int)APP_IIC_BUSY);
    assert(st == APP_IIC_BUSY);
    app_iic_on_i2c_cplt(&hi2c1);
    assert(g_cb_calls == 1);

    printf("=== 4) error path releases driver ===\n");
    g_cb_calls = 0;
    st = app_iic_meas_async(&d, test_cb, NULL);
    assert(st == APP_IIC_OK);
    hi2c1.ErrorCode = HAL_I2C_ERROR_AF;
    app_iic_on_i2c_error(&hi2c1);
    printf("  status             = %d (expect %d = APP_IIC_ERROR_NACK)\n", (int)g_cb_status, (int)APP_IIC_ERROR_NACK);
    assert(g_cb_calls == 1 && g_cb_status == APP_IIC_ERROR_NACK);
    st = app_iic_meas_async(&d, test_cb, NULL);
    assert(st == APP_IIC_OK);
    app_iic_on_i2c_cplt(&hi2c1);
    printf("  reusable afterwards= ok\n");

    printf("=== 5) offline device ===\n");
    g_device_present = 0;
    st = app_iic_meas_async(&d, test_cb, NULL);
    g_device_present = 1;
    printf("  status             = %d (expect %d = APP_IIC_ERROR_NACK)\n", (int)st, (int)APP_IIC_ERROR_NACK);
    assert(st == APP_IIC_ERROR_NACK);

    printf("=== 6) wrong WHO_AM_I ===\n");
    g_regs[APP_IIC_REG_WHO_AM_I] = 0x00;
    g_cb_calls = 0; g_cb_status = APP_IIC_OK;
    st = app_iic_meas_async(&d, test_cb, NULL);
    assert(st == APP_IIC_OK);
    app_iic_on_i2c_cplt(&hi2c1);
    printf("  cb status          = %d (expect %d = APP_IIC_ERROR_ID)\n", (int)g_cb_status, (int)APP_IIC_ERROR_ID);
    assert(g_cb_status == APP_IIC_ERROR_ID);

    printf("=== 7) app_iic_reg_async / app_iic_who_am_i_async ===\n");
    {
        uint8_t id = 0;
        g_regs[APP_IIC_REG_WHO_AM_I] = 0x41;
        g_cb_calls = 0;
        st = app_iic_reg_async(APP_IIC_REG_WHO_AM_I, &id, 1U, test_cb, NULL);
        assert(st == APP_IIC_OK);
        assert(g_last_mem == APP_IIC_REG_WHO_AM_I && g_last_size == 1);
        app_iic_on_i2c_cplt(&hi2c1);
        printf("  reg_async id=0x%02X cb_calls=%d (expect 0x41, 1)\n", id, g_cb_calls);
        assert(g_cb_calls == 1 && g_cb_status == APP_IIC_OK && id == 0x41);

        g_cb_calls = 0; id = 0;
        st = app_iic_who_am_i_async(&id, test_cb, NULL);
        assert(st == APP_IIC_OK);
        app_iic_on_i2c_cplt(&hi2c1);
        printf("  who_am_i_async id=0x%02X\n", id);
        assert(id == 0x41);
    }

    printf("=== 8) rebind to another I2C ===\n");
    app_iic_init(&hi2c2);
    g_cb_calls = 0;
    st = app_iic_meas_async(&d, test_cb, NULL);
    assert(st == APP_IIC_OK);
    app_iic_on_i2c_cplt(&hi2c2);            /* 完成回调来自新总线 */
    assert(g_cb_calls == 1 && g_cb_status == APP_IIC_OK);
    app_iic_on_i2c_cplt(&hi2c1);            /* 旧总线回调应被忽略 */
    assert(g_cb_calls == 1);
    printf("  rebound to hi2c2, old-bus callback ignored\n");
    app_iic_init(&hi2c1);

    printf("=== 9) unbound handle -> PARAM ===\n");
    app_iic_init(NULL);
    st = app_iic_meas_async(&d, test_cb, NULL);
    printf("  status             = %d (expect %d = APP_IIC_ERROR_PARAM)\n", (int)st, (int)APP_IIC_ERROR_PARAM);
    assert(st == APP_IIC_ERROR_PARAM);
    app_iic_init(&hi2c1);

    printf("=== 10) app_iic_decode_battery standalone ===\n");
    {
        app_iic_battery_data_t e;
        memset(&e, 0, sizeof(e));
        app_iic_decode_battery(g_regs + APP_IIC_BLOCK_FIRST_REG, &e);
        assert(e.who_am_i == 0x41 && e.voltage_mv == 24137);
        printf("  ok\n");
    }

    printf("\nALL TESTS PASSED (HAL DMA read calls: %d)\n", g_call_count);
    return 0;
}
