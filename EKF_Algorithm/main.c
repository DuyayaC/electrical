#include "imu_ekf_app.h"

#include <stdbool.h>
#include <stdint.h>

#define REG32(addr) (*(volatile uint32_t *)(addr))
#define REG8(addr)  (*(volatile uint8_t *)(addr))

#define RCC_AHB1ENR REG32(0x40023830UL)
#define RCC_APB2ENR REG32(0x40023844UL)

#define GPIOA_BASE  0x40020000UL
#define GPIOB_BASE  0x40020400UL
#define GPIOG_BASE  0x40021800UL
#define GPIOH_BASE  0x40021C00UL

#define GPIO_MODER(base)   REG32((base) + 0x00UL)
#define GPIO_OSPEEDR(base) REG32((base) + 0x08UL)
#define GPIO_AFRL(base)    REG32((base) + 0x20UL)
#define GPIO_AFRH(base)    REG32((base) + 0x24UL)
#define GPIO_BSRR(base)    REG32((base) + 0x18UL)

#define SPI1_CR1    REG32(0x40013000UL)
#define SPI1_SR     REG32(0x40013008UL)
#define SPI1_DR8    REG8(0x4001300CUL)

#define USART1_SR   REG32(0x40011000UL)
#define USART1_DR   REG32(0x40011004UL)
#define USART1_BRR  REG32(0x40011008UL)
#define USART1_CR1  REG32(0x4001100CUL)

#define USART6_SR   REG32(0x40011400UL)
#define USART6_DR   REG32(0x40011404UL)
#define USART6_BRR  REG32(0x40011408UL)
#define USART6_CR1  REG32(0x4001140CUL)

#define DEMCR       REG32(0xE000EDFCUL)
#define DWT_CTRL    REG32(0xE0001000UL)
#define DWT_CYCCNT  REG32(0xE0001004UL)

#define HSI_HZ      16000000UL
#define DEG_MILLI_PER_RAD 57295.77951308232f

static imu_ekf_app_t app;

static void clock_init(void)
{
    RCC_AHB1ENR |= (1UL << 0) | (1UL << 1) | (1UL << 6) | (1UL << 7);
    RCC_APB2ENR |= (1UL << 4) | (1UL << 5) | (1UL << 12);
    __asm volatile ("dsb");
}

static void dwt_init(void)
{
    DEMCR |= (1UL << 24);
    DWT_CYCCNT = 0UL;
    DWT_CTRL |= 1UL;
}

static uint32_t micros(void)
{
    return DWT_CYCCNT / (HSI_HZ / 1000000UL);
}

static void delay_ms_busy(uint32_t ms)
{
    const uint32_t start = micros();
    const uint32_t wait_us = ms * 1000UL;
    while ((uint32_t)(micros() - start) < wait_us) {
    }
}

static void gpio_init(void)
{
    GPIO_MODER(GPIOH_BASE) &= ~0x03F00000UL;
    GPIO_MODER(GPIOH_BASE) |=  0x01500000UL;

    GPIO_MODER(GPIOA_BASE) &= ~((3UL << 8) | (3UL << 14) | (3UL << 18));
    GPIO_MODER(GPIOA_BASE) |=  ((1UL << 8) | (2UL << 14) | (2UL << 18));
    GPIO_AFRL(GPIOA_BASE) &= ~(0xFUL << 28);
    GPIO_AFRL(GPIOA_BASE) |=  (5UL << 28);
    GPIO_AFRH(GPIOA_BASE) &= ~(0xFUL << 4);
    GPIO_AFRH(GPIOA_BASE) |=  (7UL << 4);

    GPIO_MODER(GPIOB_BASE) &= ~((3UL << 0) | (3UL << 6) | (3UL << 8));
    GPIO_MODER(GPIOB_BASE) |=  ((1UL << 0) | (2UL << 6) | (2UL << 8));
    GPIO_AFRL(GPIOB_BASE) &= ~((0xFUL << 12) | (0xFUL << 16));
    GPIO_AFRL(GPIOB_BASE) |=  ((5UL << 12) | (5UL << 16));

    GPIO_MODER(GPIOG_BASE) &= ~(3UL << 28);
    GPIO_MODER(GPIOG_BASE) |=  (2UL << 28);
    GPIO_AFRH(GPIOG_BASE) &= ~(0xFUL << 24);
    GPIO_AFRH(GPIOG_BASE) |=  (8UL << 24);

    GPIO_OSPEEDR(GPIOA_BASE) |= (3UL << 8) | (3UL << 14) | (3UL << 18);
    GPIO_OSPEEDR(GPIOB_BASE) |= (3UL << 0) | (3UL << 6) | (3UL << 8);
    GPIO_OSPEEDR(GPIOG_BASE) |= (3UL << 28);

    GPIO_BSRR(GPIOA_BASE) = (1UL << 4);
    GPIO_BSRR(GPIOB_BASE) = (1UL << 0);
}

static void uart_init(void)
{
    USART1_BRR = 0x008BUL;
    USART1_CR1 = 0x2008UL;
    USART6_BRR = 0x008BUL;
    USART6_CR1 = 0x2008UL;
}

static void spi1_init(void)
{
    SPI1_CR1 = 0UL;
    SPI1_CR1 = 0x037CUL;
}

static void uart1_putc(char ch)
{
    while ((USART1_SR & (1UL << 7)) == 0UL) {
    }
    USART1_DR = (uint32_t)(uint8_t)ch;
}

static void uart6_putc(char ch)
{
    while ((USART6_SR & (1UL << 7)) == 0UL) {
    }
    USART6_DR = (uint32_t)(uint8_t)ch;
}

static void uart_putc_both(char ch)
{
    uart1_putc(ch);
    uart6_putc(ch);
}

static void uart_puts_both(const char *s)
{
    while (*s != '\0') {
        uart_putc_both(*s++);
    }
}

static void uart_put_u32(uint32_t value)
{
    char buf[10];
    uint32_t n = 0UL;

    do {
        buf[n++] = (char)('0' + (value % 10UL));
        value /= 10UL;
    } while ((value != 0UL) && (n < sizeof(buf)));

    while (n > 0UL) {
        uart_putc_both(buf[--n]);
    }
}

static void uart_put_s32_milli(int32_t value)
{
    if (value < 0) {
        uart_putc_both('-');
        value = -value;
    }

    uart_put_u32((uint32_t)value / 1000UL);
    uart_putc_both('.');
    const uint32_t frac = (uint32_t)value % 1000UL;
    uart_putc_both((char)('0' + ((frac / 100UL) % 10UL)));
    uart_putc_both((char)('0' + ((frac / 10UL) % 10UL)));
    uart_putc_both((char)('0' + (frac % 10UL)));
}

static int32_t spi1_txrx_byte(void *user, uint8_t tx, uint8_t *rx)
{
    (void)user;
    uint32_t timeout = 200000UL;

    while ((SPI1_SR & (1UL << 1)) == 0UL) {
        if (--timeout == 0UL) {
            return -1;
        }
    }
    SPI1_DR8 = tx;

    timeout = 200000UL;
    while ((SPI1_SR & (1UL << 0)) == 0UL) {
        if (--timeout == 0UL) {
            return -1;
        }
    }

    const uint8_t value = SPI1_DR8;
    timeout = 200000UL;
    while ((SPI1_SR & (1UL << 7)) != 0UL) {
        if (--timeout == 0UL) {
            return -1;
        }
    }

    if (rx != 0) {
        *rx = value;
    }
    return 0;
}

static void accel_cs(void *user, bool selected)
{
    (void)user;
    GPIO_BSRR(GPIOA_BASE) = selected ? (1UL << (4 + 16)) : (1UL << 4);
}

static void gyro_cs(void *user, bool selected)
{
    (void)user;
    GPIO_BSRR(GPIOB_BASE) = selected ? (1UL << (0 + 16)) : (1UL << 0);
}

static void app_delay_ms(void *user, uint32_t ms)
{
    (void)user;
    delay_ms_busy(ms);
}

static void led_pulse(void)
{
    GPIO_BSRR(GPIOH_BASE) = (1UL << 10) | (1UL << 11) | (1UL << 12);
    delay_ms_busy(2UL);
    GPIO_BSRR(GPIOH_BASE) = (1UL << (10 + 16))
                          | (1UL << (11 + 16))
                          | (1UL << (12 + 16));
}

static void print_status(imu_ekf_app_status_t st)
{
    uart_puts_both("status=");
    if (st < 0) {
        uart_putc_both('-');
        uart_put_u32((uint32_t)(-st));
    } else {
        uart_put_u32((uint32_t)st);
    }
    uart_puts_both("\r\n");
}

int main(void)
{
    clock_init();
    dwt_init();
    gpio_init();
    uart_init();
    spi1_init();

    uart_puts_both("BMI088 EKF bare start\r\n");

    bmi088_bus_t bus = {
        .user = 0,
        .spi_txrx_byte = spi1_txrx_byte,
        .accel_cs = accel_cs,
        .gyro_cs = gyro_cs,
        .delay_ms = app_delay_ms,
    };

    imu_ekf_app_config_t cfg;
    imu_ekf_app_default_config(&cfg);
    cfg.gyro_bias_samples = 200U;
    cfg.gyro_bias_sample_interval_ms = 2U;

    imu_ekf_app_status_t st = imu_ekf_app_init(&app, &bus, &cfg, micros());
    if (st != IMU_EKF_APP_OK) {
        uart_puts_both("init failed ");
        print_status(st);
        while (1) {
            led_pulse();
            delay_ms_busy(200UL);
        }
    }

    uart_puts_both("EKF ready\r\n");
    uint32_t last_print_us = micros();

    while (1) {
        const uint32_t now = micros();
        st = imu_ekf_app_update(&app, now);

        if ((uint32_t)(now - last_print_us) >= 50000UL) {
            float32_t roll;
            float32_t pitch;
            float32_t yaw;
            imu_ekf_app_get_rpy(&app, &roll, &pitch, &yaw);

            uart_puts_both("r=");
            uart_put_s32_milli((int32_t)(roll * DEG_MILLI_PER_RAD));
            uart_puts_both(" p=");
            uart_put_s32_milli((int32_t)(pitch * DEG_MILLI_PER_RAD));
            uart_puts_both(" y=");
            uart_put_s32_milli((int32_t)(yaw * DEG_MILLI_PER_RAD));
            uart_puts_both(" st=");
            if (st < 0) {
                uart_putc_both('-');
                uart_put_u32((uint32_t)(-st));
            } else {
                uart_put_u32((uint32_t)st);
            }
            uart_puts_both("\r\n");
            last_print_us = now;
            led_pulse();
        }
    }
}
