#ifndef BMI088_H
#define BMI088_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BMI088_ACCEL_CHIP_ID_VALUE 0x1EU
#define BMI088_GYRO_CHIP_ID_VALUE  0x0FU

typedef enum {
    BMI088_OK = 0,
    BMI088_ERROR_NULL = -1,
    BMI088_ERROR_IO = -2,
    BMI088_ERROR_BAD_ID = -3,
    BMI088_ERROR_BAD_ARG = -4
} bmi088_status_t;

typedef enum {
    BMI088_ACCEL_RANGE_3G = 0,
    BMI088_ACCEL_RANGE_6G = 1,
    BMI088_ACCEL_RANGE_12G = 2,
    BMI088_ACCEL_RANGE_24G = 3
} bmi088_accel_range_t;

typedef enum {
    BMI088_GYRO_RANGE_2000_DPS = 0,
    BMI088_GYRO_RANGE_1000_DPS = 1,
    BMI088_GYRO_RANGE_500_DPS = 2,
    BMI088_GYRO_RANGE_250_DPS = 3,
    BMI088_GYRO_RANGE_125_DPS = 4
} bmi088_gyro_range_t;

typedef int32_t (*bmi088_spi_txrx_byte_t)(void *user,
                                          uint8_t tx,
                                          uint8_t *rx);
typedef void (*bmi088_chip_select_t)(void *user, bool selected);
typedef void (*bmi088_delay_ms_t)(void *user, uint32_t ms);

typedef struct {
    void *user;
    bmi088_spi_txrx_byte_t spi_txrx_byte;
    bmi088_chip_select_t accel_cs;
    bmi088_chip_select_t gyro_cs;
    bmi088_delay_ms_t delay_ms;
} bmi088_bus_t;

typedef struct {
    bmi088_accel_range_t accel_range;
    bmi088_gyro_range_t gyro_range;
    uint8_t accel_conf;
    uint8_t gyro_bandwidth;
} bmi088_config_t;

typedef struct {
    int16_t accel[3];
    int16_t gyro[3];
} bmi088_raw_t;

typedef struct {
    float accel_m_s2[3];
    float gyro_rad_s[3];
} bmi088_data_t;

typedef struct {
    bmi088_bus_t bus;
    bmi088_config_t cfg;
    float accel_m_s2_per_lsb;
    float gyro_rad_s_per_lsb;
} bmi088_t;

void bmi088_default_config(bmi088_config_t *cfg);
bmi088_status_t bmi088_init(bmi088_t *dev,
                            const bmi088_bus_t *bus,
                            const bmi088_config_t *cfg);
bmi088_status_t bmi088_read_ids(bmi088_t *dev,
                                uint8_t *accel_id,
                                uint8_t *gyro_id);
bmi088_status_t bmi088_read_raw(bmi088_t *dev, bmi088_raw_t *raw);
void bmi088_convert(const bmi088_t *dev,
                    const bmi088_raw_t *raw,
                    bmi088_data_t *data);
bmi088_status_t bmi088_read_converted(bmi088_t *dev, bmi088_data_t *data);

#ifdef __cplusplus
}
#endif

#endif
