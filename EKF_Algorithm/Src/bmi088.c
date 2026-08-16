#include "bmi088.h"

#include <string.h>

#define BMI088_ACCEL_CHIP_ID      0x00U
#define BMI088_ACCEL_X_LSB       0x12U
#define BMI088_ACCEL_CONF        0x40U
#define BMI088_ACCEL_RANGE       0x41U
#define BMI088_ACCEL_PWR_CONF    0x7CU
#define BMI088_ACCEL_PWR_CTRL    0x7DU

#define BMI088_GYRO_CHIP_ID      0x00U
#define BMI088_GYRO_X_LSB        0x02U
#define BMI088_GYRO_RANGE        0x0FU
#define BMI088_GYRO_BANDWIDTH    0x10U
#define BMI088_GYRO_LPM1         0x11U

#define BMI088_READ_BIT          0x80U
#define BMI088_ACCEL_DUMMY_READS 1U

#define BMI088_GRAVITY_M_S2      9.80665f
#define BMI088_DEG_TO_RAD        0.01745329251994329577f

static uint8_t accel_range_g(bmi088_accel_range_t range)
{
    switch (range) {
    case BMI088_ACCEL_RANGE_3G:
        return 3U;
    case BMI088_ACCEL_RANGE_6G:
        return 6U;
    case BMI088_ACCEL_RANGE_12G:
        return 12U;
    case BMI088_ACCEL_RANGE_24G:
        return 24U;
    default:
        return 0U;
    }
}

static uint16_t gyro_range_dps(bmi088_gyro_range_t range)
{
    switch (range) {
    case BMI088_GYRO_RANGE_2000_DPS:
        return 2000U;
    case BMI088_GYRO_RANGE_1000_DPS:
        return 1000U;
    case BMI088_GYRO_RANGE_500_DPS:
        return 500U;
    case BMI088_GYRO_RANGE_250_DPS:
        return 250U;
    case BMI088_GYRO_RANGE_125_DPS:
        return 125U;
    default:
        return 0U;
    }
}

static int16_t i16_le(const uint8_t bytes[2])
{
    return (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
}

static bmi088_status_t txrx(const bmi088_t *dev, uint8_t tx, uint8_t *rx)
{
    uint8_t local_rx = 0U;
    if (dev->bus.spi_txrx_byte(dev->bus.user, tx, rx ? rx : &local_rx) != 0) {
        return BMI088_ERROR_IO;
    }
    return BMI088_OK;
}

static void delay_ms(const bmi088_t *dev, uint32_t ms)
{
    if (dev->bus.delay_ms != NULL) {
        dev->bus.delay_ms(dev->bus.user, ms);
    }
}

static bmi088_status_t accel_write_reg(bmi088_t *dev,
                                       uint8_t reg,
                                       uint8_t value)
{
    bmi088_status_t st;

    dev->bus.accel_cs(dev->bus.user, true);
    st = txrx(dev, reg & (uint8_t)~BMI088_READ_BIT, NULL);
    if (st == BMI088_OK) {
        st = txrx(dev, value, NULL);
    }
    dev->bus.accel_cs(dev->bus.user, false);

    return st;
}

static bmi088_status_t gyro_write_reg(bmi088_t *dev,
                                      uint8_t reg,
                                      uint8_t value)
{
    bmi088_status_t st;

    dev->bus.gyro_cs(dev->bus.user, true);
    st = txrx(dev, reg & (uint8_t)~BMI088_READ_BIT, NULL);
    if (st == BMI088_OK) {
        st = txrx(dev, value, NULL);
    }
    dev->bus.gyro_cs(dev->bus.user, false);

    return st;
}

static bmi088_status_t accel_read_reg(bmi088_t *dev,
                                      uint8_t reg,
                                      uint8_t *data,
                                      size_t len)
{
    bmi088_status_t st;
    uint8_t dummy;

    if ((data == NULL) && (len > 0U)) {
        return BMI088_ERROR_NULL;
    }

    dev->bus.accel_cs(dev->bus.user, true);
    st = txrx(dev, reg | BMI088_READ_BIT, NULL);
    for (uint8_t i = 0U; (st == BMI088_OK) && (i < BMI088_ACCEL_DUMMY_READS); ++i) {
        st = txrx(dev, 0x00U, &dummy);
    }
    for (size_t i = 0U; (st == BMI088_OK) && (i < len); ++i) {
        st = txrx(dev, 0x00U, &data[i]);
    }
    dev->bus.accel_cs(dev->bus.user, false);

    return st;
}

static bmi088_status_t gyro_read_reg(bmi088_t *dev,
                                     uint8_t reg,
                                     uint8_t *data,
                                     size_t len)
{
    bmi088_status_t st;

    if ((data == NULL) && (len > 0U)) {
        return BMI088_ERROR_NULL;
    }

    dev->bus.gyro_cs(dev->bus.user, true);
    st = txrx(dev, reg | BMI088_READ_BIT, NULL);
    for (size_t i = 0U; (st == BMI088_OK) && (i < len); ++i) {
        st = txrx(dev, 0x00U, &data[i]);
    }
    dev->bus.gyro_cs(dev->bus.user, false);

    return st;
}

static bmi088_status_t validate_bus(const bmi088_bus_t *bus)
{
    if (bus == NULL) {
        return BMI088_ERROR_NULL;
    }
    if ((bus->spi_txrx_byte == NULL)
        || (bus->accel_cs == NULL)
        || (bus->gyro_cs == NULL)) {
        return BMI088_ERROR_NULL;
    }
    return BMI088_OK;
}

void bmi088_default_config(bmi088_config_t *cfg)
{
    if (cfg == NULL) {
        return;
    }

    cfg->accel_range = BMI088_ACCEL_RANGE_3G;
    cfg->gyro_range = BMI088_GYRO_RANGE_2000_DPS;
    cfg->accel_conf = 0xA8U;
    cfg->gyro_bandwidth = 0x07U;
}

bmi088_status_t bmi088_init(bmi088_t *dev,
                            const bmi088_bus_t *bus,
                            const bmi088_config_t *cfg)
{
    bmi088_config_t local_cfg;
    uint8_t accel_id = 0U;
    uint8_t gyro_id = 0U;
    bmi088_status_t st;

    if (dev == NULL) {
        return BMI088_ERROR_NULL;
    }

    st = validate_bus(bus);
    if (st != BMI088_OK) {
        return st;
    }

    if (cfg == NULL) {
        bmi088_default_config(&local_cfg);
        cfg = &local_cfg;
    }

    const uint8_t accel_g = accel_range_g(cfg->accel_range);
    const uint16_t gyro_dps = gyro_range_dps(cfg->gyro_range);
    if ((accel_g == 0U) || (gyro_dps == 0U)) {
        return BMI088_ERROR_BAD_ARG;
    }

    memset(dev, 0, sizeof(*dev));
    dev->bus = *bus;
    dev->cfg = *cfg;
    dev->accel_m_s2_per_lsb = ((float)accel_g * BMI088_GRAVITY_M_S2) / 32768.0f;
    dev->gyro_rad_s_per_lsb = ((float)gyro_dps * BMI088_DEG_TO_RAD) / 32768.0f;

    dev->bus.accel_cs(dev->bus.user, false);
    dev->bus.gyro_cs(dev->bus.user, false);
    delay_ms(dev, 2U);

    st = bmi088_read_ids(dev, &accel_id, &gyro_id);
    if (st != BMI088_OK) {
        return st;
    }
    if ((accel_id != BMI088_ACCEL_CHIP_ID_VALUE)
        || (gyro_id != BMI088_GYRO_CHIP_ID_VALUE)) {
        return BMI088_ERROR_BAD_ID;
    }

    st = accel_write_reg(dev, BMI088_ACCEL_PWR_CTRL, 0x04U);
    if (st != BMI088_OK) {
        return st;
    }
    delay_ms(dev, 5U);

    st = accel_write_reg(dev, BMI088_ACCEL_PWR_CONF, 0x00U);
    if (st != BMI088_OK) {
        return st;
    }
    delay_ms(dev, 50U);

    st = accel_write_reg(dev, BMI088_ACCEL_RANGE, (uint8_t)cfg->accel_range);
    if (st != BMI088_OK) {
        return st;
    }

    st = accel_write_reg(dev, BMI088_ACCEL_CONF, cfg->accel_conf);
    if (st != BMI088_OK) {
        return st;
    }

    st = gyro_write_reg(dev, BMI088_GYRO_LPM1, 0x00U);
    if (st != BMI088_OK) {
        return st;
    }

    st = gyro_write_reg(dev, BMI088_GYRO_RANGE, (uint8_t)cfg->gyro_range);
    if (st != BMI088_OK) {
        return st;
    }

    st = gyro_write_reg(dev, BMI088_GYRO_BANDWIDTH, cfg->gyro_bandwidth);
    if (st != BMI088_OK) {
        return st;
    }
    delay_ms(dev, 10U);

    return BMI088_OK;
}

bmi088_status_t bmi088_read_ids(bmi088_t *dev,
                                uint8_t *accel_id,
                                uint8_t *gyro_id)
{
    bmi088_status_t st;

    if ((dev == NULL) || (accel_id == NULL) || (gyro_id == NULL)) {
        return BMI088_ERROR_NULL;
    }

    st = accel_read_reg(dev, BMI088_ACCEL_CHIP_ID, accel_id, 1U);
    if (st != BMI088_OK) {
        return st;
    }

    return gyro_read_reg(dev, BMI088_GYRO_CHIP_ID, gyro_id, 1U);
}

bmi088_status_t bmi088_read_raw(bmi088_t *dev, bmi088_raw_t *raw)
{
    uint8_t acc_bytes[6];
    uint8_t gyro_bytes[6];
    bmi088_status_t st;

    if ((dev == NULL) || (raw == NULL)) {
        return BMI088_ERROR_NULL;
    }

    st = accel_read_reg(dev, BMI088_ACCEL_X_LSB, acc_bytes, sizeof(acc_bytes));
    if (st != BMI088_OK) {
        return st;
    }

    st = gyro_read_reg(dev, BMI088_GYRO_X_LSB, gyro_bytes, sizeof(gyro_bytes));
    if (st != BMI088_OK) {
        return st;
    }

    raw->accel[0] = i16_le(&acc_bytes[0]);
    raw->accel[1] = i16_le(&acc_bytes[2]);
    raw->accel[2] = i16_le(&acc_bytes[4]);
    raw->gyro[0] = i16_le(&gyro_bytes[0]);
    raw->gyro[1] = i16_le(&gyro_bytes[2]);
    raw->gyro[2] = i16_le(&gyro_bytes[4]);

    return BMI088_OK;
}

void bmi088_convert(const bmi088_t *dev,
                    const bmi088_raw_t *raw,
                    bmi088_data_t *data)
{
    if ((dev == NULL) || (raw == NULL) || (data == NULL)) {
        return;
    }

    for (uint8_t i = 0U; i < 3U; ++i) {
        data->accel_m_s2[i] = (float)raw->accel[i] * dev->accel_m_s2_per_lsb;
        data->gyro_rad_s[i] = (float)raw->gyro[i] * dev->gyro_rad_s_per_lsb;
    }
}

bmi088_status_t bmi088_read_converted(bmi088_t *dev, bmi088_data_t *data)
{
    bmi088_raw_t raw;
    bmi088_status_t st;

    if ((dev == NULL) || (data == NULL)) {
        return BMI088_ERROR_NULL;
    }

    st = bmi088_read_raw(dev, &raw);
    if (st != BMI088_OK) {
        return st;
    }

    bmi088_convert(dev, &raw, data);
    return BMI088_OK;
}
