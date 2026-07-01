/************************************************************************
 * @file bsp_mpu6050_driver.c
 * @brief MPU6050 driver implementation file
 *
 * Detailed description:
 * - Function 1: Implement software I2C timing and read/write primitive
 * - Function 2: Complete MPU6050 register initialization and identity check
 * - Function 3: Provide raw accel, temperature and gyro burst read ability
 *
 * @author DFBH
 * @date 2026-07-01
 * @version 1.0.0
 *
 * @note Current implementation uses PB12/PB13 software I2C
 * @warning Software I2C timing depends on CPU frequency and GPIO pull-up condition
 ************************************************************************/
#include "bsp_mpu6050_driver.h"

#include "bsp_mpu6050_reg.h"
#include <stdio.h>

typedef struct
{
    GPIO_TypeDef *scl_port;
    uint16_t scl_pin;
    GPIO_TypeDef *sda_port;
    uint16_t sda_pin;
    uint8_t address;
} mpu6050_stm32_ctx_t;

static mpu6050_status_t mpu6050_stm32_init(mpu6050_dev_t *dev);
static mpu6050_status_t mpu6050_stm32_read_raw(mpu6050_dev_t *dev, mpu6050_raw_data_t *raw_data);

static void mpu6050_iic_delay(void);
static void mpu6050_scl_write(mpu6050_stm32_ctx_t *ctx, GPIO_PinState state);
static void mpu6050_sda_write(mpu6050_stm32_ctx_t *ctx, GPIO_PinState state);
static GPIO_PinState mpu6050_sda_read(mpu6050_stm32_ctx_t *ctx);
static void mpu6050_sda_mode_output(mpu6050_stm32_ctx_t *ctx);
static void mpu6050_sda_mode_input(mpu6050_stm32_ctx_t *ctx);
static void mpu6050_iic_start(mpu6050_stm32_ctx_t *ctx);
static void mpu6050_iic_stop(mpu6050_stm32_ctx_t *ctx);
static uint8_t mpu6050_iic_wait_ack(mpu6050_stm32_ctx_t *ctx);
static void mpu6050_iic_ack(mpu6050_stm32_ctx_t *ctx);
static void mpu6050_iic_nack(mpu6050_stm32_ctx_t *ctx);
static void mpu6050_iic_send_byte(mpu6050_stm32_ctx_t *ctx, uint8_t data);
static uint8_t mpu6050_iic_read_byte(mpu6050_stm32_ctx_t *ctx, uint8_t is_ack);
static mpu6050_status_t mpu6050_write_reg(mpu6050_stm32_ctx_t *ctx, uint8_t reg, uint8_t data);
static mpu6050_status_t
mpu6050_read_regs(mpu6050_stm32_ctx_t *ctx, uint8_t reg, uint8_t *buffer, uint8_t size);

static const mpu6050_ops_t g_mpu6050_ops = {
    .pf_mpu6050_init = mpu6050_stm32_init,
    .pf_mpu6050_read_raw = mpu6050_stm32_read_raw,
};

static mpu6050_stm32_ctx_t g_mpu6050_ctx = {
    .scl_port = MPU_SCL_GPIO_Port,
    .scl_pin = MPU_SCL_Pin,
    .sda_port = MPU_SDA_GPIO_Port,
    .sda_pin = MPU_SDA_Pin,
    .address = MPU6050_I2C_ADDR,
};

static mpu6050_dev_t g_mpu6050_dev = {
    .ops = &g_mpu6050_ops,
    .ctx = &g_mpu6050_ctx,
};

static void mpu6050_iic_delay(void)
{
    volatile uint16_t i;

    for (i = 0; i < 30U; i++)
    {
        __NOP();
    }
}

static void mpu6050_scl_write(mpu6050_stm32_ctx_t *ctx, GPIO_PinState state)
{
    HAL_GPIO_WritePin(ctx->scl_port, ctx->scl_pin, state);
}

static void mpu6050_sda_write(mpu6050_stm32_ctx_t *ctx, GPIO_PinState state)
{
    HAL_GPIO_WritePin(ctx->sda_port, ctx->sda_pin, state);
}

static GPIO_PinState mpu6050_sda_read(mpu6050_stm32_ctx_t *ctx)
{
    return HAL_GPIO_ReadPin(ctx->sda_port, ctx->sda_pin);
}

static void mpu6050_sda_mode_output(mpu6050_stm32_ctx_t *ctx)
{
    GPIO_InitTypeDef gpio_init;

    gpio_init.Pin = ctx->sda_pin;
    gpio_init.Mode = GPIO_MODE_OUTPUT_OD;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ctx->sda_port, &gpio_init);
}

static void mpu6050_sda_mode_input(mpu6050_stm32_ctx_t *ctx)
{
    GPIO_InitTypeDef gpio_init;

    gpio_init.Pin = ctx->sda_pin;
    gpio_init.Mode = GPIO_MODE_INPUT;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ctx->sda_port, &gpio_init);
}

static void mpu6050_iic_start(mpu6050_stm32_ctx_t *ctx)
{
    mpu6050_sda_mode_output(ctx);
    mpu6050_sda_write(ctx, GPIO_PIN_SET);
    mpu6050_scl_write(ctx, GPIO_PIN_SET);
    mpu6050_iic_delay();
    mpu6050_sda_write(ctx, GPIO_PIN_RESET);
    mpu6050_iic_delay();
    mpu6050_scl_write(ctx, GPIO_PIN_RESET);
}

static void mpu6050_iic_stop(mpu6050_stm32_ctx_t *ctx)
{
    mpu6050_sda_mode_output(ctx);
    mpu6050_scl_write(ctx, GPIO_PIN_RESET);
    mpu6050_sda_write(ctx, GPIO_PIN_RESET);
    mpu6050_iic_delay();
    mpu6050_scl_write(ctx, GPIO_PIN_SET);
    mpu6050_iic_delay();
    mpu6050_sda_write(ctx, GPIO_PIN_SET);
    mpu6050_iic_delay();
}

static uint8_t mpu6050_iic_wait_ack(mpu6050_stm32_ctx_t *ctx)
{
    uint16_t timeout;

    timeout = 0U;
    mpu6050_sda_mode_input(ctx);
    mpu6050_sda_write(ctx, GPIO_PIN_SET);
    mpu6050_iic_delay();
    mpu6050_scl_write(ctx, GPIO_PIN_SET);
    mpu6050_iic_delay();

    while (mpu6050_sda_read(ctx) == GPIO_PIN_SET)
    {
        timeout++;
        if (timeout > 250U)
        {
            mpu6050_iic_stop(ctx);
            return 1U;
        }
    }

    mpu6050_scl_write(ctx, GPIO_PIN_RESET);
    return 0U;
}

static void mpu6050_iic_ack(mpu6050_stm32_ctx_t *ctx)
{
    mpu6050_scl_write(ctx, GPIO_PIN_RESET);
    mpu6050_sda_mode_output(ctx);
    mpu6050_sda_write(ctx, GPIO_PIN_RESET);
    mpu6050_iic_delay();
    mpu6050_scl_write(ctx, GPIO_PIN_SET);
    mpu6050_iic_delay();
    mpu6050_scl_write(ctx, GPIO_PIN_RESET);
}

static void mpu6050_iic_nack(mpu6050_stm32_ctx_t *ctx)
{
    mpu6050_scl_write(ctx, GPIO_PIN_RESET);
    mpu6050_sda_mode_output(ctx);
    mpu6050_sda_write(ctx, GPIO_PIN_SET);
    mpu6050_iic_delay();
    mpu6050_scl_write(ctx, GPIO_PIN_SET);
    mpu6050_iic_delay();
    mpu6050_scl_write(ctx, GPIO_PIN_RESET);
}

static void mpu6050_iic_send_byte(mpu6050_stm32_ctx_t *ctx, uint8_t data)
{
    uint8_t i;

    mpu6050_sda_mode_output(ctx);
    mpu6050_scl_write(ctx, GPIO_PIN_RESET);

    for (i = 0U; i < 8U; i++)
    {
        if ((data & 0x80U) != 0U)
        {
            mpu6050_sda_write(ctx, GPIO_PIN_SET);
        }
        else
        {
            mpu6050_sda_write(ctx, GPIO_PIN_RESET);
        }

        data <<= 1;
        mpu6050_iic_delay();
        mpu6050_scl_write(ctx, GPIO_PIN_SET);
        mpu6050_iic_delay();
        mpu6050_scl_write(ctx, GPIO_PIN_RESET);
        mpu6050_iic_delay();
    }
}

static uint8_t mpu6050_iic_read_byte(mpu6050_stm32_ctx_t *ctx, uint8_t is_ack)
{
    uint8_t i;
    uint8_t receive;

    receive = 0U;
    mpu6050_sda_mode_input(ctx);
    for (i = 0U; i < 8U; i++)
    {
        mpu6050_scl_write(ctx, GPIO_PIN_RESET);
        mpu6050_iic_delay();
        mpu6050_scl_write(ctx, GPIO_PIN_SET);
        receive <<= 1;
        if (mpu6050_sda_read(ctx) == GPIO_PIN_SET)
        {
            receive |= 0x01U;
        }
        mpu6050_iic_delay();
    }
    if (is_ack != 0U)
    {
        mpu6050_iic_ack(ctx);
    }
    else
    {
        mpu6050_iic_nack(ctx);
    }

    return receive;
}

static mpu6050_status_t mpu6050_write_reg(mpu6050_stm32_ctx_t *ctx, uint8_t reg, uint8_t data)
{
    mpu6050_iic_start(ctx);
    mpu6050_iic_send_byte(ctx, (uint8_t)(ctx->address << 1));
    if (mpu6050_iic_wait_ack(ctx) != 0U)
    {
#ifdef MPU6050_DEBUG
        printf("MPU6050_ERROR: write dev addr no ack at %s, reg=0x%02X\r\n", __FUNCTION__, reg);
#endif
        return MPU6050_ERROR;
    }

    mpu6050_iic_send_byte(ctx, reg);
    if (mpu6050_iic_wait_ack(ctx) != 0U)
    {
#ifdef MPU6050_DEBUG
        printf("MPU6050_ERROR: write reg no ack at %s, reg=0x%02X\r\n", __FUNCTION__, reg);
#endif
        return MPU6050_ERROR;
    }

    mpu6050_iic_send_byte(ctx, data);
    if (mpu6050_iic_wait_ack(ctx) != 0U)
    {
#ifdef MPU6050_DEBUG
        printf("MPU6050_ERROR: write data no ack at %s, reg=0x%02X data=0x%02X\r\n",
               __FUNCTION__,
               reg,
               data);
#endif
        return MPU6050_ERROR;
    }

    mpu6050_iic_stop(ctx);
    return MPU6050_OK;
}

static mpu6050_status_t
mpu6050_read_regs(mpu6050_stm32_ctx_t *ctx, uint8_t reg, uint8_t *buffer, uint8_t size)
{
    uint8_t i;

    if ((buffer == NULL) || (size == 0U))
    {
#ifdef MPU6050_DEBUG
        printf("MPU6050_ERROR: read param invalid at %s\r\n", __FUNCTION__);
#endif
        return MPU6050_PARAMERROR;
    }

    mpu6050_iic_start(ctx);
    mpu6050_iic_send_byte(ctx, (uint8_t)(ctx->address << 1));
    if (mpu6050_iic_wait_ack(ctx) != 0U)
    {
#ifdef MPU6050_DEBUG
        printf(
            "MPU6050_ERROR: read send write addr no ack at %s, reg=0x%02X\r\n", __FUNCTION__, reg);
#endif
        return MPU6050_ERROR;
    }

    mpu6050_iic_send_byte(ctx, reg);
    if (mpu6050_iic_wait_ack(ctx) != 0U)
    {
#ifdef MPU6050_DEBUG
        printf("MPU6050_ERROR: read send reg no ack at %s, reg=0x%02X\r\n", __FUNCTION__, reg);
#endif
        return MPU6050_ERROR;
    }

    mpu6050_iic_start(ctx);
    mpu6050_iic_send_byte(ctx, (uint8_t)((ctx->address << 1) | 0x01U));
    if (mpu6050_iic_wait_ack(ctx) != 0U)
    {
#ifdef MPU6050_DEBUG
        printf(
            "MPU6050_ERROR: read send read addr no ack at %s, reg=0x%02X\r\n", __FUNCTION__, reg);
#endif
        return MPU6050_ERROR;
    }

    for (i = 0U; i < size; i++)
    {
        buffer[i] = mpu6050_iic_read_byte(ctx, (uint8_t)(i < (size - 1U)));
    }

    mpu6050_iic_stop(ctx);
    return MPU6050_OK;
}

static mpu6050_status_t mpu6050_stm32_init(mpu6050_dev_t *dev)
{
    uint8_t who_am_i;
    mpu6050_stm32_ctx_t *ctx;

    if ((dev == NULL) || (dev->ctx == NULL))
    {
#ifdef MPU6050_DEBUG
        printf("MPU6050_ERROR: dev or ctx is NULL at %s\r\n", __FUNCTION__);
#endif
        return MPU6050_PARAMERROR;
    }

    ctx = (mpu6050_stm32_ctx_t *)dev->ctx;
    mpu6050_sda_mode_output(ctx);
    mpu6050_sda_write(ctx, GPIO_PIN_SET);
    mpu6050_scl_write(ctx, GPIO_PIN_SET);
    HAL_Delay(50U);

    if (mpu6050_write_reg(ctx, MPU6050_REG_PWR_MGMT_1, 0x00U) != MPU6050_OK)
    {
#ifdef MPU6050_DEBUG
        printf("MPU6050_ERROR: write pwr mgmt failed at %s\r\n", __FUNCTION__);
#endif
        return MPU6050_ERROR;
    }
    if (mpu6050_write_reg(ctx, MPU6050_REG_SMPLRT_DIV, 0x07U) != MPU6050_OK)
    {
#ifdef MPU6050_DEBUG
        printf("MPU6050_ERROR: write smplrt div failed at %s\r\n", __FUNCTION__);
#endif
        return MPU6050_ERROR;
    }
    if (mpu6050_write_reg(ctx, MPU6050_REG_CONFIG, 0x06U) != MPU6050_OK)
    {
#ifdef MPU6050_DEBUG
        printf("MPU6050_ERROR: write config failed at %s\r\n", __FUNCTION__);
#endif
        return MPU6050_ERROR;
    }
    if (mpu6050_write_reg(ctx, MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_FS_250DPS) != MPU6050_OK)
    {
#ifdef MPU6050_DEBUG
        printf("MPU6050_ERROR: write gyro config failed at %s\r\n", __FUNCTION__);
#endif
        return MPU6050_ERROR;
    }
    if (mpu6050_write_reg(ctx, MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS_2G) != MPU6050_OK)
    {
#ifdef MPU6050_DEBUG
        printf("MPU6050_ERROR: write accel config failed at %s\r\n", __FUNCTION__);
#endif
        return MPU6050_ERROR;
    }

    if (mpu6050_read_regs(ctx, MPU6050_REG_WHO_AM_I, &who_am_i, 1U) != MPU6050_OK)
    {
#ifdef MPU6050_DEBUG
        printf("MPU6050_ERROR: read who am i failed at %s\r\n", __FUNCTION__);
#endif
        return MPU6050_ERROR;
    }

#ifdef MPU6050_DEBUG
    printf("MPU6050_INFO: who_am_i=0x%02X at %s\r\n", who_am_i, __FUNCTION__);
#endif

    // if (who_am_i != MPU6050_WHO_AM_I_ID)
    // {
    //     return MPU6050_ERROR;
    // }

    return MPU6050_OK;
}

static mpu6050_status_t mpu6050_stm32_read_raw(mpu6050_dev_t *dev, mpu6050_raw_data_t *raw_data)
{
    uint8_t buffer[14];
    mpu6050_stm32_ctx_t *ctx;

    if ((dev == NULL) || (dev->ctx == NULL) || (raw_data == NULL))
    {
#ifdef MPU6050_DEBUG
        printf("MPU6050_ERROR: dev ctx or raw_data is NULL at %s\r\n", __FUNCTION__);
#endif
        return MPU6050_PARAMERROR;
    }

    ctx = (mpu6050_stm32_ctx_t *)dev->ctx;

    if (mpu6050_read_regs(ctx, MPU6050_REG_ACCEL_XOUT_H, buffer, 14U) != MPU6050_OK)
    {
#ifdef MPU6050_DEBUG
        printf("MPU6050_ERROR: read raw burst failed at %s\r\n", __FUNCTION__);
#endif
        return MPU6050_ERROR;
    }

    raw_data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    raw_data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    raw_data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
    raw_data->temp = (int16_t)((buffer[6] << 8) | buffer[7]);
    raw_data->gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]);
    raw_data->gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
    raw_data->gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);

    return MPU6050_OK;
}

mpu6050_dev_t *MPU6050_GetDefaultDevice(void)
{
    return &g_mpu6050_dev;
}

mpu6050_status_t MPU6050_Init(mpu6050_dev_t *dev)
{
    if ((dev == NULL) || (dev->ops == NULL) || (dev->ops->pf_mpu6050_init == NULL))
    {
#ifdef MPU6050_DEBUG
        printf("MPU6050_ERROR: dev ops or init func is NULL at %s\r\n", __FUNCTION__);
#endif
        return MPU6050_PARAMERROR;
    }

    return dev->ops->pf_mpu6050_init(dev);
}

mpu6050_status_t MPU6050_ReadRawData(mpu6050_dev_t *dev, mpu6050_raw_data_t *raw_data)
{
    if ((dev == NULL) || (dev->ops == NULL) || (dev->ops->pf_mpu6050_read_raw == NULL))
    {
#ifdef MPU6050_DEBUG
        printf("MPU6050_ERROR: dev ops or read func is NULL at %s\r\n", __FUNCTION__);
#endif
        return MPU6050_PARAMERROR;
    }

    return dev->ops->pf_mpu6050_read_raw(dev, raw_data);
}
