/************************************************************************
 * @file bsp_ir_driver.c
 * @brief ir灰度传感器驱动c文件
 *
 * 详细描述:
 * - 功能1: 具体描述
 * - 功能2: 具体描述
 *
 * @author DFBH
 * @date 2026-06-18
 * @version 1.0.0
 *
 * @note 备注信息 (可选)
 * @warning 警告信息 (可选)
 ************************************************************************/
#include "bsp_ir_driver.h"

#include <stdio.h>

#include "bsp_ir_reg.h"
#include "i2c.h"

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint16_t address;
} ir_stm32_ctx_t;

static ir_status_t ir_stm32_init(ir_dev_t *dev);
static ir_status_t ir_stm32_calibration_mode(ir_dev_t *dev, uint8_t is_on);
static ir_status_t ir_stm32_read_state(ir_dev_t *dev, uint8_t *receive);

static const ir_ops_t g_ir_ops = {
    .pf_ir_init = ir_stm32_init,
    .pf_ir_calibration_mode = ir_stm32_calibration_mode,
    .pf_ir_read_state = ir_stm32_read_state,
};

static ir_stm32_ctx_t g_ir_ctx = {
    .hi2c = &hi2c1,
    .address = IR_ADDRESS,
};

static ir_dev_t g_ir_dev = {
    .ops = &g_ir_ops,
    .ctx = &g_ir_ctx,
};

static ir_status_t ir_stm32_init(ir_dev_t *dev)
{
    (void)dev;
    return IR_OK;
}

static ir_status_t ir_stm32_calibration_mode(ir_dev_t *dev, uint8_t is_on)
{
    ir_stm32_ctx_t *ctx;

    if ((dev == NULL) || (dev->ctx == NULL))
    {
        return IR_ERROR;
    }

    ctx = (ir_stm32_ctx_t *)dev->ctx;
    is_on = (uint8_t)(is_on != 0U);

    if (HAL_OK !=
        HAL_I2C_Mem_Write(
            ctx->hi2c, ctx->address, IR_CALIBRATION_REG, I2C_MEMADD_SIZE_8BIT, &is_on, 1, 100))
    {
        return IR_ERROR;
    }

    return IR_OK;
}

static ir_status_t ir_stm32_read_state(ir_dev_t *dev, uint8_t *receive)
{
    ir_stm32_ctx_t *ctx;

    if ((dev == NULL) || (dev->ctx == NULL) || (receive == NULL))
    {
        return IR_ERROR;
    }

    ctx = (ir_stm32_ctx_t *)dev->ctx;

    if (HAL_OK != HAL_I2C_Mem_Read(
                      ctx->hi2c, ctx->address, IR_STATE_REG, I2C_MEMADD_SIZE_8BIT, receive, 1, 100))
    {
#ifdef IR_DEBUG
        printf("I2C Err: State=%d, ErrorCode=0x%lx\r\n", ctx->hi2c->State, ctx->hi2c->ErrorCode);
#endif
        return IR_ERROR;
    }

    return IR_OK;
}

ir_dev_t *IR_GetDefaultDevice(void)
{
    return &g_ir_dev;
}

ir_status_t IR_Init(ir_dev_t *dev)
{
    if ((dev == NULL) || (dev->ops == NULL) || (dev->ops->pf_ir_init == NULL))
    {
        return IR_ERROR;
    }

    return dev->ops->pf_ir_init(dev);
}

ir_status_t IR_SetCalibrationMode(ir_dev_t *dev, uint8_t is_on)
{
    if ((dev == NULL) || (dev->ops == NULL) || (dev->ops->pf_ir_calibration_mode == NULL))
    {
        return IR_ERROR;
    }

    return dev->ops->pf_ir_calibration_mode(dev, is_on);
}

ir_status_t IR_ReadState(ir_dev_t *dev, uint8_t *receive)
{
    if ((dev == NULL) || (dev->ops == NULL) || (dev->ops->pf_ir_read_state == NULL))
    {
        return IR_ERROR;
    }

    return dev->ops->pf_ir_read_state(dev, receive);
}
