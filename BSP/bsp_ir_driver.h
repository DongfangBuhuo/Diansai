/************************************************************************
 * @file bsp_ir_driver.h
 * @brief 文件功能简述
 *
 *
 * @author DFBH
 * @date 2026-06-15
 * @version 1.0.0
 *
 * @note 备注信息 (可选)
 * @warning 警告信息 (可选)
 ************************************************************************/
#ifndef __BSP_IR_I2C_H
#define __BSP_IR_I2C_H
#include "stm32f4xx_hal.h"

//#define IR_DEBUG

typedef enum
{
    IR_OK,
    IR_ERROR,
} ir_status_t;

typedef struct ir_dev ir_dev_t;

typedef struct
{
    ir_status_t (*pf_ir_init)(ir_dev_t *dev);
    ir_status_t (*pf_ir_calibration_mode)(ir_dev_t *dev, uint8_t is_on);
    ir_status_t (*pf_ir_read_state)(ir_dev_t *dev, uint8_t *receive);
} ir_ops_t;

struct ir_dev
{
    const ir_ops_t *ops;
    void *ctx;
};

ir_dev_t *IR_GetDefaultDevice(void);
ir_status_t IR_Init(ir_dev_t *dev);
ir_status_t IR_SetCalibrationMode(ir_dev_t *dev, uint8_t is_on);
ir_status_t IR_ReadState(ir_dev_t *dev, uint8_t *receive);

#endif // !__BSP_IR_I2C_H
