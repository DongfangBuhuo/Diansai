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
typedef enum
{
    IR_OK,
    IR_ERROR,
} ir_status_t;
typedef struct
{
    // ir_status_t (*pf_ir_iic_start)(void *);
    // ir_status_t (*pf_ir_iic_stop)(void *);
    ir_status_t (*pf_ir_calibration_mode)(uint8_t is_on);
    ir_status_t (*pf_ir_read_state)(uint8_t *receive);
    // ir_status_t (*pf_iic_wait_ack)(void *); /*   IIC w-ack   */
    // ir_status_t (*pf_iic_send_ack)(void *); /*   IIC s-ack   */
} ir_ops_t;
extern ir_ops_t ir_ops;
void ir_init();

#endif // !__BSP_IR_I2C_H