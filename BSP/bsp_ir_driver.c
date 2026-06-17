/************************************************************************
 * @file bsp_ir_driver.c
 * @brief 文件功能简述
 *
 * 详细描述:
 * - 功能1: 具体描述
 * - 功能2: 具体描述
 *
 * @author DFBH
 * @date 2026-06-15
 * @version 1.0.0
 *
 * @note 备注信息 (可选)
 * @warning 警告信息 (可选)
 ************************************************************************/
#include "bsp_ir_driver.h"
#include "bsp_ir_reg.h"
#include "i2c.h"
ir_ops_t ir_ops;
/**
 * @brief ir 校准模式
 * is_on = 1开启 is_on =0 关闭
 * @param 参数名 is_on
 * @return
 */
ir_status_t ir_calibration_mode(uint8_t is_on)
{
    is_on = (is_on != 0);
    if (HAL_OK != HAL_I2C_Mem_Write(&hi2c1, IR_ADDRESS, IR_CALIBRATION_REG, I2C_MEMADD_SIZE_8BIT, &is_on, 1, 100))
    {
        return IR_ERROR;
    }
    return IR_OK;
}
/**
 * @brief 读取ir寄存器状态
 *
 * @param 参数名 参数说明
 * @return 返回值说明
 */
ir_status_t ir_read_state(uint8_t *receive)
{
    if (HAL_OK !=HAL_I2C_Mem_Read(&hi2c1, IR_ADDRESS, IR_STATE_REG, I2C_MEMADD_SIZE_8BIT, receive, 1, 100))
    {
        printf("I2C Err: State=%d, ErrorCode=0x%lx\r\n", hi2c1.State, hi2c1.ErrorCode);
        return IR_ERROR;
    }
    return IR_OK;
}
void ir_init()
{
    ir_ops.pf_ir_calibration_mode = ir_calibration_mode;
    ir_ops.pf_ir_read_state = ir_read_state;
}