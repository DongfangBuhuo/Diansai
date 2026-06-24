/************************************************************************
 * @file app_ir.c
 * @brief APP 层红外模块实现文件
 *
 * 详细描述:
 * - 功能1: 完成 APP 层红外模块初始化
 * - 功能2: 创建红外采集任务
 * - 功能3: 周期性读取红外状态并输出调试信息
 *
 * @author DFBH
 * @date 2026-06-18
 * @version 1.0.0
 *
 * @note 当前实现主要用于任务组织与基础读数验证
 * @warning 当前任务内使用 printf 输出调试信息, 频率过高时需关注串口开销
 ************************************************************************/
#include "app_ir.h"

#include "FreeRTOS.h"
#include "app_algorithm.h"
#include "bsp_ir_driver.h"
#include "task.h"
#include <stdio.h>

static void AppIrTask(void *argument);
/**
 * @brief APP 层红外模块初始化
 *
 * 功能说明:
 * - 获取默认红外设备对象
 * - 调用 BSP IR 层初始化接口
 *
 * @retval None
 */
void AppIr_Init(void)
{
    ir_dev_t *ir_dev;

    ir_dev = IR_GetDefaultDevice();
    (void)IR_Init(ir_dev);
}

/**
 * @brief 创建红外任务
 *
 * 功能说明:
 * - 获取默认红外设备对象
 * - 将设备对象作为任务参数创建红外采集任务
 *
 * @retval None
 */
void AppIr_TaskCreate(void)
{
    ir_dev_t *ir_dev;

    ir_dev = IR_GetDefaultDevice();
    (void)xTaskCreate(AppIrTask, "appIrTask", 128, ir_dev, tskIDLE_PRIORITY + 1U, NULL);
}

/**
 * @brief 红外采集任务函数
 *
 * 功能说明:
 * - 启动后延时等待系统稳定
 * - 周期性读取红外状态寄存器
 * - 将读取结果输出到调试串口
 *
 * @param argument 任务参数, 当前传入默认红外设备指针
 * @retval None
 */
static void AppIrTask(void *argument)
{
    ir_dev_t *ir_dev;
    app_ir_data_t ir_data;
    uint8_t data;

    ir_dev = (ir_dev_t *)argument;
    data = 0U;

    vTaskDelay(pdMS_TO_TICKS(3000U));

    for (;;)
    {
        if (IR_ReadState(ir_dev, &data) == IR_OK)
        {
            // printf("data is %d\r\n", data);
        }
        ir_data.data.x1 = (data >> 7) & 0x01;
        ir_data.data.x2 = (data >> 6) & 0x01;
        ir_data.data.x3 = (data >> 5) & 0x01;
        ir_data.data.x4 = (data >> 4) & 0x01;
        ir_data.data.x5 = (data >> 3) & 0x01;
        ir_data.data.x6 = (data >> 2) & 0x01;
        ir_data.data.x7 = (data >> 1) & 0x01;
        ir_data.data.x8 = (data >> 0) & 0x01;
        // 从左到右依次是x1-x8 ，0表示黑线
        AppAlgorithmGetIr(&ir_data);

        vTaskDelay(pdMS_TO_TICKS(30U));
    }
}
