/************************************************************************
 * @file app_task.c
 * @brief APP 层任务注册实现文件
 *
 * 详细描述:
 * - 功能1: 统一管理 APP 层任务创建入口
 * - 功能2: 集中调度各业务模块任务注册流程
 * - 功能3: 避免将所有任务函数直接堆叠在同一文件中
 *
 * @author DFBH
 * @date 2026-06-18
 * @version 1.0.0
 *
 * @note 本文件只负责任务注册编排, 具体任务实现应放在对应业务模块内
 * @warning 新增业务任务时, 建议只在此处增加创建调用, 不直接写任务主体
 ************************************************************************/
#include "app.h"

#include "app_algorithm.h"
#include "app_imu.h"
#include "app_ir.h"
#include "app_motor.h"

/**
 * @brief APP 层任务统一创建入口
 *
 * 功能说明:
 * - 创建算法任务
 * - 创建红外业务任务
 * - 创建电机控制任务
 *
 * @retval None
 */
void App_TaskCreate(void)
{
    AppAlgorithm_TaskCreate();
    AppImu_TaskCreate();
    AppIr_TaskCreate();
    AppMotor_TaskCreate();
}
