/************************************************************************
 * @file app.c
 * @brief APP 层总入口实现文件
 *
 * 详细描述:
 * - 功能1: 初始化 APP 层公共能力, 如日志系统
 * - 功能2: 统一调用各业务模块初始化接口
 * - 功能3: 为系统任务启动前提供应用层准备动作
 *
 * @author DFBH
 * @date 2026-06-18
 * @version 1.0.0
 *
 * @note 本文件负责 APP 层“初始化编排”, 不直接承载具体业务任务逻辑
 * @warning 若新增 APP 模块, 应在 App_Init 中统一接入初始化流程
 ************************************************************************/
#include "app.h"

#include "app_algorithm.h"
#include "app_imu.h"
#include "app_ir.h"
#include "app_motor.h"
#include "elog.h"

/**
 * @brief APP 层日志系统初始化
 *
 * 功能说明:
 * - 初始化 EasyLogger
 * - 配置各日志等级输出格式
 * - 启动日志系统
 *
 * @retval None
 */
static void App_LogInit(void)
{
    elog_init();
    elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_ALL);
    elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_ALL);
    elog_set_fmt(ELOG_LVL_WARN, ELOG_FMT_ALL);
    elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_ALL);
    elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_ALL);
    elog_set_fmt(ELOG_LVL_VERBOSE, ELOG_FMT_ALL);
    elog_start();
}

/**
 * @brief APP 层初始化入口
 *
 * 功能说明:
 * - 初始化日志系统
 * - 初始化算法模块
 * - 初始化红外模块
 * - 初始化电机模块
 *
 * @retval None
 */
void App_Init(void)
{
    App_LogInit();
    AppAlgorithm_Init();
    AppImu_Init();
    AppIr_Init();
    AppMotor_Init();
}
