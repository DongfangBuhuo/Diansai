/************************************************************************
 * @file app.h
 * @brief APP 层总入口头文件
 *
 * 详细描述:
 * - 功能1: 声明 APP 层统一初始化入口
 * - 功能2: 声明 APP 层统一任务创建入口
 * - 功能3: 作为 APP 各模块的总调度头文件
 *
 * @author DFBH
 * @date 2026-06-18
 * @version 1.0.0
 *
 * @note APP 层负责组织业务模块初始化与任务创建, 不直接操作底层硬件
 * @warning 调用 App_TaskCreate 前应先完成系统外设初始化与 App_Init
 ************************************************************************/
#ifndef __APP_H
#define __APP_H

/**
 * @brief APP 层初始化入口
 *
 * 功能说明:
 * - 初始化日志系统
 * - 初始化 APP 层依赖的业务模块
 * - 为后续任务创建提供运行基础
 *
 * @retval None
 */
void App_Init(void);

/**
 * @brief APP 层任务创建入口
 *
 * 功能说明:
 * - 统一创建 APP 层业务任务
 * - 集中管理各业务模块任务注册
 *
 * @retval None
 */
void App_TaskCreate(void);

#endif /* __APP_H */
