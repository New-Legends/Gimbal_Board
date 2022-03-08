/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_control.c/h
  * @brief      chassis power control.底盘功率控制
  * @note       this is only controling 80 w power, mainly limit motor current set.
  *             if power limit is 40w, reduce the value JUDGE_TOTAL_CURRENT_LIMIT 
  *             and POWER_CURRENT_LIMIT, and chassis max speed (include max_vx_speed, min_vx_speed)
  *             只控制80w功率，主要通过控制电机电流设定值,如果限制功率是40w，减少
  *             JUDGE_TOTAL_CURRENT_LIMIT和POWER_CURRENT_LIMIT的值，还有底盘最大速度
  *             (包括max_vx_speed, min_vx_speed)
  * @history
  *  Version    Date            Author          Modification
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
	
#ifndef REFEREE_CONTROL_H
#define REFEREE_CONTROL_H


#include "chassis_task.h"
#include "shoot_task.h"


#include "main.h"

/**
  * @brief          限制功率，主要限制电机电流
  * @param[in]      chassis_power_control: 底盘数据
  * @retval         none
  */
extern void chassis_power_control(chassis_move_t *chassis_power_control);

/**
  * @brief          限制17mm发射机构射速和射频，主要限制电机电流
  * @param[in]      shoot_heat0_speed_and_cooling_control: 发送机构数据
  * @retval         none
  */
extern void shoot_heat0_speed_and_cooling_control(shoot_control_t *shoot_heat0_speed_and_cooling_control);

//通过读取裁判数据,直接修改射速和射频等级
//射速等级  摩擦电机
extern fp32 shoot_fric_grade[4];

//射频等级 拨弹电机
extern fp32 shoot_grigger_grade[6];




#endif
