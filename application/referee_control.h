/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_control.c/h
  * @brief      chassis power control.���̹��ʿ���
  * @note       this is only controling 80 w power, mainly limit motor current set.
  *             if power limit is 40w, reduce the value JUDGE_TOTAL_CURRENT_LIMIT 
  *             and POWER_CURRENT_LIMIT, and chassis max speed (include max_vx_speed, min_vx_speed)
  *             ֻ����80w���ʣ���Ҫͨ�����Ƶ�������趨ֵ,������ƹ�����40w������
  *             JUDGE_TOTAL_CURRENT_LIMIT��POWER_CURRENT_LIMIT��ֵ�����е�������ٶ�
  *             (����max_vx_speed, min_vx_speed)
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
  * @brief          ���ƹ��ʣ���Ҫ���Ƶ������
  * @param[in]      chassis_power_control: ��������
  * @retval         none
  */
extern void chassis_power_control(chassis_move_t *chassis_power_control);

/**
  * @brief          ����17mm����������ٺ���Ƶ����Ҫ���Ƶ������
  * @param[in]      shoot_heat0_speed_and_cooling_control: ���ͻ�������
  * @retval         none
  */
extern void shoot_heat0_speed_and_cooling_control(shoot_control_t *shoot_heat0_speed_and_cooling_control);

//ͨ����ȡ��������,ֱ���޸����ٺ���Ƶ�ȼ�
//���ٵȼ�  Ħ�����
extern fp32 shoot_fric_grade[4];

//��Ƶ�ȼ� �������
extern fp32 shoot_grigger_grade[6];




#endif
