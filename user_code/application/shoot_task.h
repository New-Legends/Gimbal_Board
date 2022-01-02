/**
  ****************************(C) COPYRIGHT 2021 SUMMERPRAY****************************
  * @file       shoot.cpp/h
  * @brief      �������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     NOV-30-2021     summerpray      1. doing
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2021 SUMMERPRAY****************************
  */
#ifndef GIMBAL_BOARD_SHOOT_H
#define GIMBAL_BOARD_SHOOT_H

#include "cmsis_os.h"
#include "System_Config.h"
#include "user_lib.h"
#include "shoot.h"

//�����ʼ�� ����һ��ʱ��
#define SHOOT_TASK_INIT_TIME 201 
#define SHOOT_CONTROL_TIME 1

//�����������޵������
#define SHOOT_FRIC_MOTOR_NO_CURRENT 1
#define SHOOT_TRIGGER_MOTOR_NO_CURRENT 1

extern void shoot_task(void *pvParameters);


#endif
