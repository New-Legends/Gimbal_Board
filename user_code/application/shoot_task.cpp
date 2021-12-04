/**
  ****************************(C) COPYRIGHT 2021 SUMMERPRAY****************************
  * @file       shoot.cpp/h
  * @brief      �������.
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

#include "shoot_task.h"
 /**
  * @brief          ������񣬳�ʼ��PID��ң����ָ�룬���ָ��
  * @param[in]      void
  * @retval         ���ؿ�
  */
void shoot_task(void *pvParameters){
    //��ʼ����ʱ
    vTaskDelay(SHOOT_TASK_INIT_TIME); 
    Shoot.init();
    while (1)
    {
        Shoot.set_mode();           //����״̬��
        Shoot.feedback_update();    //��������
        Shoot.set_control();        //����������ѭ��

        //CAN����
        //Can.cmd_shoot(Shoot.fric_motor[LEFT].give_current, Shoot.fric_motor[RIGHT].give_current, Shoot.given_current, 0);
        //Can.cmd_shoot(0, 0, Shoot.given_current, 0);
        vTaskDelay(SHOOT_CONTROL_TIME);
    }
    
}


