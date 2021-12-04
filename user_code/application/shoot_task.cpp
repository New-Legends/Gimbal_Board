/**
  ****************************(C) COPYRIGHT 2021 SUMMERPRAY****************************
  * @file       shoot.cpp/h
  * @brief      射击功能.
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
  * @brief          射击任务，初始化PID，遥控器指针，电机指针
  * @param[in]      void
  * @retval         返回空
  */
void shoot_task(void *pvParameters){
    //初始化延时
    vTaskDelay(SHOOT_TASK_INIT_TIME); 
    Shoot.init();
    while (1)
    {
        Shoot.set_mode();           //设置状态机
        Shoot.feedback_update();    //更新数据
        Shoot.set_control();        //射击任务控制循环

        //CAN发送
        //Can.cmd_shoot(Shoot.fric_motor[LEFT].give_current, Shoot.fric_motor[RIGHT].give_current, Shoot.given_current, 0);
        //Can.cmd_shoot(0, 0, Shoot.given_current, 0);
        vTaskDelay(SHOOT_CONTROL_TIME);
    }
    
}


