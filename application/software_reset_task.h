/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       software_reset_task.c/h
  * @brief      重启单片机功能。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     May-5-2021     fzj              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

//软件复位 用于紧急情况重启单片机

#ifndef SOFTWARE_REST_TASK_H
#define SOFTWARE_REST_TASK_H

#define SOFTWARE_RESET_CONTROL_TIME 2
#define SOFTWARE_REST_TASK_INIT_TIME 5000


extern void software_reset_task(void const *pvParameters);





#endif