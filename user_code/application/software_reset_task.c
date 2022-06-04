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

//软件重启 用于紧急情况重启单片机

#include "software_reset_task.h"
//#include "Communicate.h"


uint16_t software_reset_key_delay_time = 0;


/**
 * @brief          communucate_task
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
void software_reset_task(void *pvParameters)
{
  //等待整个系统正常启动
  vTaskDelay(SOFTWARE_REST_TASK_INIT_TIME);

  while (1)
  {
    //软件复位,让单片机重启  同时按下Z X CTRL 一秒
    // if (if_key_pessed(remote_control.rc_ctrl.key.v, 'Z')
    // && if_key_pessed(remote_control.rc_ctrl.key.v, 'X')
    // && if_key_pessed(remote_control.rc_ctrl.key.v, '$'))
    // {
    //     software_reset_key_delay_time++;
    // }

    if (software_reset_key_delay_time >= 500)
    {
      NVIC_SystemReset();
      software_reset_key_delay_time = 0;
    }
    vTaskDelay(SOFTWARE_RESET_CONTROL_TIME);
  }
}


