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

#include "software_reset_task.h"

#include "cmsis_os.h"
#include "main.h"
#include "arm_math.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"

uint8_t software_reset_key_delay_time = 0;

/**
  * @brief          软件复位任务，间隔 SOFTWARE_RESET_CONTROL_TIME 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void software_reset_task(void const *pvParameters)
{
    //等待整个系统正常启动
    vTaskDelay(SOFTWARE_REST_TASK_INIT_TIME);

    //软件复位,让单片机重启  同时按下Z X CTRL 一秒
    if(IF_KEY_PRESSED_Z && IF_KEY_PRESSED_X && IF_KEY_PRESSED_CTRL)
    {   
       software_reset_key_delay_time++;
    }

    if(software_reset_key_delay_time >= 500)
    { 
        NVIC_SystemReset();
        software_reset_key_delay_time = 0;
    }

    vTaskDelay(SOFTWARE_RESET_CONTROL_TIME);
}

