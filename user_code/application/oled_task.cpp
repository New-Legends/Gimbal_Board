/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       oled_task.c/h
  * @brief      OLED show error value.oled∆¡ƒªœ‘ æ¥ÌŒÛ¬Î
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "oled_task.h"
#include "main.h"
#include "Oled.h"

#include "cmsis_os.h"
#include "detect_task.h"

#include <stdio.h>
#include <stdarg.h>
#include <string>


#define OLED_CONTROL_TIME 10
#define REFRESH_RATE    10

const error_t *error_list_local;

uint8_t other_toe_name[4][5] = {"GYR\0","ACC\0","MAG\0","REF\0"};

uint8_t Dbus_name[6] = "DBUS\0";

uint8_t last_oled_error = 0;
uint8_t now_oled_errror = 0;
static uint8_t refresh_tick = 0;


/**
  * @brief          oled task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          oled»ŒŒÒ
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void oled_task(void *pvParameters)
{
    uint8_t i;
    uint8_t show_col, show_row;
    error_list_local = get_error_list_point();
    osDelay(1000);
    OLED.init();
    while (1){
        OLED.run();
        osDelay(OLED_CONTROL_TIME);
    }
}

