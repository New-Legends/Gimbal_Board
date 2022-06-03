#ifndef MY_TEST_TASK_H
#define MY_TEST_TASK_H

#include "cmsis_os.h"
#include "main.h"

#include "bsp_led.h"
#include "pid.h"



//测试开始空闲一段时间
#define TEST_TASK_INIT_TIME 30

//测试任务控制间隔 2ms
#define TEST_CONTROL_TIME_MS 2

/**
  * @brief          test_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void my_test_task(void *pvParameters);

#endif 