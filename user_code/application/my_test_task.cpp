#include "my_test_task.h"

#include "pid.h"
#include "bsp_led.h"

uint8_t test_flag = 0;

/**
  * @brief          test_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void my_test_task(void *pvParameters)
{
  //空闲一段时间
  vTaskDelay(TEST_TASK_INIT_TIME);
  while (1)
  {
    test_flag = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);

    //系统延时
    vTaskDelay(TEST_CONTROL_TIME_MS);
  }
    
}