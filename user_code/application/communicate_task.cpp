#include "communicate_task.h"

#include "Communicate.h"

uint8_t communicate_flag = 0;

/**
* @brief          communucat_task
* @param[in]      pvParameters: NULL
* @retval         none
*/
void communicate_task(void *pvParameters)
{
  vTaskDelay(COMMUNICATE_TASK_INIT_TIME);

  communicate.init();

  while (1)
  {
    communicate_flag = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
    
    communicate.run();

    // //系统延时
    vTaskDelay(COMMUNICATE_CONTROL_TIME_MS);
  }
}






