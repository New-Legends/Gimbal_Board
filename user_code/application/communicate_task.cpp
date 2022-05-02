#include "communicate_task.h"

#include "Communicate.h"

uint8_t communicate_flag = 0;

/**
* @brief          communucate_task
* @param[in]      pvParameters: NULL
* @retval         none
*/
void communicate_task(void *pvParameters)
{
  vTaskDelay(COMMUNICATE_TASK_INIT_TIME);

  communicate.init();

  while (1)
  {

    communicate.run();

    communicate_flag = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
    vTaskDelay(COM_CONTROL_TIME_MS);
  }
}
