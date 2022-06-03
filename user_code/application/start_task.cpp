#include "start_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "bsp_delay.h"

#include "communicate_task.h"
#include "my_test_task.h"
#include  "chassis_task.H"

#define Tiny_Stack_Size       64
#define Small_Stack_Size      128
#define Normal_Stack_Size     256
#define Large_Stack_Size      512
#define Huge_Stack_Size       1024
#define PriorityVeryLow       1
#define PriorityLow           2
#define PriorityBelowNormal   3
#define PriorityNormal        4
#define PriorityAboveNormal   5
#define PriorityHigh          6
#define PrioritySuperHigh     7
#define PriorityRealtime      8

TaskHandle_t my_test_task_handle;
TaskHandle_t communicate_task_handle;
TaskHandle_t chassis_task_handle;


/**
* @brief Load and start User Tasks.
* @note  Edit this function to add tasks into the activated tasks list.
*/
void System_Resource_Init(void)
{
    /* Syetem Service init --------------*/
    delay_init();

    /* Applications Init ----------------*/
}

    /**
* @brief Load and start User Tasks.
* @note  Edit this function to add tasks into the activated tasks list.
*/
void Task_start(void)
{
        /* Syetem Service init --------------*/
        /* Applications Init ----------------*/
        xTaskCreate(chassis_task, "chassis_task", Large_Stack_Size, NULL, PriorityHigh, &chassis_task_handle);

        xTaskCreate(communicate_task, "communicate_task", Large_Stack_Size, NULL, PriorityHigh, &communicate_task_handle);

        xTaskCreate(my_test_task, "my_test_task", Small_Stack_Size, NULL, PriorityHigh, &my_test_task_handle);
}