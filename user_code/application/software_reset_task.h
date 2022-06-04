#ifndef SOFTWARE_RESET_TASK_H
#define SOFTWARE_RESET_TASK_H

#include "struct_typedef.h"
#include "main.h"
//任务初始化 空闲一段时间
#define SOFTWARE_REST_TASK_INIT_TIME 5000
#define SOFTWARE_RESET_CONTROL_TIME 2

extern void software_reset_task(void *pvParameters);

#endif
