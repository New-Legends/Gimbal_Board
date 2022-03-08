/***********************************************************/
/*板间通讯的相关代码*/
/*
 *      ┌─┐       ┌─┐
 *   ┌──┘ ┴───────┘ ┴──┐
 *   │                 │
 *   │       ───       │
 *   │  ─┬┘       └┬─  │
 *   │                 │
 *   │       ─┴─       │
 *   │                 │
 *   └───┐         ┌───┘
 *       │         │
 *       │         │
 *       │         │
 *       │         └──────────────┐
 *       │                        │
 *       │                        ├─┐
 *       │                        ┌─┘
 *       │                        │
 *       └─┐  ┐  ┌───────┬──┐  ┌──┘
 *         │ ─┤ ─┤       │ ─┤ ─┤
 *         └──┴──┘       └──┴──┘
 * */


#include "communicate.h"
#include "shoot_task.h"
#include "main.h"
#include "vision.h"
#include "cmsis_os.h"

#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "referee_control.h"

gimbal_send_t gimbal_send;



void communicate_task(void *pvParameters)
{
	



	
	gimbal_send.send_RC = get_remote_control_point();
	while (1)
    {
    gimbal_send.ch_0 = gimbal_send.send_RC -> rc.ch[0];
    gimbal_send.ch_1 = gimbal_send.send_RC -> rc.ch[1];
    gimbal_send.s_0 = gimbal_send.send_RC -> rc.s[0];
    gimbal_send.s_1 = gimbal_send.send_RC -> rc.s[1];
    CAN_rc_data_board(100,200,3,4);
			
			
			
			
			
			
			
			
			

     // vTaskDelay(2);
    }
	


}