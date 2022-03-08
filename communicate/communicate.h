#ifndef COMMUNICATE_H
#define COMMUNICATE_H

#include "remote_control.h"
#include "CAN_receive.h"
#include "main.h"
#include "struct_typedef.h"
#include "user_lib.h"


typedef struct 
{
	
   const RC_ctrl_t *send_RC;
   int16_t ch_0;
   int16_t ch_1;
   char s_0;
   char s_1;
}gimbal_send_t;



#endif COMMUNICATE_H
