//
// Created by summerpray on 2021/11/3.
//
#include "Can_receive.h"
#include "cmsis_os.h"
#include "main.h"
#ifdef  __cplusplus
extern "C" {
#endif

#include "bsp_can.h"
#include "can.h"

#ifdef  __cplusplus
}
#endif

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

CAN_Gimbal Can;

void CAN_Gimbal::init()
{
    can_filter_init();
}

const motor_measure *CAN_Gimbal::get_gimbal_motor_measure_point(uint8_t i)
{
    return &motor_gimbal[(i & 0x01)];
}

void CAN_Gimbal::CAN_cmd_gimbal(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = motor1 >> 8;
    gimbal_can_send_data[1] = motor1;
    gimbal_can_send_data[2] = motor2 >> 8;
    gimbal_can_send_data[3] = motor2;
    gimbal_can_send_data[4] = motor3 >> 8;
    gimbal_can_send_data[5] = motor3;
    gimbal_can_send_data[6] = motor4 >> 8;
    gimbal_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

// void CAN_Gimbal::CAN_cmd_gimbal_temp(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
// {
//     uint32_t send_mail_box;
//     gimbal_tx_message.StdId = CAN_GIMBAL_YAW_ID;
//     gimbal_tx_message.IDE = CAN_ID_STD;
//     gimbal_tx_message.RTR = CAN_RTR_DATA;
//     gimbal_tx_message.DLC = 0x08;
//     gimbal_can_send_data[0] = motor1 >> 8;
//     gimbal_can_send_data[1] = motor1;
//     gimbal_can_send_data[2] = motor2 >> 8;
//     gimbal_can_send_data[3] = motor2;
//     gimbal_can_send_data[4] = motor3 >> 8;
//     gimbal_can_send_data[5] = motor3;
//     gimbal_can_send_data[6] = motor4 >> 8;
//     gimbal_can_send_data[7] = motor4;

//     HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
// }

void CAN_Gimbal::CAN_cmd_gimbal_reset_ID()
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = 0x700;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = 0;
    gimbal_can_send_data[1] = 0;
    gimbal_can_send_data[2] = 0;
    gimbal_can_send_data[3] = 0;
    gimbal_can_send_data[4] = 0;
    gimbal_can_send_data[5] = 0;
    gimbal_can_send_data[6] = 0;
    gimbal_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}


