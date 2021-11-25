//
// Created by summerpray on 2021/11/3.
//

#ifndef GIMBAL_BOARD_CAN_H
#define GIMBAL_BOARD_CAN_H

#include "struct_typedef.h"
#include "main.h"
#include "motor_measure.h"

typedef enum {
    CAN_GIMBAL_ALL_ID = 0x1FF,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_6020_M3_ID = 0x203,
    CAN_6020_M4_ID = 0x204,
} can_msg_id_e;

class CAN_Gimbal {
public:
    CAN_TxHeaderTypeDef gimbal_tx_message;
    uint8_t gimbal_can_send_data[8];
    void CAN_cmd_gimbal(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
    void CAN_cmd_gimbal_reset_ID();
    motor_measure motor_gimbal[4];
    const motor_measure *get_gimbal_motor_measure_point(uint8_t i);

};

#endif //GIMBAL_BOARD_CAN_H
