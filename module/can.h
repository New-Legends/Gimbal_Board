//
// Created by summerpray on 2021/11/3.
//

#ifndef GIMBAL_BOARD_CAN_H
#define GIMBAL_BOARD_CAN_H

#include "struct_typedef.h"
#include "main.h"
#include "motor_measure.h"

#define GIMBAL_CAN hcan1
#define CHASSIS_CAN hcan2

typedef enum {
    CAN_YAW_MOTOR_ID = 0x209,
    //CAN_GIMBAL_YAW_ID = 0x1FF,
    CAN_PIT_MOTOR_ID = 0x20A,
    CAN_GIMBAL_ALL_ID = 0x2FF,
} can_msg_id_e;

class CAN_Gimbal {
public:
    CAN_TxHeaderTypeDef gimbal_tx_message;
    uint8_t gimbal_can_send_data[8];
    void CAN_cmd_gimbal(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
    void CAN_cmd_gimbal_temp(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
    void CAN_cmd_gimbal_reset_ID();
    motor_measure motor_gimbal[2];
    const motor_measure *get_gimbal_motor_measure_point(uint8_t i);

};

#endif //GIMBAL_BOARD_CAN_H
