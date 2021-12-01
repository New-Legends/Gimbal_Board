//
// Created by summerpray on 2021/11/3.
//

#ifndef GIMBAL_BOARD_CAN_H
#define GIMBAL_BOARD_CAN_H

#include "struct_typedef.h"
#include "Motor.h"

#define GIMBAL_CAN hcan1
#define SHOOT_CAN hcan1
#define CHASSIS_CAN hcan2

typedef enum
{
    //发射机构接收ID
    CAN_LEFT_FRIC_MOTOR_ID = 0x205,
    CAN_RIGHT_FRIC_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_SHOOT_ALL_ID = 0x1FF,
    //云台电机接收ID
    CAN_YAW_MOTOR_ID = 0x209,
    CAN_PIT_MOTOR_ID = 0x20A,
    CAN_GIMBAL_ALL_ID = 0x2FF,
} can_msg_id_e;

class CAN_Gimbal
{
public:
    /*
电机数据, 
0:左摩擦轮电机 3508电机, 1右摩擦轮电机 3508电机, 2拨弹电机 2006电机,
3:yaw云台电机 6020电机; 4:pitch云台电机 6020电机;
*/

    motor_measure motor[5];

    CAN_TxHeaderTypeDef gimbal_tx_message;

    CAN_TxHeaderTypeDef shoot_tx_message;

    uint8_t gimbal_can_send_data[8];

    uint8_t shoot_can_send_data[8];

    void init();

    void cmd_gimbal(int16_t yaw, int16_t pitch, int16_t motor3, int16_t motor4);

    void cmd_shoot(int16_t left_fric, int16_t right_fric, int16_t trigger, int16_t rev);

    //void CAN_cmd_gimbal_temp(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

    const motor_measure *get_trigger_motor_measure_point(void);

    const motor_measure *get_shoot_motor_measure_point(uint8_t i);

    const motor_measure *get_gimbal_motor_measure_point(uint8_t i);
};

extern CAN_Gimbal Can;

#endif //GIMBAL_BOARD_CAN_H
