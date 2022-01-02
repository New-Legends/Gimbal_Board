//
// Created by summerpray on 2021/11/3.
//

#ifndef GIMBAL_BOARD_CAN_H
#define GIMBAL_BOARD_CAN_H

#include "struct_typedef.h"
#include "Motor.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


#define GIMBAL_CAN hcan1
#define SHOOT_CAN hcan1
#define BOARD_COM_CAN hcan1

typedef enum
{
    //发射机构电机接受ID CAN1
    CAN_LEFT_FRIC_MOTOR_ID = 0x201,
    CAN_RIGHT_FRIC_MOTOR_ID = 0x202,
    CAN_TRIGGER_MOTOR_ID = 0x203,
    CAN_COVER_MOTOR_ID = 0X204,
    CAN_SHOOT_ALL_ID = 0x200,

    //云台电机接收ID CAN1
    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PITCH_MOTOR_ID = 0x206,
    CAN_GIMBAL_ALL_ID = 0x1FF,

    //板间通信ID
    CAN_RC_BOARM_COM_ID = 0x301,
    CAN_GIMBAL_BOARD_COM_ID = 0x302,
    CAN_COOLING_BOARM_COM_ID = 0x303,
    CAN_17MM_SPEED_BOARD_COM_ID = 0x304,
} can_msg_id_e;

//云台发送数据结构体
typedef struct
{
    //遥控器数据
    int16_t ch_1;
    int16_t ch_2;
    int16_t ch_3;
    uint16_t v;

    //云台状态
    uint8_t s1;
    uint8_t gimbal_behaviour;
    fp32 gimbal_yaw_angle;
} gimbal_send_t;

//云台接收数据结构体
typedef struct
{
    //测试热量及ID
    uint16_t id1_17mm_cooling_limit; //17mm测速热量上限
    uint16_t id1_17mm_cooling_rate;  //17mm测速热量冷却
    uint16_t id1_17mm_cooling_heat;  //17mm测速实时热量
    uint8_t color;                   //判断红蓝方
    uint8_t robot_id;                //机器人编号

    //测速速度及底盘模式
    uint16_t id1_17mm_speed_limi; //17mm测速射速上限
    uint16_t bullet_speed;        //17mm测速实时射速

    uint8_t chassis_behaviour;

} gimbal_receive_t;

class Can_receive
{
public:
/*
电机数据, 
0:左摩擦轮电机 3508电机, 1:右摩擦轮电机 3508电机, 2:拨弹电机 2006电机,
3:弹舱电机 2006电机,4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机;
*/
    motor_measure motor[6];
    CAN_TxHeaderTypeDef gimbal_tx_message;
    uint8_t gimbal_can_send_data[8];



    //板间通信
    //底盘接收信息
    gimbal_receive_t gimbal_receive;

    gimbal_send_t gimbal_send;

    void init();

    void get_motor_measure(uint8_t num, uint8_t data[8]);

    void cmd_gimbal(int16_t yaw, int16_t pitch, int16_t motor3, int16_t motor4);

    void cmd_shoot(int16_t left_fric, int16_t right_fric, int16_t trigger, int16_t rev);

    //void CAN_cmd_gimbal_temp(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

    const motor_measure *get_trigger_motor_measure_point(void);

    const motor_measure *get_shoot_motor_measure_point(uint8_t i);

    const motor_measure *get_gimbal_motor_measure_point(uint8_t i);


    //板间通信函数
    void receive_cooling_and_id_board_com(uint8_t data[8]);

    void receive_17mm_speed_and_mode_board_com(uint8_t data[8]);

    //发送遥控器数据
    void send_rc_board_com(int16_t ch_1, int16_t ch_2, int16_t ch_3, uint16_t v);
    //发送云台模式及状态
    void send_gimbal_board_com(uint8_t s1, uint8_t gimbal_behaviour, fp32 gimbal_yaw_angle);

};


#endif //GIMBAL_BOARD_CAN_H
