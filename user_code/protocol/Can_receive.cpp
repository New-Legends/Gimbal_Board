//
// Created by summerpray on 2021/11/3.
//
#include "Can_receive.h"
#include "cmsis_os.h"
#include "main.h"
#include "detect_task.h"
#ifdef __cplusplus
extern "C"
{
#endif

#include "bsp_can.h"
#include "can.h"

#ifdef __cplusplus
}
#endif

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


void Can_receive::init()
{
    can_filter_init();
}
 
void Can_receive::get_motor_measure(uint8_t num, uint8_t data[8])
{
    motor[num].last_ecd = motor[num].ecd;
    motor[num].ecd = (uint16_t)(data[0] << 8 | data[1]);
    motor[num].speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
    motor[num].given_current = (uint16_t)(data[4] << 8 | data[5]);
    motor[num].temperate = data[6];
}

const motor_measure *Can_receive::get_gimbal_motor_measure_point(uint8_t i)
{
    return &motor[i];
}

/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure *Can_receive::get_trigger_motor_measure_point(void)
{
    return &motor[2];
}

const motor_measure *Can_receive::get_shoot_motor_measure_point(uint8_t i)
{
    return &motor[i];
}


void Can_receive::cmd_gimbal(int16_t yaw, int16_t pitch, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = yaw >> 8;
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = pitch >> 8;
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = motor3 >> 8;
    gimbal_can_send_data[5] = motor3;
    gimbal_can_send_data[6] = motor4 >> 8;
    gimbal_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

void Can_receive::cmd_shoot(int16_t left_fric, int16_t right_fric, int16_t trigger, int16_t rev)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_SHOOT_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (left_fric >> 8);
    gimbal_can_send_data[1] = left_fric;
    gimbal_can_send_data[2] = (right_fric >> 8);
    gimbal_can_send_data[3] = right_fric;
    gimbal_can_send_data[4] = (trigger >> 8);
    gimbal_can_send_data[5] = trigger;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;

    HAL_CAN_AddTxMessage(&SHOOT_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}



void Can_receive::receive_cooling_and_id_board_com(uint8_t data[8])
{
    gimbal_receive.id1_17mm_cooling_limit = (uint16_t)(data[0] << 8 | data[1]);
    gimbal_receive.id1_17mm_cooling_rate = (uint16_t)(data[2] << 8 | data[3]);
    gimbal_receive.id1_17mm_cooling_heat = (uint16_t)(data[4] << 8 | data[5]);
    gimbal_receive.color = (data[6]);
    gimbal_receive.robot_id = (data[7]);
}

void Can_receive::receive_17mm_speed_and_mode_board_com(uint8_t data[8])
{
    gimbal_receive.id1_17mm_speed_limi = (uint16_t)(data[0] << 8 | data[1]);
    gimbal_receive.bullet_speed = (uint16_t)(data[2] << 8 | data[3]);
    gimbal_receive.chassis_behaviour = data[4];
}



void Can_receive::send_rc_board_com(int16_t ch_1, int16_t ch_2, int16_t ch_3, uint16_t v)
{
    //数据填充
    gimbal_send.ch_1 = ch_1;
    gimbal_send.ch_2 = ch_2;
    gimbal_send.ch_3 = ch_3;
    gimbal_send.v = v;

    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_RC_BOARM_COM_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = ch_1 >> 8;
    gimbal_can_send_data[1] = ch_1;
    gimbal_can_send_data[2] = ch_2 >> 8;
    gimbal_can_send_data[3] = ch_2;
    gimbal_can_send_data[4] = ch_3 >> 8;
    gimbal_can_send_data[5] = ch_3;
    gimbal_can_send_data[6] = v >> 8;
    gimbal_can_send_data[7] = v;

    HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

void Can_receive::send_gimbal_board_com(uint8_t s1, uint8_t gimbal_behaviour, fp32 gimbal_yaw_angle)
{
    //数据填充
    gimbal_send.s1 = s1;
    gimbal_send.gimbal_behaviour = gimbal_behaviour;
    gimbal_send.gimbal_yaw_angle = gimbal_yaw_angle;

    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_BOARD_COM_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = s1;
    gimbal_can_send_data[1] = gimbal_behaviour;
    gimbal_can_send_data[2] = (uint8_t)((int16_t)gimbal_yaw_angle >> 8);
    gimbal_can_send_data[3] = (uint8_t)(gimbal_yaw_angle);
    gimbal_can_send_data[4] = 0;
    gimbal_can_send_data[5] = 0;
    gimbal_can_send_data[6] = 0;
    gimbal_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}
