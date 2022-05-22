#include "Can_receive.h"

#include "cmsis_os.h"
#include "main.h"

#include "bsp_can.h"
#include "can.h"

#include "struct_typedef.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

Can_receive can_receive;

int field_event_outpost;

void Can_receive::init()
{
    can_filter_init();
}

void Can_receive::get_gimbal_motor_measure(uint8_t num, uint8_t data[8])
{
    gimbal_motor[num].last_ecd = gimbal_motor[num].ecd;
    gimbal_motor[num].ecd = (uint16_t)(data[0] << 8 | data[1]);
    gimbal_motor[num].speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
    gimbal_motor[num].given_current = (uint16_t)(data[4] << 8 | data[5]);
    gimbal_motor[num].temperate = data[6];
}

/**
* @brief          
* @param[in]      
* @param[in]          
* @retval         none
*/
void Can_receive::can_cmd_gimbal_motor(int16_t yaw, int16_t pitch, int16_t empty1, int16_t empty2)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = yaw >> 8;
    can_send_data[1] = yaw;
    can_send_data[2] = pitch >> 8;
    can_send_data[3] = pitch;
    can_send_data[4] = empty1 >> 8;
    can_send_data[5] = empty1;
    can_send_data[6] = empty2 >> 8;
    can_send_data[7] = empty2;

    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

/**
  * @brief          返回云台电机 6020电机数据指针
  * @param[in]      i: 电机编号,范围[0,1]
  * @retval         电机数据指针
  */
const motor_measure_t *Can_receive::get_gimbal_motor_measure_point(uint8_t i)
{
    return &gimbal_motor[i];
}

void Can_receive::get_shoot_motor_measure(uint8_t num, uint8_t data[8])
{
    shoot_motor[num].last_ecd = shoot_motor[num].ecd;
    shoot_motor[num].ecd = (uint16_t)(data[0] << 8 | data[1]);
    shoot_motor[num].speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
    shoot_motor[num].given_current = (uint16_t)(data[4] << 8 | data[5]);
    shoot_motor[num].temperate = data[6];
}

/**
* @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
* @param[in]      left_fric: (0x201) 3508电机控制电流, 范围 [-16384,16384]
* @param[in]      right_fric: (0x202) 3508电机控制电流, 范围 [-16384,16384]
* @param[in]      tigger: (0x203) 3508电机控制电流, 范围 [-16384,16384]
* @retval         none
*/
void Can_receive::can_cmd_shoot_motor_motor(int16_t left_fric, int16_t right_fric, int16_t tigger)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_SHOOT_ALL_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = left_fric >> 8;
    can_send_data[1] = left_fric;
    can_send_data[2] = right_fric >> 8;
    can_send_data[3] = right_fric;
    can_send_data[4] = tigger >> 8;
    can_send_data[5] = tigger;
    can_send_data[6] = 0 >> 8;
    can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&SHOOT_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void Can_receive::can_cmd_shoot_motor_reset_ID(void)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = 0x700;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = 0;
    can_send_data[1] = 0;
    can_send_data[2] = 0;
    can_send_data[3] = 0;
    can_send_data[4] = 0;
    can_send_data[5] = 0;
    can_send_data[6] = 0;
    can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&SHOOT_CAN, &can_tx_message, can_send_data, &send_mail_box);
}


/**
  * @brief          返回云台电机 6020电机数据指针
  * @param[in]      i: 电机编号,范围[0,1]
  * @retval         电机数据指针
  */
const motor_measure_t *Can_receive::get_shoot_motor_measure_point(uint8_t i)
{
    return &shoot_motor[i];
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
    gimbal_receive.id1_17mm_speed_limit = (uint16_t)(data[0] << 8 | data[1]);
    gimbal_receive.bullet_speed = (uint16_t)(data[2] << 8 | data[3]);
    gimbal_receive.chassis_behaviour = data[4];
    gimbal_receive.base_HP = (uint16_t)(data[5] << 8 | data[6]);
}

void Can_receive::send_rc_board_com(int16_t ch_0, int16_t ch_2, int16_t ch_1, uint8_t s0, uint8_t s1)
{
    //数据填充
    gimbal_send.ch_0 = ch_0;
    gimbal_send.ch_2 = ch_2;
    gimbal_send.s0 = s0;
    gimbal_send.ch_1 = ch_1;
    gimbal_send.s1 = s1;

    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_RC_BOARM_COM_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = ch_0 >> 8;
    can_send_data[1] = ch_0;
    can_send_data[2] = ch_2 >> 8;
    can_send_data[3] = ch_2;
    can_send_data[4] = ch_1 >> 8;
    can_send_data[5] = ch_1;
    can_send_data[6] = s0;
    can_send_data[7] = s1;

    HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &can_tx_message, can_send_data, &send_mail_box);
}




void Can_receive::output_state(void){
    if(gimbal_receive.base_HP>0){
        field_event_outpost=1;
    }
    else 
    {
        field_event_outpost=0;
    }
}
