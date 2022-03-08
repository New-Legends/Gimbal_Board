/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"

#include "cmsis_os.h"

#include "main.h"
#include "bsp_rng.h"


#include "detect_task.h"
#include "communicate.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
    


/*
电机数据, 
0:底盘电机1 3508电机,  ;
1拨弹电机 2006电机 2:左摩擦轮电机 3508电机, 3右摩擦轮电机 3508电机,
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机;
*/
static motor_measure_t motor_chassis[6];

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_shoot_tx_message;
static uint8_t              chassis_shoot_can_send_data[8]; 
static CAN_TxHeaderTypeDef  board_communicat_tx_message;
static uint8_t              board_communicat_can_send_data[8];
static CAN_TxHeaderTypeDef  send_chassis_board_message;
static uint8_t              send_chassis_board_send_data[8];




/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case CAN_LEFT_FRIC_MOTOR_ID:
        case CAN_RIGHT_FRIC_MOTOR_ID:
        case CAN_TRIGGER_MOTOR_ID:
        case CAN_CHASSIS_MOTOR_ID:
        case CAN_YAW_MOTOR_ID:
        case CAN_PIT_MOTOR_ID:
        case CAN_BORAD1_COMMUNICAT_ID:
        {
            if(rx_header.StdId == CAN_BORAD1_COMMUNICAT_ID)  //板间通信
            {                                  
            
              detect_hook(BORAD_COMMUNICAT_TOE);

            }
            else
            {
              static uint8_t i = 0;
              //get motor id
              i = rx_header.StdId - CAN_TRIGGER_MOTOR_ID;
              get_motor_measure(&motor_chassis[i], rx_data);
              detect_hook(CHASSIS_MOTOR_TOE + i);
            }
              break;
        }

        default:
        {
            break;
        }
}


}


 /**
  * @brief          发送电机控制电流(0x209,0x20A,0x20B,0x20C)
  * @param[in]      yaw: (0x209) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x20A) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      rev1: (0x20B)  保留，电机控制电流
  * @param[in]      rev2: (0x20C) 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t rev1, int16_t rev2)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (rev1 >> 8);
    gimbal_can_send_data[5] = rev1;
    gimbal_can_send_data[6] = (rev2 >> 8);
    gimbal_can_send_data[7] = rev2;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}








/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      trigger: (0x201) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      left_fric: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      right_fric: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      chassis: (0x204) 3508电机控制电路，范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis_shoot(int16_t trigger, int16_t left_fric, int16_t right_fric, int16_t chassis)
{
    uint32_t send_mail_box;
    chassis_shoot_tx_message.StdId = CAN_CHASSIS_SHOOT_ALL_ID;
    chassis_shoot_tx_message.IDE = CAN_ID_STD;
    chassis_shoot_tx_message.RTR = CAN_RTR_DATA;
    chassis_shoot_tx_message.DLC = 0x08;
    chassis_shoot_can_send_data[0] = (trigger >> 8);
    chassis_shoot_can_send_data[1] = trigger;
    chassis_shoot_can_send_data[2] = (left_fric >> 8);
    chassis_shoot_can_send_data[3] = left_fric;
    chassis_shoot_can_send_data[4] = (right_fric >> 8);
    chassis_shoot_can_send_data[5] = right_fric;
    chassis_shoot_can_send_data[6] = (chassis >> 8);
    chassis_shoot_can_send_data[7] = chassis;
    HAL_CAN_AddTxMessage(&CHASSIS_SHOOT_CAN, &chassis_shoot_tx_message, chassis_shoot_can_send_data, &send_mail_box);

}

/**
 * @brief             板件通讯
 * 
 * @param data1 
 * @param data2 
 * @param data3 
 * @param data4 
 */
void CAN_rc_data_board(int16_t data1,int16_t data2,char data3,char data4)
{
    uint32_t send_mail_box;
    send_chassis_board_message.StdId = CAN_RC_BOARM_ID;
    send_chassis_board_message.IDE = CAN_ID_STD;
    send_chassis_board_message.RTR = CAN_RTR_DATA;
    send_chassis_board_message.DLC = 0x08;
    send_chassis_board_send_data[0] = (data1 >> 8);
    send_chassis_board_send_data[1] = data1;
    send_chassis_board_send_data[2] = (data2 >> 8);
    send_chassis_board_send_data[3] = data2;
    send_chassis_board_send_data[4] = data3 ;
    send_chassis_board_send_data[5] = data3;
    send_chassis_board_send_data[6] = data4;
    send_chassis_board_send_data[7] = data4;
    HAL_CAN_AddTxMessage(&SEND_BOARD, &send_chassis_board_message, send_chassis_board_send_data, &send_mail_box);
}

///**
//  * @brief          板间通信发送函数
//  * @param[in]      0x220 
//  * @retval         none
//  */
//void CAN_cmd_board_communicat(int16_t temPower)
//{	
//   uint32_t send_mail_box;
//    super_cap_tx_message.StdId = CAN_SUPER_CAP_ALL_ID;
//    super_cap_tx_message.IDE = CAN_ID_STD;
//    super_cap_tx_message.RTR = CAN_RTR_DATA;
//    super_cap_tx_message.DLC = 0x08;
//    super_cap_can_send_data[0] = (temPower >> 8);
//    super_cap_can_send_data[1] = temPower;
//    super_cap_can_send_data[2] = 0;
//    super_cap_can_send_data[3] = 0;
//    super_cap_can_send_data[4] = 0;
//    super_cap_can_send_data[5] = 0;
//    super_cap_can_send_data[6] = 0;
//    super_cap_can_send_data[7] = 0;

//    HAL_CAN_AddTxMessage(&SUPER_CAP_CAN, &super_cap_tx_message, super_cap_can_send_data, &send_mail_box);

//}

/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}


/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}


/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[0];
}


/**
  * @brief          返回摩擦轮 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,1]
  * @retval         电机数据指针
  */
const motor_measure_t *get_fric_motor_measure_point(uint8_t i)
{
    return &motor_chassis[i];
}


/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(void)
{
    return &motor_chassis[3];
}
