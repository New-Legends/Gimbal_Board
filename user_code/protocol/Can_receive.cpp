#include "can_receive.h"

#include "cmsis_os.h"
#include "main.h"

#include "bsp_can.h"
#include "bsp_delay.h"
#include "can.h"

#include "struct_typedef.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void Can_receive::init()
{
    can_filter_init();
}
 
void Can_receive::get_motive_motor_measure(uint8_t num, uint8_t data[8])
{
    chassis_motive_motor[num].last_ecd = chassis_motive_motor[num].ecd;
    chassis_motive_motor[num].ecd = (uint16_t)(data[0] << 8 | data[1]);
    chassis_motive_motor[num].speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
    chassis_motive_motor[num].given_current = (uint16_t)(data[4] << 8 | data[5]);
    chassis_motive_motor[num].temperate = data[6];
}

/**
* @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
* @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
* @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
* @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
* @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
* @retval         none
*/
void Can_receive::can_cmd_chassis_motive_motor(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_MOTIVE_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);

}

/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void Can_receive::can_cmd_chassis_motive_motor_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          返回底盘动力电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *Can_receive::get_chassis_motive_motor_measure_point(uint8_t i)
{
    return &chassis_motive_motor[i];
}

void Can_receive::get_rudder_motor_measure(uint8_t num, uint8_t data[8])
{
    chassis_rudder_motor[num].last_ecd = chassis_rudder_motor[num].ecd;
    chassis_rudder_motor[num].ecd = (uint16_t)(data[0] << 8 | data[1]);
    chassis_rudder_motor[num].speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
    chassis_rudder_motor[num].given_current = (uint16_t)(data[4] << 8 | data[5]);
    chassis_rudder_motor[num].temperate = data[6];
}

/**
* @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
* @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
* @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
* @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
* @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
* @retval         none
*/
void Can_receive::can_cmd_chassis_rudder_motor(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_RUDDER_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          返回底盘舵向电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *Can_receive::get_chassis_rudder_motor_measure_point(uint8_t i)
{
    return &chassis_rudder_motor[i];
}

void Can_receive::receive_rc_board_com(uint8_t data[8])
{
    chassis_receive.ch_0 = (int16_t)(data[0] << 8 | data[1]);
    chassis_receive.ch_2 = (int16_t)(data[2] << 8 | data[3]);
    chassis_receive.ch_3 = (int16_t)(data[4] << 8 | data[5]);
    chassis_receive.v = (uint16_t)(data[6] << 8 | data[7]);
}

void Can_receive::receive_gimbal_board_com(uint8_t data[8])
{
    chassis_receive.s0 = data[0];
    chassis_receive.gimbal_behaviour = data[1];
    chassis_receive.gimbal_yaw_angle = (fp32)(int32_t)(data[2] << 24 | data[3] << 16 | data[4] << 8 | data[5]) / 1000;
}

void Can_receive::send_cooling_and_id_board_com(uint16_t id1_17mm_cooling_limit, uint16_t id1_17mm_cooling_rate, uint16_t id1_17mm_cooling_heat, uint8_t color, uint8_t robot_id)
{
    //数据填充
    chassis_send.id1_17mm_cooling_limit = id1_17mm_cooling_limit;
    chassis_send.id1_17mm_cooling_rate = id1_17mm_cooling_rate;
    chassis_send.id1_17mm_cooling_heat = id1_17mm_cooling_heat;
    chassis_send.color = color;
    chassis_send.robot_id = robot_id;


    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_COOLING_BOARM_COM_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = id1_17mm_cooling_limit >> 8;
    chassis_can_send_data[1] = id1_17mm_cooling_limit;
    chassis_can_send_data[2] = id1_17mm_cooling_rate >> 8;
    chassis_can_send_data[3] = id1_17mm_cooling_rate;
    chassis_can_send_data[4] = id1_17mm_cooling_heat >> 8;
    chassis_can_send_data[5] = id1_17mm_cooling_heat;
    chassis_can_send_data[6] = color;
    chassis_can_send_data[7] = robot_id;

    HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void Can_receive::send_17mm_speed_and_mode_board_com(uint16_t id1_17mm_speed_limit, uint16_t bullet_speed, uint8_t chassis_behaviour)
{
    //数据填充
    chassis_send.id1_17mm_speed_limi = id1_17mm_speed_limit;
    chassis_send.bullet_speed = bullet_speed;
    chassis_send.chassis_behaviour = chassis_behaviour;

    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_17MM_SPEED_BOARD_COM_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = id1_17mm_speed_limit >> 8;
    chassis_can_send_data[1] = id1_17mm_speed_limit;
    chassis_can_send_data[2] = bullet_speed >> 8;
    chassis_can_send_data[3] = bullet_speed;
    chassis_can_send_data[4] = chassis_behaviour;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void Can_receive::get_super_cap_data(uint8_t data[8])
{
    cap_receive.input_vot     = ((float) ((int16_t) (data[1] << 8 | data[0])) / 100.0f);  //输入电压
    cap_receive.cap_vot       = ((float) ((int16_t) (data[3] << 8 | data[2])) / 100.0f);  //电容电压
    cap_receive.input_current = ((float) ((int16_t) (data[5] << 8 | data[4])) / 100.0f);  //输入电流
    cap_receive.target_power  = ((float) ((int16_t) (data[7] << 8 | data[6])) / 100.0f);  //输入功率
}

  void Can_receive::can_cmd_super_cap_power(uint16_t set_power)
  {
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x210;
    chassis_tx_message.IDE = CAN_ID_STD; //0x0000
    chassis_tx_message.RTR = CAN_RTR_DATA; //0x0000
    chassis_tx_message.DLC = 0x08;

    chassis_can_send_data[0] = (set_power >> 8);
    chassis_can_send_data[1] = (set_power);
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;    
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}