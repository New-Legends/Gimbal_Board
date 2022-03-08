/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
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

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_SHOOT_CAN hcan2
#define GIMBAL_CAN hcan2
#define SEND_BOARD hcan1


/* CAN send and receive ID */
typedef enum
{
  //���̺ͷ�������������ID
    CAN_CHASSIS_SHOOT_ALL_ID = 0x200,
    CAN_TRIGGER_MOTOR_ID = 0x201,
    CAN_LEFT_FRIC_MOTOR_ID = 0x202,
    CAN_RIGHT_FRIC_MOTOR_ID = 0x203,
    CAN_CHASSIS_MOTOR_ID = 0x204,
    
  //��̨�������ID
    CAN_GIMBAL_ALL_ID = 0x1FF,
    CAN_YAW_MOTOR_ID = 0x205,//��ס6020ʵ�ʱ���Ҫ-4
    CAN_PIT_MOTOR_ID = 0x206,//��ס6020ʵ�ʱ���Ҫ-4

  //���ͨ�ŷ���ID
    CAN_RC_BOARM_ID = 0x301,
    CAN_BORAD1_COMMUNICAT_ID = 0x221,

    
} can_msg_id_e; 

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;


/**
  * @brief          ���͵�����Ƶ���(0x209,0x20A,0x20B,0x20C)
  * @param[in]      yaw: (0x209) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      pitch: (0x20A) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      rev1: (0x20B)  ������������Ƶ���
  * @param[in]      rev2: (0x20C) ������������Ƶ���
  * @retval         none
  */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t rev1, int16_t rev2);


/**
  * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      trigger: (0x201) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      left_fric: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      right_fric: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      chassis: (0x204) 3508������Ƶ�·����Χ [-16384,16384]
  * @retval         none
  */
void CAN_cmd_shoot(int16_t trigger, int16_t left_fric, int16_t right_fric);


/**
  * @brief          ����yaw 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          ����pitch 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);


/**
  * @brief          ���ز������ 2006�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          ����Ħ���ֵ�� 3508�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_fric_motor_measure_point(uint8_t i);


/**
  * @brief          ���ص��̵�� 3508�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_chassis_motor_measure_point();

/**
 * @brief 
 * 
 */
void CAN_rc_data_board(int16_t ,int16_t,char ,char);
#endif
