/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      ������ܡ�
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "user_lib.h"

//�����ʼ�� ����һ��ʱ��
#define SHOOT_TASK_INIT_TIME 201 
#define SHOOT_CONTROL_TIME 1

#define TRIGGER_CCW 1 //����˳ʱ��
#define TRIGGER_CW -1 //������ʱ��


#define SHOOT_TRIGGER_DIRECTION TRIGGER_CW

//������俪��ͨ������
#define SHOOT_RC_MODE_CHANNEL       1
//��̨ģʽʹ�õĿ���ͨ��
#define shoot_MODE_CHANNEL 0


//����Ħ���ֵ�б��
#define SHOOT_FRIC_ADD_VALUE    0.1f

//���Ħ���ּ���� �ر�
#define SHOOT_KEYBOARD           KEY_PRESSED_OFFSET_G

//�����ɺ� �ӵ�����ȥ���ж�ʱ�䣬�Է��󴥷�
#define SHOOT_DONE_KEY_OFF_TIME     15
//��곤���ж�
#define PRESS_LONG_TIME             400
//Ħ���ֿ���������ʱ
#define KEY_FRIC_LONG_TIME             200


//ң����������ش��µ�һ��ʱ��� ���������ӵ� �����嵥
#define RC_S_LONG_TIME              2000
//Ħ���ָ��� ���� ʱ��
#define UP_ADD_TIME                 80
//�����������ֵ��Χ
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191


//���̵��rmp �仯�� ��ת�ٶȵı���
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f 
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f * SHOOT_TRIGGER_DIRECTION
#define FULL_COUNT                  18



//�����ٶ�
#define TRIGGER_SPEED               10.0f * SHOOT_TRIGGER_DIRECTION   //10
#define CONTINUE_TRIGGER_SPEED      15.0f * SHOOT_TRIGGER_DIRECTION  //15
#define READY_TRIGGER_SPEED         5.0f * SHOOT_TRIGGER_DIRECTION //5

#define KEY_OFF_JUGUE_TIME          500
#define SWITCH_TRIGGER_ON           0
#define SWITCH_TRIGGER_OFF          1

//����ʱ�� �Լ���תʱ��
#define BLOCK_TRIGGER_SPEED         1.0f
#define BLOCK_TIME                  700
#define REVERSE_TIME                500
#define REVERSE_SPEED_LIMIT         13.0f

#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_TEN                      0.314f


//�����ֵ��PID
#define TRIGGER_ANGLE_PID_KP        1000.0f  //800
#define TRIGGER_ANGLE_PID_KI        2.0f  //0.5
#define TRIGGER_ANGLE_PID_KD        5.0f

#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 200.0f

#define TRIGGER_READY_PID_MAX_OUT   10000.0f
#define TRIGGER_READY_PID_MAX_IOUT  200.0f

//Ħ���ֵ��rmp �仯�� ��ת�ٶȵı���
#define FRIC_RPM_TO_SPEED           0.000415809748903494517209f*5



//Ħ���ֵ��PID
#define FRIC_SPEED_PID_KP        1800.0f
#define FRIC_SPEED_PID_KI        0.5f
#define FRIC_SPEED_PID_KD        2.0f

#define FRIC_PID_MAX_OUT  8000.0f
#define FRIC_PID_MAX_IOUT 200.0f


#define FRIC_MAX_SPEED_RMP 4000.0f
#define FRIC_REQUIRE_SPEED_RMP 500.0f

#define SHOOT_HEAT_REMAIN_VALUE     80
//���̸���
#define TRIGGER_GRID_NUM 8     
#define TRIGGER_ONCE 2*PI/TRIGGER_GRID_NUM


#define LEFT 0
#define RIGHT 1



typedef enum
{
    SHOOT_STOP = 0,    //ֹͣ����ṹ
    SHOOT_READY_FRIC,  //Ħ����׼����
    SHOOT_READY_BULLET,//����׼����,Ħ�����Ѵﵽת��
    SHOOT_READY,       //�����������׼�����
    SHOOT_BULLET,      //����
    SHOOT_CONTINUE_BULLET,//����
    SHOOT_DONE,      
} shoot_mode_e;

typedef struct
{
  const motor_measure_t *fric_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;

  fp32 max_speed;  //Ħ������ת����ٶ�
  fp32 min_speed;  //Ħ������ת��С�ٶ�
  fp32 require_speed; //�����̿���������ٶ�

} fric_motor_t;

typedef struct
{
    shoot_mode_e shoot_mode;
    const RC_ctrl_t *shoot_rc;
    uint16_t shoot_last_key_v;

    //�����������
    const motor_measure_t *trigger_motor_measure;
    pid_type_def trigger_motor_pid;
    fp32 trigger_speed_set;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
    int16_t given_current;
    int8_t ecd_count;

    //Ħ���ֵ������
    fric_motor_t fric_motor[2]; 
    pid_type_def fric_speed_pid[2];

    //Ħ���ֵ�� ���ֶ�� ��λ���� ״̬
    bool_t fric_status;
    bool_t magazine_status;
    bool_t limit_switch_status;
  
    //���״̬
    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    uint16_t block_time;
    uint16_t reverse_time;
    bool_t move_flag;

    //΢������
    bool_t key;              
    uint8_t key_time;

    bool_t shoot_control_way; //������Ʒ�ʽ

 
} shoot_control_t;


extern shoot_control_t shoot_control;          //�������

//Ħ���ְ�������
#define KEY_FRIC (shoot_control.shoot_rc->key.v  & KEY_PRESSED_OFFSET_G) && !(shoot_control.shoot_last_key_v & KEY_PRESSED_OFFSET_G) 


//�����������̨ʹ��ͬһ��can��id��Ҳ�����������̨������ִ��
extern void shoot_task(void const *pvParameters);;
extern void shoot_init(void);
extern void shoot_set_control(void);
extern bool_t shoot_cmd_to_gimbal_stop(void);

#endif
