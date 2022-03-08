/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             ���̿�������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "user_lib.h"

//in the beginning of task ,wait a time
//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357

//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL 2

//the channel of choosing chassis mode,
//ѡ�����״̬ ����ͨ����
#define CHASSIS_MODE_CHANNEL 0

//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.005f

#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//rocker value deadline
//ҡ������
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

//chassis task control time  2ms
//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//chassis task control time 0.002s
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//chassis control frequence, no use now.
//�����������Ƶ�ʣ���δʹ�������
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 16000.0f


//chassi forward, back, left, right key
//�������ҿ��ư���
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

//m3508 rmp change to chassis speed,
//m3508ת���ɵ����ٶ�(m/s)�ı�����
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//�������̵������ٶ�
#define MAX_WHEEL_SPEED 4.0f   //4
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f  //1.5

//����Ѳ���ٶȵȼ�
#define CHASSIS_LOW_SPEED 0.5*NORMAL_MAX_CHASSIS_SPEED_Y
#define CHASSIS_MID_SPEED 0.8*NORMAL_MAX_CHASSIS_SPEED_Y
#define CHASSIS_HIGH_SPEED 1.0*NORMAL_MAX_CHASSIS_SPEED_Y

//chassis motor speed PID
//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 6000.0f
#define M3505_MOTOR_SPEED_PID_KI 0.0f
#define M3505_MOTOR_SPEED_PID_KD 2.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT  6000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//�����˶�����
#define LEFT 0
#define RIGHT 1
#define NO_MOVE 2


typedef enum
{
  CHASSIS_VECTOR_NO_FOLLOW_YAW,       //chassis will have rotation speed control. ��������ת�ٶȿ���
  CHASSIS_VECTOR_RAW,                 //control-current will be sent to CAN bus derectly.

} chassis_mode_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} chassis_motor_t;

typedef struct
{
  const RC_ctrl_t *chassis_RC;               //����ʹ�õ�ң����ָ��, the point to remote control
  uint16_t chassis_last_key_v;               //��¼��һ�μ���ֵ
 
  const gimbal_motor_t *chassis_yaw_motor;   //will use the relative angle of yaw gimbal motor to calculate the euler angle.����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����.
  const gimbal_motor_t *chassis_pitch_motor; //will use the relative angle of pitch gimbal motor to calculate the euler angle.����ʹ�õ�pitch��̨�������ԽǶ���������̵�ŷ����
  const fp32 *chassis_INS_angle;             //the point to the euler angle of gyro sensor.��ȡ�����ǽ������ŷ����ָ��
  chassis_mode_e chassis_mode;               //state machine. ���̿���״̬��
  chassis_mode_e last_chassis_mode;          //last state machine.�����ϴο���״̬��
  chassis_motor_t motor_chassis;          //chassis motor data.���̵������
  pid_type_def motor_speed_pid;             //motor speed PID.���̵���ٶ�pid

  first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point.ʹ��һ�׵�ͨ�˲������趨ֵ

  fp32 vy;                          //chassis horizontal speed, positive means letf,unit m/s.�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  fp32 vy_set;                      //chassis set horizontal speed,positive means left,unit m/s.�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  fp32 chassis_relative_angle;      //the relative angle between chassis and gimbal.��������̨����ԽǶȣ���λ rad
  fp32 chassis_relative_angle_set;  //the set relative angle.���������̨���ƽǶ�
  fp32 chassis_yaw_set; 
              
  fp32 max_wheel_speed; //������������ת�ٶ�
  fp32 vy_max_speed;  //max letf speed, unit m/s.��������ٶ� ��λm/s
  fp32 vy_min_speed;  //max right speed, unit m/s.�ҷ�������ٶ� ��λm/s
  fp32 chassis_yaw;   //the yaw angle calculated by gyro sensor and gimbal motor.�����Ǻ���̨������ӵ�yaw�Ƕ�
  fp32 chassis_pitch; //the pitch angle calculated by gyro sensor and gimbal motor.�����Ǻ���̨������ӵ�pitch�Ƕ�
  fp32 chassis_roll;  //the roll angle calculated by gyro sensor and gimbal motor.�����Ǻ���̨������ӵ�roll�Ƕ�

  //Ѳ�߻��õ�������
  bool_t chassis_control_way; //���̿��Ʒ�ʽ
  bool_t left_light_sensor;  //����紫���� 0Ϊδ��Ӧ�� 1Ϊ��Ӧ��
  bool_t right_light_sensor;  //�Ҳ��紫���� 0Ϊδ��Ӧ�� 1Ϊ��Ӧ��
  uint16_t left_light_sensor_update_time; //��紫�������ݸ���ʱ��
  uint16_t right_light_sensor_update_time; //��紫�������ݸ���ʱ��
  uint8_t direction;          //�����ƶ����� ��ΪNO_MOVE LEFT RIGHT 
  

} chassis_move_t;

//�����˶�����
extern chassis_move_t chassis_move;

/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
extern void chassis_task(void const *pvParameters);


/**
  * @brief          ����ң����ͨ��ֵ����������ͺ����ٶ�
  *                 
  * @param[out]     vx_set: �����ٶ�ָ��
  * @param[out]     vy_set: �����ٶ�ָ��
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
  * @retval         none
  */
extern void chassis_rc_to_control_vector(fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
