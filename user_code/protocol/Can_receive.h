#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"

#include "struct_typedef.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define CHASSIS_CAN hcan2
#define BOARD_COM_CAN hcan1

//底盘动力电机编号
enum motive_chassis_motor_id_e
{
  //底盘动力电机接收
  MOTIVE_FR_MOTOR = 0,
  MOTIVE_FL_MOTOR,
  MOTIVE_BL_MOTOR,
  MOTIVE_BR_MOTOR,
};

//底盘舵向电机编号
enum rudde_chassisr_motor_id_e
{
  //底盘舵向电机
  RUDDER_FR_MOTOR = 0,
  RUDDER_FL_MOTOR,
  RUDDER_BL_MOTOR,
  RUDDER_BR_MOTOR,
};

/* CAN send and receive ID */
typedef enum
{
  //底盘动力电机接收ID  CAN2
  CAN_MOTIVE_FR_MOTOR_ID = 0x201,
  CAN_MOTIVE_FL_MOTOR_ID = 0x202,
  CAN_MOTIVE_BL_MOTOR_ID = 0x203,
  CAN_MOTIVE_BR_MOTOR_ID = 0x204,
  CAN_CHASSIS_MOTIVE_ALL_ID = 0x200,

  //底盘舵向电机ID CAN2
  CAN_RUDDER_FR_MOTOR_ID = 0x205,
  CAN_RUDDER_FL_MOTOR_ID = 0x206,
  CAN_RUDDER_BL_MOTOR_ID = 0x207,
  CAN_RUDDER_BR_MOTOR_ID = 0X208,
  CAN_CHASSIS_RUDDER_ALL_ID = 0x1FF,

  //板间通信ID
  CAN_RC_BOARM_COM_ID = 0x101,
  CAN_GIMBAL_BOARD_COM_ID = 0x102,
  CAN_COOLING_BOARM_COM_ID = 0x303,
  CAN_17MM_SPEED_BOARD_COM_ID = 0x304,

  //超级电容接收ID
  CAN_SUPER_CAP_ID = 0x211

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

//TODO 超电还未对接
// //rm motor data
// typedef struct
// {
//   fp32 input_vot;
//   fp32 supercap_vot;
//   fp32 input_current;
//   fp32 target_power;
// } super_cap_measure_t;

//底盘接收数据结构体
typedef struct
{
  //遥控器数据
  int16_t ch_0;
  int16_t ch_2;
  int16_t ch_3;
  uint16_t v;

  //云台状态
  uint8_t s0;
  uint8_t gimbal_behaviour;
  fp32    gimbal_yaw_angle;
} chassis_receive_t;


//底盘发送数据结构体
typedef struct
{
  //测试热量及ID
  uint16_t id1_17mm_cooling_limit;//17mm测速热量上限
  uint16_t id1_17mm_cooling_rate;//17mm测速热量冷却
  uint16_t id1_17mm_cooling_heat; //17mm测速实时热量
  uint8_t color;               //判断红蓝方
  uint8_t robot_id;            //机器人编号

  //测速速度及底盘模式
  uint16_t id1_17mm_speed_limi;//17mm测速射速上限
  uint16_t bullet_speed;       //17mm测速实时射速

  uint8_t chassis_behaviour;

} chassis_send_t;

typedef struct 
{
  float input_vot;      //输入电压
  float cap_vot;        //超级电容电压
  float input_current;  //输入电流
  float target_power;   //目标功率
} cap_receive_t;



class Can_receive {
public: 
  //动力电机反馈数据结构体
  motor_measure_t chassis_motive_motor[4];
  //舵向电机反馈数据结构体
  motor_measure_t chassis_rudder_motor[4];

  //发送数据结构体
  CAN_TxHeaderTypeDef chassis_tx_message;
  uint8_t chassis_can_send_data[8];

  //板间通信
  //底盘接收信息
  chassis_receive_t chassis_receive;

  chassis_send_t chassis_send;

  //超电数据
  cap_receive_t cap_receive;

  void init();

  //电机数据接收
  void get_motive_motor_measure(uint8_t num, uint8_t data[8]);

  void can_cmd_chassis_motive_motor(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);      //动力电机数据

  void can_cmd_chassis_motive_motor_reset_ID();

  const motor_measure_t *get_chassis_motive_motor_measure_point(uint8_t i);

  void get_rudder_motor_measure(uint8_t num, uint8_t data[8]);

  void can_cmd_chassis_rudder_motor(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);      //舵向电机数据

  const motor_measure_t *get_chassis_rudder_motor_measure_point(uint8_t i);

  //板间通信函数
  void receive_rc_board_com(uint8_t data[8]);

  void receive_gimbal_board_com(uint8_t data[8]);
  
  // 发送枪口热量及ID
  void send_cooling_and_id_board_com(uint16_t id1_17mm_cooling_limit, uint16_t id1_17mm_cooling_rate, uint16_t id1_17mm_cooling_heat, uint8_t color, uint8_t robot_id);
  //发送枪口速度及底盘模式
  void send_17mm_speed_and_mode_board_com(uint16_t id1_17mm_speed_limi, uint16_t bullet_speed, uint8_t chassis_behaviour);



  // 获取超电输入电压、电容电压、输入电流、设定功率
  void get_super_cap_data(uint8_t data[8]);

  //发送超级电容设定功率
  void can_cmd_super_cap_power(uint16_t set_power);

};



#endif
