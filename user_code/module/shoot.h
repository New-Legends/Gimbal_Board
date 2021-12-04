#ifndef GIMBAL_BOARD_SHOOTMODULE_H
#define GIMBAL_BOARD_SHOOTMODULE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "pid.h"

#ifdef __cplusplus
}
#endif

#include "Communicate.h"
#include "Motor.h"
#include "gimbal.h"


//微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

#define TRIGGER_CCW 1 //拨盘顺时针
#define TRIGGER_CW -1 //拨盘逆时针


#define SHOOT_TRIGGER_DIRECTION TRIGGER_CW

//射击发射开关通道数据
#define SHOOT_RC_MODE_CHANNEL       1
//云台模式使用的开关通道



//开启摩擦轮的斜坡
#define SHOOT_FRIC_ADD_VALUE    0.1f

//射击摩擦轮激光打开 关闭
#define SHOOT_KEYBOARD           KEY_PRESSED_OFFSET_G

//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME     15
//鼠标长按判断
#define PRESS_LONG_TIME             400
//摩擦轮开启按键延时
#define KEY_FRIC_LONG_TIME             200


//遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
#define RC_S_LONG_TIME              2000
//摩擦轮高速 加速 时间
#define UP_ADD_TIME                 80
//电机反馈码盘值范围
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191


//拨盘电机rmp 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f 
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f * SHOOT_TRIGGER_DIRECTION
#define FULL_COUNT                  18

//拨弹速度
#define TRIGGER_SPEED               10.0f * SHOOT_TRIGGER_DIRECTION   //10
#define CONTINUE_TRIGGER_SPEED      15.0f * SHOOT_TRIGGER_DIRECTION  //15
#define READY_TRIGGER_SPEED         5.0f * SHOOT_TRIGGER_DIRECTION //5

#define KEY_OFF_JUGUE_TIME          500
#define SWITCH_TRIGGER_ON           0
#define SWITCH_TRIGGER_OFF          1

//卡单时间 以及反转时间
#define BLOCK_TRIGGER_SPEED         1.0f
#define BLOCK_TIME                  700
#define REVERSE_TIME                500
#define REVERSE_SPEED_LIMIT         13.0f

#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_TEN                      0.314f

//拨弹轮电机PID
#define TRIGGER_ANGLE_PID_KP        1000.0f  //800
#define TRIGGER_ANGLE_PID_KI        2.0f  //0.5
#define TRIGGER_ANGLE_PID_KD        5.0f

#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 200.0f

#define TRIGGER_READY_PID_MAX_OUT   10000.0f
#define TRIGGER_READY_PID_MAX_IOUT  200.0f

//摩擦轮电机rmp 变化成 旋转速度的比例
#define FRIC_RPM_TO_SPEED           0.000415809748903494517209f*5

//摩擦轮电机PID
#define FRIC_SPEED_PID_KP        1800.0f
#define FRIC_SPEED_PID_KI        0.5f
#define FRIC_SPEED_PID_KD        2.0f

#define FRIC_PID_MAX_OUT  8000.0f
#define FRIC_PID_MAX_IOUT 200.0f


#define FRIC_MAX_SPEED_RMP 4000.0f
#define FRIC_REQUIRE_SPEED_RMP 500.0f

#define SHOOT_HEAT_REMAIN_VALUE     80
//拨盘格数
#define TRIGGER_GRID_NUM 8     
#define TRIGGER_ONCE 2*PI/TRIGGER_GRID_NUM


#define LEFT 0
#define RIGHT 1


//射击模式参数结构体
typedef struct
{
  const motor_measure *fric_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;

  fp32 max_speed;  //摩擦轮旋转最大速度
  fp32 min_speed;  //摩擦轮旋转最小速度
  fp32 require_speed; //允许拨盘开启的最低速度

} fric_motor_t;

typedef enum
{
    SHOOT_STOP = 0,    //停止发射结构
    SHOOT_READY_FRIC,  //摩擦轮准备中
    SHOOT_READY_BULLET,//拨盘准备中,摩擦轮已达到转速
    SHOOT_READY,       //整个发射机构准备完成
    SHOOT_BULLET,      //单发
    SHOOT_CONTINUE_BULLET,//连发
    SHOOT_DONE,      
} shoot_mode_e;

class shoot{
public:
    shoot_mode_e shoot_mode;                                        //射击模式参数结构体
    uint16_t shoot_last_key_v;

    CAN_Gimbal shoot_can;
    //拨弹电机数据
    const motor_measure *trigger_motor_measure;
    pid_type_def trigger_motor_pid;
    fp32 trigger_speed_set;
    fp32 speed_t;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
    int16_t given_current;
    int16_t ecd_count;

    fric_motor_t fric_motor[2]; 
    pid_type_def fric_speed_pid[2];

    //摩擦轮电机 弹仓舵机 限位开关 状态
    bool_t fric_status;
    bool_t magazine_status;
    bool_t limit_switch_status;

    //鼠标状态
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

    //微动开关
    bool_t button_key;              
    uint8_t key_time;

    void init(void);                                          //初始化
    void set_mode(void);                                      //射击状态机设置
    void feedback_update(void);                               //射击数据更新
    void set_control(void);                                   //射击循环
    void trigger_motor_turn_back(void);                       //堵转倒转处理
    void bullet_control(void);                                //控制拨弹电机角度
    bool_t cmd_to_gimbal_stop(void);                          //弹仓打开云台停止运动判断位
};

extern shoot Shoot;

//摩擦轮按键控制
#define KEY_FRIC (remote_control.rc_ctrl.key.v  & KEY_PRESSED_OFFSET_G) && !(Shoot.shoot_last_key_v & KEY_PRESSED_OFFSET_G) 

#endif
