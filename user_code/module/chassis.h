#ifndef CHASSIS_H
#define CHASSIS_H

#include "start_task.h"

#include "struct_typedef.h"
#include "First_order_filter.h"
#include "Remote_control.h"
#include "Motor.h"
#include "Pid.h"
#include "Super_cap.h"

#include "Config.h"

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357

//左右的遥控器通道号码
#define CHASSIS_X_CHANNEL 2

//前后的遥控器通道号码
#define CHASSIS_Y_CHANNEL 3

//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 0

//初试yaw轴角度
#define INIT_YAW_SET 0.0f

//选择底盘状态 开关通道号
#define CHASSIS_MODE_CHANNEL 0

//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.006f

//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.005f

//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f

//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.01f

//一阶低通滤波参数
#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//摇杆死区
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f

//m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//gm6020转化成底盘速度(m/s)的比例，
#define GM6020_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f * 187 / 3591

//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 6.0f 
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 6.0f 
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_X 5.0f 
//底盘运动过程最大旋转速度
#define NORMAL_MAX_CHASSIS_SPEED_Z 12.0f


//原地旋转小陀螺下Z轴转速
#define TOP_WZ_ANGLE_STAND 3.0f
//移动状态下小陀螺转速
#define TOP_WZ_ANGLE_MOVE 0.7f


//摇摆原地不动摇摆最大角度(rad)
#define SWING_NO_MOVE_ANGLE 0.7f //0.7
//摇摆过程底盘运动最大角度(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

#define RUDDER_RADIUS 0.2f //轮中心距

//舵向电机初试位置拨码值
#define RUDDER_OFFSET_0 4780
#define RUDDER_OFFSET_1 4780
#define RUDDER_OFFSET_2 2082 //编码器
#define RUDDER_OFFSET_3 4780

//电机反馈码盘值范围
#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191

//电机编码值转化成角度值
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192

#define MIN_RUDDER_ANGLE -2*PI
#define MID_RUDDER_ANGLE 0.0f
#define MAX_RUDDER_ANGLE 2*PI

#define ACCEL_RUDDER_NUM 0.002f

#define MISS_CLOSE 0
#define MISS_BEGIN 1
#define MISS_OVER 2

#define PISA_DELAY_TIME 500
#define CHASSIS_OPEN_RC_SCALE 10 // in CHASSIS_OPEN mode, multiply the value. 在chassis_open 模型下，遥控器乘以该比例发送到can上

//chassis motor speed PID
//底盘电机速度环PID
#define MOTIVE_MOTOR_SPEED_PID_KP 2000.0f
#define MOTIVE_MOTOR_SPEED_PID_KI 0.0f
#define MOTIVE_MOTOR_SPEED_PID_KD 0.8f
#define MOTIVE_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
#define MOTIVE_MOTOR_SPEED_PID_MAX_OUT 6000.0f

//chassis follow angle PID
//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 8.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 4.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 2.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 20.0f

//底盘舵向电机 速度环 PID参数以及 PID最大输出，积分输出
#define RUDDER_MOTOR_SPEED_PID_KP 2900.0f //2900
#define RUDDER_MOTOR_SPEED_PID_KI 0.0f
#define RUDDER_MOTOR_SPEED_PID_KD 0.0f
#define RUDDER_MOTOR_SPEED_PID_MAX_IOUT 10000.0f
#define RUDDER_MOTOR_SPEED_PID_MAX_OUT 30000.0f

//底盘舵向电机 角度环 角度由编码器解算 PID参数以及 PID最大输出，积分输出
#define RUDDER_MATOR_ANGLE_PID_KP 20.0f //15
#define RUDDER_MATOR_ANGLE_PID_KI 0.0f
#define RUDDER_MATOR_ANGLE_PID_KD 5.0f
#define RUDDER_MATOR_ANGLE_PID_MAX_IOUT 0.0f
#define RUDDER_MATOR_ANGLE_PID_MAX_OUT 6.0f

//功率控制参数
#define POWER_DEFAULT_LIMIT 50.0f  //默认功率限制
#define WARNING_POWER_DISTANCE 10.0f //距离超过率的距离
#define WARNING_POWER_BUFF 30.0f   
 //警告能量缓冲  通过计算超级电容 电压低于12v得到的值

#define NO_JUDGE_TOTAL_CURRENT_LIMIT 64000.0f // 16000 * 4,
#define BUFFER_TOTAL_CURRENT_LIMIT 40000.0f
#define POWER_TOTAL_CURRENT_LIMIT 20000.0f

typedef enum {
    CHASSIS_ZERO_FORCE,                  // chassis will be like no power,底盘无力, 跟没上电那样
    CHASSIS_NO_MOVE,                     // chassis will be stop,底盘保持不动
    CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,  // chassis will follow gimbal, usually in infantry,正常步兵底盘跟随云台
    CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, // chassis will follow chassis yaw angle, usually in engineer,
                                         // because chassis does have gyro sensor, its yaw angle is calculed by gyro in gimbal and gimbal motor angle,
                                         // if you have a gyro sensor in chassis, please updata yaw, pitch, roll angle in "chassis_feedback_update"  function
                                         //工程底盘角度控制底盘，由于底盘未有陀螺仪，故而角度是减去云台角度而得到，
                                         //如果有底盘陀螺仪请更新底盘的yaw，pitch，roll角度 在chassis_feedback_update函数中
    CHASSIS_NO_FOLLOW_YAW, // chassis does not follow angle, angle is open-loop,but wheels have closed-loop speed
                           //底盘不跟随角度，角度是开环的，但轮子是有速度环
    CHASSIS_OPEN, // the value of remote control will mulitiply a value, get current value that will be sent to can bus
                  //  遥控器的值乘以比例成电流值 直接发送到can总线上
} chassis_behaviour_e;

typedef enum
{
    CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,  //底盘会跟随云台相对角度
    CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW, //底盘有底盘角度控制闭环
    CHASSIS_VECTOR_NO_FOLLOW_YAW,      // 底盘有旋转速度控制
    CHASSIS_VECTOR_RAW,                //电流之间控制,开环

} chassis_mode_e;


struct speed_t
{
    fp32 speed;
    fp32 speed_set;

    fp32 max_speed;
    fp32 min_speed;
};



class Chassis {
public:
    const RC_ctrl_t *chassis_RC; //底盘使用的遥控器指针
    RC_ctrl_t *last_chassis_RC; //底盘使用的遥控器指针

    chassis_behaviour_e chassis_behaviour_mode; //底盘行为状态机
    chassis_behaviour_e last_chassis_behaviour_mode; //底盘上次行为状态机

    chassis_mode_e chassis_mode; //底盘控制状态机
    chassis_mode_e last_chassis_mode; //底盘上次控制状态机

    M3508_motor chassis_motive_motor[4]; //底盘动力电机数据
    G6020_motor chassis_rudder_motor[4]; //底盘舵向电机数据

    First_order_filter chassis_cmd_slow_set_vx;        //使用一阶低通滤波减缓设定值
    First_order_filter chassis_cmd_slow_set_vy;        //使用一阶低通滤波减缓设定值

    Pid chassis_wz_angle_pid;        //底盘角度pid

    speed_t x;
    speed_t y;
    speed_t z;

    fp32 chassis_relative_angle;     //底盘与云台的相对角度，单位 rad
    fp32 chassis_relative_angle_set; //设置相对云台控制角度
    fp32 chassis_yaw_set;

    fp32 chassis_yaw;   //陀螺仪和云台电机叠加的yaw角度
    fp32 chassis_pitch; //.陀螺仪和云台电机叠加的pitch角度
    fp32 chassis_roll;  //陀螺仪和云台电机叠加的roll角度


    //任务流程
    void init();

    void set_mode();

    void feedback_update();

    void set_contorl();

    void solve();

    void power_ctrl();

    void output();

    //行为控制

    void chassis_behaviour_mode_set();

    void chassis_behaviour_control_set(fp32 *vx_set_, fp32 *vy_set_, fp32 *angle_set);

    void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set);

    void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set);

    void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set);

    void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set);

    void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set);

    void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set);

    //功能性函数
    void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set);

    void chassis_vector_to_mecanum_wheel_speed(fp32 wheel_speed[4], fp32 rudder_angle[4]);

    fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);


};


extern Chassis chassis;


//超电模块
extern Super_Cap cap;

#endif