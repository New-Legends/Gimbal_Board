//
// Created by WSJ on 2021/11/2.
//

#ifndef GIMBAL_BOARD_GIMBAL_H
#define GIMBAL_BOARD_GIMBAL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "pid.h"

#ifdef __cplusplus
}
#endif

#include "Motor.h"
#include "struct_typedef.h"
#include "first_order_filter.h"
#include "remote_control.h"
#include "Can_receive.h"
#include "user_lib.h"
#include "INS.h"
#include "Communicate.h"

extern CAN_Gimbal Can;

//pitch speed close-loop PID params, max out and max iout
//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_SPEED_PID_KP 2000.0f //2900
#define PITCH_SPEED_PID_KI 0.0f
#define PITCH_SPEED_PID_KD 0.0f
#define PITCH_SPEED_PID_MAX_OUT 30000.0f
#define PITCH_SPEED_PID_MAX_IOUT 10000.0f

//yaw speed close-loop PID params, max out and max iout
//yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAW_SPEED_PID_KP 1000.0f //1800
#define YAW_SPEED_PID_KI 0.1f    //20
#define YAW_SPEED_PID_KD 0.0f
#define YAW_SPEED_PID_MAX_OUT 30000.0f
#define YAW_SPEED_PID_MAX_IOUT 5000.0f

//pitch gyro angle close-loop PID params, max out and max iout
//pitch 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define PITCH_GYRO_ABSOLUTE_PID_KP 100.0f //15
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.1f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 6.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

//yaw gyro angle close-loop PID params, max out and max iout
//yaw 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define YAW_GYRO_ABSOLUTE_PID_KP 300.0f //26
#define YAW_GYRO_ABSOLUTE_PID_KI 0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD 0.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT 10000.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

//pitch encode angle close-loop PID params, max out and max iout
//pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define PITCH_ENCODE_RELATIVE_PID_KP 100.0f //15
#define PITCH_ENCODE_RELATIVE_PID_KI 0.0f
#define PITCH_ENCODE_RELATIVE_PID_KD 0.0f

#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 10000.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

//yaw encode angle close-loop PID params, max out and max iout
//yaw 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define YAW_ENCODE_RELATIVE_PID_KP 300.0f //8
#define YAW_ENCODE_RELATIVE_PID_KI 0.0f
#define YAW_ENCODE_RELATIVE_PID_KD 0.1f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT 10000.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT 1.0f

//任务开始空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201
//yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL 0
#define PITCH_CHANNEL 1
#define GIMBAL_MODE_CHANNEL 0

//turn 180°
//掉头180 按键
#define TURN_KEYBOARD KEY_PRESSED_OFFSET_F
//turn speed
//掉头云台速度
#define TURN_SPEED 0.04f
//测试按键尚未使用
#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R
//rocker value deadband
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND 10

#define YAW_RC_SEN -0.0000005f
#define PITCH_RC_SEN -0.00000006f //0.005

#define YAW_MOUSE_SEN 0.00005f
#define PITCH_MOUSE_SEN 0.00015f

#define YAW_ENCODE_SEN 0.01f
#define PITCH_ENCODE_SEN 0.01f

#define GIMBAL_CONTROL_TIME 1

//test mode, 0 close, 1 open
//云台测试模式 宏定义 0 为不使用测试模式
#define GIMBAL_TEST_MODE 0

//电机是否接反
#define PITCH_TURN 1
#define YAW_TURN 0

//电机码盘值最大以及中值
#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191

//限幅 需要自己手动校准
#define YAW_OFFSET 6506   //编码器
#define PITCH_OFFSET 4155 //编码器

//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR 0.1f
#define GIMBAL_INIT_STOP_TIME 100
#define GIMBAL_INIT_TIME 6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
//云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED 0.004f
#define GIMBAL_INIT_YAW_SPEED 0.005f

#define INIT_YAW_SET 0.0f
#define INIT_PITCH_SET 0.0f

//云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define GIMBAL_CALI_MOTOR_SET 8000
#define GIMBAL_CALI_STEP_TIME 2000
#define GIMBAL_CALI_GYRO_LIMIT 0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP 1
#define GIMBAL_CALI_PITCH_MIN_STEP 2
#define GIMBAL_CALI_YAW_MAX_STEP 3
#define GIMBAL_CALI_YAW_MIN_STEP 4

#define GIMBAL_CALI_START_STEP GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP 5

//判断遥控器无输入的时间以及遥控器无输入判断，设置云台yaw回中值以防陀螺仪漂移
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX 3000

//电机编码值转化成角度值
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif

//云台状态机
typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
} gimbal_motor_mode_e;

//云台行为模式
typedef enum
{
    GIMBAL_ZERO_FORCE = 0,
    GIMBAL_INIT,
    GIMBAL_CALI,
    GIMBAL_ABSOLUTE_ANGLE,
    GIMBAL_RELATIVE_ANGLE,
    GIMBAL_MOTIONLESS,
} gimbal_behaviour_e;

//云台校准结构体
typedef struct
{
    fp32 max_yaw;
    fp32 min_yaw;
    fp32 max_pitch;
    fp32 min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} gimbal_step_cali_t;

class Gimbal
{
public:
    const RC_ctrl_t *gimbal_RC; //云台使用的遥控器指针
    const motor_measure *gimbal_motor_measure;
    const fp32 *gimbal_INT_angle_point;         //获取陀螺仪角度值
    const fp32 *gimbal_INT_gyro_point;          //获取陀螺仪角速度值
    motor_6020 gimbal_yaw_motor;                //yaw云台电机
    motor_6020 gimbal_pitch_motor;              //pitch云台电机
    gimbal_motor_mode_e gimbal_motor_mode;      //云台控制状态机
    gimbal_motor_mode_e last_gimbal_motor_mode; //云台上次控制状态机
    gimbal_behaviour_e gimbal_behaviour;        //云台行为模式

    //IMU Gimbal_imu;                                                 //陀螺仪接口

    pid_type_def gimbal_speed_pid[2]; //云台电机速度pid
    pid_type_def gimbal_angle_pid[2]; //云台电机角度pid

    First_order_filter gimbal_cmd_slow_set_vx; //使用一阶低通滤波减缓设定值
    First_order_filter gimbal_cmd_slow_set_vy; //使用一阶低通滤波减缓设定值

    fp32 max_yaw;   //yaw电机最大值限位
    fp32 min_yaw;   //yaw电机最小值限位
    fp32 max_pitch; //pitch电机最大值限位
    fp32 min_pitch; //pitch电机最小值限位
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;

    //发送的电机电流
    int16_t yaw_can_set_current;
    int16_t pitch_can_set_current;

    void init();                        //云台初始化
    void set_mode();                    //设置云台控制模式
    void set_control();                 //设置云台控制量
    void behavour_set();                //设置云台行为状态机
    void feedback_update();             //云台数据反馈
    void behaviour_mode_set();          //云台行为状态机及电机状态机设置
    void gimbal_control_loop();         //云台控制PID计算
    void mode_change_control_transit(); //转换状态保存数据

    /***************************(C)  MOTOR control *******************************/
    void absolute_angle_limit(motor_6020 *gimbal_motor, fp32 add); //陀螺仪模式电机计算
    void relative_angle_limit(motor_6020 *gimbal_motor, fp32 add); //编码器模式电机计算
    void motor_raw_angle_control(motor_6020 *gimbal_motor);        //云台直接电流计算
    void motor_absolute_angle_control(motor_6020 *gimbal_motor);   //云台陀螺仪模式电流计算
    void motor_relative_angle_control(motor_6020 *gimbal_motor);   //云台编码器模式电流计算
                                                                   /***************************(C)  MOTOR control *******************************/

    /***************************(C) GIMBAL control *******************************/
    void behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch); //云台行为控制
    void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch);     //不上电模式
    void gimbal_init_control(fp32 *yaw, fp32 *pitch);           //初始化模式
    void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch); //陀螺仪模式
    void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch); //编码器模式
    void gimbal_motionless_control(fp32 *yaw, fp32 *pitch);     //无输入控制模式
    /***************************(C) GIMBAL control *******************************/

    static void PID_clear(gimbal_PID_t *gimbal_pid_clear);      //清除pid
    static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);
    static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
    static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);

    /***************************(C) GIMBAL CALI *******************************/

    gimbal_step_cali_t gimbal_cali;
    void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);
    //云台校准设置
    bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);
    //云台校准发送
    static void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);
    //云台校准计算
    /***************************(C) GIMBAL CALI *******************************/

    bool_t gimbal_cmd_to_shoot_stop(void);
};

extern Gimbal gimbal;

#endif
