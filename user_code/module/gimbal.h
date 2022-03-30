//
// Created by WSJ on 2021/11/2.
//

#ifndef GIMBAL_H
#define GIMBAL_H

#include "Motor.h"
#include "struct_typedef.h"
#include "First_high_pass_filter.h"
#include "Remote_control.h"
#include "Can_receive.h"
#include "user_lib.h"
#include "INS.h"
#include "Communicate.h"

#include "config.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "bsp_buzzer.h"

#ifdef __cplusplus
}
#endif

#include "gimbal_task.h"



/*----------------------pid系数------------------------*/
//yaw speed close-loop PID params, max out and max iout
//yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAW_SPEED_PID_KP 8000.0f 
#define YAW_SPEED_PID_KI 0.0f    
#define YAW_SPEED_PID_KD 0.0f
#define YAW_SPEED_PID_MAX_IOUT 200.0f
#define YAW_SPEED_PID_MAX_OUT 30000.0f

//pitch speed close-loop PID params, max out and max iout
//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_SPEED_PID_KP 8000.0f //2900
#define PITCH_SPEED_PID_KI 0.0f
#define PITCH_SPEED_PID_KD 0.0f
#define PITCH_SPEED_PID_MAX_IOUT 25.0f
#define PITCH_SPEED_PID_MAX_OUT 12000.0f

//yaw gyro angle close-loop PID params, max out and max iout
//yaw 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define YAW_GYRO_ABSOLUTE_PID_KP 30.0f 
#define YAW_GYRO_ABSOLUTE_PID_KI 0.1f
#define YAW_GYRO_ABSOLUTE_PID_KD 2.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT 1.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT 40.0f

//pitch gyro angle close-loop PID params, max out and max iout
//pitch 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define PITCH_GYRO_ABSOLUTE_PID_KP 100.0f 
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.1f
#define PITCH_GYRO_ABSOLUTE_PID_KD 3.0f  //0.1
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 1.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 150.0f

//yaw encode angle close-loop PID params, max out and max iout
//yaw 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define YAW_ENCODE_RELATIVE_PID_KP 5.0f 
#define YAW_ENCODE_RELATIVE_PID_KI 0.0f
#define YAW_ENCODE_RELATIVE_PID_KD 0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT 5.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT 20.0f

//pitch encode angle close-loop PID params, max out and max iout
//pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define PITCH_ENCODE_RELATIVE_PID_KP 30.0f 
#define PITCH_ENCODE_RELATIVE_PID_KI 0.01f
#define PITCH_ENCODE_RELATIVE_PID_KD 0.2f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 1.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 160.0f

/*------------------------------自瞄PID------------------------*/
// yaw gyro angle close-loop PID params, max out and max iout
// yaw 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define YAW_AUTO_PID_KP 30.0f 
#define YAW_AUTO_PID_KI 0.1f
#define YAW_AUTO_PID_KD 2.0f
#define YAW_AUTO_PID_MAX_IOUT 1.0f
#define YAW_AUTO_PID_MAX_OUT 40.0f



// pitch gyro angle close-loop PID params, max out and max iout
// pitch 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define PITCH_AUTO_PID_KP 100.0f
#define PITCH_AUTO_PID_KI 0.1f
#define PITCH_AUTO_PID_KD 3.0f // 0.1
#define PITCH_AUTO_PID_MAX_IOUT 1.0f
#define PITCH_AUTO_PID_MAX_OUT 150.0f


/*---------------------按键--------------------*/
//yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL 0
#define PITCH_CHANNEL 1
#define GIMBAL_MODE_CHANNEL 0

//掉头云台速度
#define TURN_SPEED 0.04f

//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND 10
//云台 遥控器速度
#define YAW_RC_SEN -0.000002f // 右手系 z轴逆时针为正 但是遥控器通道向右为正 故加负号
#define PITCH_RC_SEN 0.000004f

//云台 鼠标速度 
#define YAW_MOUSE_SEN -0.00025f
#define PITCH_MOUSE_SEN -0.00016f

#define YAW_ENCODE_SEN 0.01f
#define PITCH_ENCODE_SEN 0.01f

#define GIMBAL_CONTROL_TIME 1

//云台测试模式 宏定义 0 为不使用测试模式
#define GIMBAL_TEST_MODE 0

//一阶高通滤波参数
#define GIMBAL_ACCEL_YAW_NUM 0.1666666667f
#define GIMBAL_ACCEL_PITCH_NUM 0.3333333333f

#define GIMBAL_CONTROL_TIME 0.001f

/*---------------------云台限幅与安装参数--------------------*/
//电机顺序
#define YAW 0
#define PITCH 1

//电机是否接反
#define YAW_TURN 1
#define PITCH_TURN 0

//电机码盘值最大以及中值
#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191

//限幅 需要自己手动校准
#define YAW_OFFSET 3899   //编码器
#define PITCH_OFFSET 5472 //编码器

//限幅
#define MAX_ABSOULATE_YAW PI
#define MIN_ABSOULATE_YAW -PI

#define MAX_ABSOULATE_PITCH 0.2f
#define MIN_ABSOULATE_PITCH -0.3f

#define MAX_RELATIVE_YAW PI
#define MIN_RELATIVE_YAW -PI

#define MAX_RELATIVE_PITCH 0.42f
#define MIN_RELATIVE_PITCH -0.45f



/*---------------------云台校准--------------------*/
//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR 0.05f
#define GIMBAL_INIT_STOP_TIME 200 //100
#define GIMBAL_INIT_TIME 6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
// //云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED 0.002f //0.02
#define GIMBAL_INIT_YAW_SPEED 0.002f  //0.02

#define INIT_YAW_SET 0.0f
#define INIT_PITCH_SET 0.0f

//云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define GIMBAL_CALI_MOTOR_SET 8000
#define GIMBAL_CALI_STEP_TIME 2000
#define GIMBAL_CALI_STEP_TIME 0.1f

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

//gm6020转化成底盘速度(m/s)的比例，
#define GM6020_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f * 187 / 3591


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

typedef struct 
{
    bool_t init_flag;

    //kp, ki kd为可调参数
    //ref set 为绘制的曲线,分别是测量值,设定值
    //使用stm stdio进行调试 直接输入下列的变量 注意修改debug模式的宏定义
    fp32 yaw_speed_kp;
    fp32 yaw_speed_ki;
    fp32 yaw_speed_kd;
    fp32 yaw_speed_ref;
    fp32 yaw_speed_set;

    fp32 yaw_relative_kp;
    fp32 yaw_relative_ki;
    fp32 yaw_relative_kd;
    fp32 yaw_relative_ref;
    fp32 yaw_relative_set;

    fp32 yaw_absolute_kp;
    fp32 yaw_absolute_ki;
    fp32 yaw_absolute_kd;
    fp32 yaw_absolute_ref;
    fp32 yaw_absolute_set;

    fp32 pitch_speed_kp;
    fp32 pitch_speed_ki;
    fp32 pitch_speed_kd;
    fp32 pitch_speed_ref;
    fp32 pitch_speed_set;

    fp32 pitch_relative_kp;
    fp32 pitch_relative_ki;
    fp32 pitch_relative_kd;
    fp32 pitch_relative_ref;
    fp32 pitch_relative_set;

    fp32 pitch_absolute_kp;
    fp32 pitch_absolute_ki;
    fp32 pitch_absolute_kd;
    fp32 pitch_absolute_ref;
    fp32 pitch_absolute_set;

} gimbal_debug_data_t;


class Gimbal
{
public:
    const RC_ctrl_t *gimbal_RC; //云台使用的遥控器指针
    RC_ctrl_t *last_gimbal_RC; //云台使用的遥控器指针
    //鼠标状态
    bool_t press_r;
    bool_t last_press_r;
    uint16_t press_r_time;

    uint16_t gimbal_last_key_v; //遥控器上次按键

    gimbal_behaviour_e gimbal_behaviour_mode;        //云台行为模式
    gimbal_behaviour_e last_gimbal_behaviour_mode;   //云台上次控制状态机

    Gimbal_motor gimbal_yaw_motor; //云台yaw电机数据
    Gimbal_motor gimbal_pitch_motor; //云台pitch电机数据

    First_high_pass_filter gimbal_yaw_high_pass_filter; //云台yaw电机一阶高通滤波
    First_high_pass_filter gimbal_pitch_high_pass_filter; //云台pitch电机一阶高通滤波

    //陀螺仪接口
    const fp32 *gimbal_INT_angle_point; //获取陀螺仪角度值
    const fp32 *gimbal_INT_gyro_point;  //获取陀螺仪角速度值
    uint8_t step;
    
    void init();                        //云台初始化
    void set_mode();                    //设置云台控制模式
    void feedback_update();             //云台数据反馈
    void set_control();                 //设置云台控制量
    void solve();                       //云台控制PID计算
    void output();                      //输出电流

    /***************************(C)  MOTOR control *******************************/
    void absolute_angle_limit(Gimbal_motor *gimbal_motor, fp32 add); //陀螺仪模式电机计算
    void relative_angle_limit(Gimbal_motor *gimbal_motor, fp32 add); //编码器模式电机计算
    void motor_raw_angle_control(Gimbal_motor *gimbal_motor);        //云台直接电流计算
    void motor_absolute_angle_control(Gimbal_motor *gimbal_motor);   //云台陀螺仪模式电流计算
    void motor_relative_angle_control(Gimbal_motor *gimbal_motor);   //云台编码器模式电流计算
    /***************************(C)  MOTOR control *******************************/

    /***************************(C) GIMBAL control *******************************/
    void behavour_set(); //设置云台行为状态机
    void behaviour_mode_set(); //云台行为状态机及电机状态机设置
    void behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch); //云台行为控制
    void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch);     //不上电模式
    void gimbal_init_control(fp32 *yaw, fp32 *pitch);           //初始化模式
    void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch); //陀螺仪模式
    void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch); //编码器模式
    void gimbal_motionless_control(fp32 *yaw, fp32 *pitch);     //无输入控制模式
    /***************************(C) GIMBAL control *******************************/

    static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);

    /***************************(C) GIMBAL CALI *******************************/
    gimbal_step_cali_t gimbal_cali;
    void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);
    
    void set_hand_operator_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);
    
    //云台校准设置
    bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);
    //云台校准发送
    static void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);
    //云台校准计算
    /***************************(C) GIMBAL CALI *******************************/

};


void gimbal_debug();

bool_t gimbal_cmd_to_shoot_stop(void);

extern Gimbal gimbal;

#endif
