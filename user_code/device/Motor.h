#ifndef MOTOR_H
#define MOTOR_H

#include "Pid.h"
#include "Can_receive.h"

//m3508电机
class M3508_motor
{
public:
    const motor_measure_t *motor_measure;
    //速度环pid和角度环pid, 用户可以选择性开启
    Pid speed_pid;
    Pid angle_pid;

    fp32 accel;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 angle_set;

    fp32 current_set;
    int16_t current_give;

    void init(const motor_measure_t *motor_measure_);
} ;

//G6020电机
class G6020_motor
{
public:
    const motor_measure_t *motor_measure;
    //速度环pid和角度环pid, 用户可以选择性开启
    Pid speed_pid;
    Pid angle_pid;

    uint16_t offset_ecd;  //用户定义的初始中值

    fp32 max_angle; //rad   角度限幅
    fp32 mid_angle; //rad
    fp32 min_angle; //rad

    fp32 angle;
    fp32 angle_set;
    fp32 speed;
    fp32 speed_set;
    fp32 current_set;
    int16_t current_give;

    void init(const motor_measure_t *motor_measure_);
};

//云台状态机
typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
} gimbal_motor_mode_e;

//gimbal电机
class Gimbal_motor
{
public:
    const motor_measure_t *motor_measure;
    //初始化电机控制模式
    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;

    //速度环pid和角度环pid, 用户可以选择性开启
    Pid speed_pid;
    Pid absolute_angle_pid;
    Pid relative_angle_pid;

    uint16_t offset_ecd; //用户定义的初始中值

    fp32 max_relative_angle; //rad   角度限幅
    fp32 mid_relative_angle; //rad
    fp32 min_relative_angle; //rad

    fp32 max_absolute_angle; //rad
    fp32 min_absolute_angle; //rad
    fp32 mid_absolute_angle; //rad

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 speed;
    fp32 speed_set;
    fp32 current_set;
    int16_t current_give;

    void init(const motor_measure_t *motor_measure_);
};

//摩擦轮电机
class Firc_motor
{
public:
    const motor_measure_t *motor_measure;
    //速度环pid和角度环pid, 用户可以选择性开启
    Pid speed_pid;
    Pid angle_pid;

    fp32 accel;
    fp32 speed;
    fp32 speed_set;

    fp32 max_speed;     //摩擦轮旋转最大速度
    fp32 min_speed;     //摩擦轮旋转最小速度
    fp32 require_speed; //允许拨盘开启的最低速度

    fp32 current_set;
    int16_t current_give;

    void init(const motor_measure_t *motor_measure_);
};

//拨弹电机
class Trigger_motor
{
public:
    const motor_measure_t *motor_measure;
    //速度环pid和角度环pid, 用户可以选择性开启
    Pid speed_pid;
    Pid angle_pid;

    fp32 accel;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 angle_set;

    fp32 current_set;
    int16_t current_give;

    int8_t ecd_count;    ///编码值计数

    void init(const motor_measure_t *motor_measure_);
};

#endif