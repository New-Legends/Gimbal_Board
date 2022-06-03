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

    fp32 current_set;

    int16_t current_give;

    void init(const motor_measure_t *motor_measure_);
};

//m3508电机
class G6020_motor
{
public:
    const motor_measure_t *motor_measure;
    //速度环pid和角度环pid, 用户可以选择性开启
    Pid speed_pid;
    Pid angle_pid;

    uint16_t offset_ecd;  //用户定义的初始中值
    uint16_t if_turn_falg;   //电机是否过圈

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

#endif