#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "pid.h"
#include "struct_typedef.h"

class motor_measure {
public:
    uint16_t ecd;
    uint16_t speed_rpm;
    uint16_t given_current;
    uint8_t temperate;
    uint16_t last_ecd;

    void get_motor_measure(const uint8_t *data);

};

class motor_6020 {
public:
    const motor_measure *gimbal_motor_measure;
    gimbal_PID_t gimbal_motor_absolute_angle_pid;
    gimbal_PID_t gimbal_motor_relative_angle_pid;
    pid_type_def gimbal_motor_gyro_pid;
    uint16_t offset_ecd;
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;
    fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;
};

class motor_3508 {
public:
    const motor_measure *gimbal_motor_measure;
    fp32 accel;
    fp32 speed;
    fp32 speed_set;
    int16_t give_current;
};




#endif

