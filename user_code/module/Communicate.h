#ifndef COMMUNICAT_H
#define COMMUNICAT_H

#include "cmsis_os.h"
#include "main.h"

#include "Remote_control.h"
#include "Can_receive.h"


//错误码以及对应设备顺序
enum errorList
{
    DBUS_TOE = 0,
    //CHASSIS_MOTOR1_TOE,
    //CHASSIS_MOTOR2_TOE,
    //CHASSIS_MOTOR3_TOE,
    //CHASSIS_MOTOR4_TOE,
    //SHOOT_LEFT_FRIC_MOTOR_ID,
    //SHOOT_RIGHT_FRIC_MOTOR_ID,
    //SHOOT_REV,
    //TRIGGER_MOTOR_TOE,
    YAW_GIMBAL_MOTOR_TOE,
    PITCH_GIMBAL_MOTOR_TOE,
    BOARD_GYRO_TOE,
    BOARD_ACCEL_TOE,
    BOARD_MAG_TOE,
    //REFEREE_TOE,
    RM_IMU_TOE,
    //OLED_TOE,
    ERROR_LIST_LENGHT,
};

class Communicate
{
public:
    void init();

    void receive();

    void send();


};

extern Remote_control remote_control;
extern CAN_Gimbal can_receive;

extern Communicate communicate;

#endif

