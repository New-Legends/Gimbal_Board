/**
  ****************************(C) COPYRIGHT 2021 *******************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     NOV-03-2021     Summerpray      1. doing
  *
  *
  @verbatim
  ==============================================================================
 *      ┌─┐       ┌─┐
 *   ┌──┘ ┴───────┘ ┴──┐
 *   │                 │
 *   │       ───       │
 *   │  ─┬┘       └┬─  │
 *   │                 │
 *   │       ─┴─       │
 *   │                 │
 *   └───┐         ┌───┘
 *       │         │
 *       │         │
 *       │         │
 *       │         └──────────────┐
 *       │                        │
 *       │                        ├─┐
 *       │                        ┌─┘
 *       │                        │
 *       └─┐  ┐  ┌───────┬──┐  ┌──┘
 *         │ ─┤ ─┤       │ ─┤ ─┤
 *         └──┴──┘       └──┴──┘
 *                神兽保佑
 *               代码无BUG!
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2021 *******************************
  */
#include "gimbal_task.h"

void gimbal_task(void *pvParameters)
{
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //云台初始化
    gimbal.init();
    //云台数据反馈
    gimbal.feedback_update();
    while (1)
    {
        
        //设置云台状态机
        gimbal.set_mode();
        //云台控制模式切换 控制数据过渡
        gimbal.mode_change_control_transit();
        //云台数据反馈
        gimbal.feedback_update();
        //设置云台控制量
        gimbal.set_control();
        //设置PID计算
        gimbal.gimbal_control_loop();
#if YAW_TURN
        gimbal.yaw_can_set_current = -gimbal.gimbal_yaw_motor.given_current;
#else
        gimbal.yaw_can_set_current = gimbal.gimbal_yaw_motor.given_current;
#endif

#if PITCH_TURN
        gimbal.pitch_can_set_current = -gimbal.gimbal_pitch_motor.given_current;
#else
        gimbal.pitch_can_set_current = gimbal.gimbal_pitch_motor.given_current;
#endif

        //gimbal.gimbal_can.cmd_gimbal(gimbal.yaw_can_set_current, gimbal.pitch_can_set_current, 0, 0);
        gimbal.gimbal_can.cmd_gimbal(0, 0, 0, 0);
        //gimbal.gimbal_can.CAN_cmd_gimbal_temp(0, gimbal.yaw_can_set_current, 0, 0);
        //TODO:看到这两个can发送机械的大兄弟有什么要说的嘛
    }
}
