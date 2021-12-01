/**
  ****************************(C) COPYRIGHT 2021 *******************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ���������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     NOV-03-2021     Summerpray      1. doing
  *
  *
  @verbatim
  ==============================================================================
 *      ������       ������
 *   �������� �ة��������������� �ة�����
 *   ��                 ��
 *   ��       ������       ��
 *   ��  ���Щ�       ���Щ�  ��
 *   ��                 ��
 *   ��       ���ة�       ��
 *   ��                 ��
 *   ����������         ����������
 *       ��         ��
 *       ��         ��
 *       ��         ��
 *       ��         ��������������������������������
 *       ��                        ��
 *       ��                        ������
 *       ��                        ������
 *       ��                        ��
 *       ������  ��  �����������������Щ�����  ��������
 *         �� ���� ����       �� ���� ����
 *         �������ة�����       �������ة�����
 *                ���ޱ���
 *               ������BUG!
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2021 *******************************
  */
#include "gimbal_task.h"

void gimbal_task(void *pvParameters)
{
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //��̨��ʼ��
    gimbal.init();
    //��̨���ݷ���
    gimbal.feedback_update();
    while (1)
    {
        
        //������̨״̬��
        gimbal.set_mode();
        //��̨����ģʽ�л� �������ݹ���
        gimbal.mode_change_control_transit();
        //��̨���ݷ���
        gimbal.feedback_update();
        //������̨������
        gimbal.set_control();
        //����PID����
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
        //TODO:����������can���ͻ�е�Ĵ��ֵ���ʲôҪ˵����
    }
}
