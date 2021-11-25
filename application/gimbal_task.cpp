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


void gimbal_task(void *pvParameters){
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    M_Gimbal GIMBAL;
    //��̨��ʼ��
    GIMBAL.init();
    //��̨���ݷ���
    GIMBAL.feedback_update();
    while(1){
        //������̨״̬��
        GIMBAL.set_mode();
        //��̨����ģʽ�л� �������ݹ���
        GIMBAL.mode_change_control_transit();
        //��̨���ݷ���
        GIMBAL.feedback_update();
        //������̨������
        GIMBAL.set_control();
        //����PID����
        GIMBAL.gimbal_control_loop();
#if YAW_TURN
        GIMBAL.yaw_can_set_current = -GIMBAL.gimbal_yaw_motor.given_current;
#else
        GIMBAL.yaw_can_set_current = GIMBAL.gimbal_yaw_motor.given_current;
#endif

#if PITCH_TURN
        GIMBAL.pitch_can_set_current = -GIMBAL.gimbal_pitch_motor.given_current;
#else
        GIMBAL.pitch_can_set_current = GIMBAL.gimbal_pitch_motor.given_current;
#endif

        GIMBAL.gimbal_can.CAN_cmd_gimbal(GIMBAL.yaw_can_set_current, GIMBAL.pitch_can_set_current, GIMBAL.shoot_can_set_current, GIMBAL.shoot_can_set_current);

    }
}

