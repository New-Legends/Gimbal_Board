/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       servo_task.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-21-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "servo_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_servo_pwm.h"
#include "remote_control.h"
#include "shoot_task.h"



#define SERVO_MIN_PWM   500
#define SERVO_MAX_PWM   2500

#define PWM_DETAL_VALUE 10

//���ֶ������
#define SERVO_MAGAZINE_KEY KEY_PRESSED_OFFSET_R


const RC_ctrl_t *servo_rc;
uint16_t servo_last_key_v = 0;;
//�������
#define KEY_SENVO (servo_rc->key.v  & KEY_PRESSED_OFFSET_R) && !(servo_last_key_v & KEY_PRESSED_OFFSET_R) 



//0�Ŷ��Ϊ���ֶ�� 1�Ŷ��Ϊ��λ���
//����˶���ʼλ��
const uint16_t servo_open_pwm[4] = {SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM};
//����˶�����λ��
const uint16_t servo_close_pwm[4] = {SERVO_MAX_PWM, SERVO_MAX_PWM, SERVO_MAX_PWM, SERVO_MAX_PWM};
//������͵Ŀ���ֵ
uint16_t servo_pwm[4] = {SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM};
//���ְ������� ��֤˫������´򿪵���
uint8_t magazine_key_num = 0;



/**
  * @brief          �������
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void servo_task(void const * argument)
{
    servo_rc = get_remote_control_point();

    while(1)
    {       
        if(KEY_SENVO)
            magazine_key_num++;
  
        //˫��R�� �򿪻򵯲�
        if(magazine_key_num >= 2 && shoot_control.magazine_status == FALSE)
        {   
            //�򿪵���
            servo_pwm[0] = servo_open_pwm[0];
            shoot_control.magazine_status = TRUE;
            magazine_key_num = 0;
        }
        else if(magazine_key_num >= 2 && shoot_control.magazine_status == TRUE)
        {
            //�رյ���
            servo_pwm[0] = servo_close_pwm[0];
            shoot_control.magazine_status = FALSE; 
            magazine_key_num = 0;   
        }

        
        //��λ����
        if((shoot_control.speed_set != 0) && shoot_control.limit_switch_status == FALSE)
        {
            //�����̵����ʼ��תʱ ��λ���ش�
            servo_pwm[1] = servo_open_pwm[1];
            shoot_control.limit_switch_status = TRUE; 

        }
        else if((shoot_control.speed_set == 0) && shoot_control.limit_switch_status == TRUE)
        {
            //�����̵��ֹͣ��תʱ ��λ���عر�
            servo_pwm[1] = servo_close_pwm[1];
            shoot_control.limit_switch_status = FALSE;
        }
            

        for(uint8_t i = 0; i < 4; i++)
        {
           //����pwm
            if(servo_pwm[i] < SERVO_MIN_PWM)
            {
                servo_pwm[i] = SERVO_MIN_PWM;
            }
            else if(servo_pwm[i] > SERVO_MAX_PWM)
            {
                servo_pwm[i] = SERVO_MAX_PWM;
            }

            servo_pwm_set(servo_pwm[i], i);
        }

        osDelay(1);
    }
}


