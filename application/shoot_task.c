/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      �������.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "shoot_task.h"
#include "main.h"
#include "vision.h"
#include "cmsis_os.h"

#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "referee_control.h"

#define shoot_fric1_on(pwm) fric1_on((pwm)) //Ħ����1pwm�궨��
#define shoot_fric2_on(pwm) fric2_on((pwm)) //Ħ����2pwm�궨��
#define shoot_fric_off()    fric_off()      //�ر�����Ħ����

#define shoot_laser_on()    laser_on()      //���⿪���궨��
#define shoot_laser_off()   laser_off()     //����رպ궨��
//΢������IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)




/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(shoot_control_t *shoot_control);
/**
  * @brief          ������ݸ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          ��ת��ת����
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void);

/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);

shoot_control_t shoot_control;          //�������
VisionRecvData_t      VisionRecvData_shoot;        //�������ݽṹ��



/**
  * @brief          ������񣬳�ʼ��PID��ң����ָ�룬���ָ��
  * @param[in]      void
  * @retval         ���ؿ�
  */
void shoot_task(void const *pvParameters)
{
    //��ʼ����ʱ
    vTaskDelay(SHOOT_TASK_INIT_TIME); 
    //����ṹ��ʼ��   
    shoot_init();

    while(1)
    {
        shoot_set_mode(&shoot_control);        //����״̬��
        shoot_feedback_update(); //��������
        shoot_set_control();        //����������ѭ��


        //ȷ������һ��������ߣ� ����CAN���ư����Ա����յ�
        if (!((toe_is_error(TRIGGER_MOTOR_TOE) && toe_is_error(SHOOT_LEFT_FRIC_MOTOR_TOE) && toe_is_error(SHOOT_RIGHT_FRIC_MOTOR_TOE)&& toe_is_error(CHASSIS_MOTOR_TOE))))
        {
            //��ң�������ߵ�ʱ�򣬷��͸���������.
            if (toe_is_error(DBUS_TOE))
            {
                CAN_cmd_chassis_shoot(0, 0, 0, 0);
            }
            else
            {
                //���Ϳ��Ƶ���
                CAN_cmd_chassis_shoot(shoot_control.given_current, shoot_control.fric_motor[LEFT].give_current, shoot_control.fric_motor[RIGHT].give_current, chassis_move.motor_chassis.give_current);
                //CAN_cmd_chassis_shoot(0, 0, 0, 0);
            }
        }

        vTaskDelay(SHOOT_CONTROL_TIME);
    }
}

/**
  * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
  * @param[in]      void
  * @retval         ���ؿ�
  */
void shoot_init(void)
{
    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    static const fp32 Fric_speed_pid[3] = {FRIC_SPEED_PID_KP, FRIC_SPEED_PID_KI, FRIC_SPEED_PID_KD};

    shoot_control.shoot_mode = SHOOT_STOP;
    //ң����ָ��
    shoot_control.shoot_rc = get_remote_control_point();

    //���ָ�� ���� Ħ����
    shoot_control.trigger_motor_measure = get_trigger_motor_measure_point();
    shoot_control.fric_motor[LEFT].fric_motor_measure = get_fric_motor_measure_point(LEFT);
    shoot_control.fric_motor[RIGHT].fric_motor_measure = get_fric_motor_measure_point(RIGHT);

    //��ʼ��PID
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_init(&shoot_control.fric_speed_pid[LEFT], PID_POSITION, Fric_speed_pid, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
    PID_init(&shoot_control.fric_speed_pid[RIGHT], PID_POSITION, Fric_speed_pid, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);

    //������� ��Сֵ  ��Ħ����˳ʱ��ת ��Ħ������ʱ��ת
    shoot_control.fric_motor[LEFT].max_speed = FRIC_MAX_SPEED_RMP;
    shoot_control.fric_motor[LEFT].min_speed = -FRIC_MAX_SPEED_RMP;
    shoot_control.fric_motor[LEFT].require_speed = -FRIC_REQUIRE_SPEED_RMP;

    shoot_control.fric_motor[RIGHT].max_speed = FRIC_MAX_SPEED_RMP;
    shoot_control.fric_motor[RIGHT].min_speed = -FRIC_MAX_SPEED_RMP;
    shoot_control.fric_motor[RIGHT].require_speed = -FRIC_REQUIRE_SPEED_RMP;
    
    //Ħ����,���ֶ��,��λ���״̬
    shoot_control.fric_status = FALSE;
    shoot_control.magazine_status = FALSE;
    shoot_control.limit_switch_status = FALSE;

    //��¼��һ�ΰ���ֵ
    shoot_control.shoot_last_key_v = 0;  

    //��������
    shoot_feedback_update();

    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.trigger_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.set_angle = shoot_control.angle;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;
}
/**
  * @brief          ������ݸ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{

    //����Ħ���ֵ���ٶ�
    shoot_control.fric_motor[LEFT].speed = shoot_control.fric_motor[LEFT].fric_motor_measure->speed_rpm * FRIC_RPM_TO_SPEED;
    shoot_control.fric_motor[RIGHT].speed = shoot_control.fric_motor[RIGHT].fric_motor_measure->speed_rpm * FRIC_RPM_TO_SPEED;


    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //�����ֵ���ٶ��˲�һ��
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //���׵�ͨ�˲�
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.trigger_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;

    //���Ȧ�����ã� ��Ϊ�������תһȦ�� �������ת 36Ȧ������������ݴ������������ݣ����ڿ��������Ƕ�
    if (shoot_control.trigger_motor_measure->ecd - shoot_control.trigger_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        shoot_control.ecd_count--;
    }
    else if (shoot_control.trigger_motor_measure->ecd - shoot_control.trigger_motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        shoot_control.ecd_count++;
    }

    if (shoot_control.ecd_count == FULL_COUNT)
    {
        shoot_control.ecd_count = -(FULL_COUNT - 1);
    }
    else if (shoot_control.ecd_count == -FULL_COUNT)
    {
        shoot_control.ecd_count = FULL_COUNT - 1;
    }

    //���������Ƕ�
    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.trigger_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
    //΢������
    shoot_control.key = BUTTEN_TRIG_PIN;
    //��갴��
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
    //������ʱ
    if (shoot_control.press_l)
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME)
        {
            shoot_control.press_l_time++;
        }
    }
    else
    {
        shoot_control.press_l_time = 0;
    }
    
    //��������µ�ʱ���ʱ
    if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {

        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_control.rc_s_time++;
        }
    }
    else
    {
        shoot_control.rc_s_time = 0;
    }

    
}

/**
  * @brief          ���ѭ��
  * @param[in]      void
  * @retval         ����can����ֵ
  */
void shoot_set_control(void)
{
    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        //���ò����ֵ��ٶ�
        shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
    {
        //���ò����ֵ��ٶ�
        shoot_control.speed_set = 0.0f;
    }
    else if(shoot_control.shoot_mode ==SHOOT_READY_BULLET)
    {
        if(shoot_control.key == SWITCH_TRIGGER_OFF)
        {
            //���ò����ֵĲ����ٶ�,��������ת��ת����
            shoot_control.trigger_speed_set = shoot_grigger_grade[1] * SHOOT_TRIGGER_DIRECTION;
        }
        else
        {
            shoot_control.trigger_speed_set = 0.0f;
            shoot_control.speed_set = 0.0f;
        }
        shoot_control.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY)
    {
        //���ò����ֵ��ٶ�
         shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_BULLET)
    {
        shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control();
    }
    else if (shoot_control.shoot_mode ==   SHOOT_CONTINUE_BULLET)
    {
        //���ò����ֵĲ����ٶ�,��������ת��ת����
        shoot_control.trigger_speed_set =  shoot_grigger_grade[2] * SHOOT_TRIGGER_DIRECTION;
    }
    else if(shoot_control.shoot_mode == SHOOT_DONE)
    {
        shoot_control.speed_set = 0.0f;
    }

    if(shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_laser_off();
        shoot_control.given_current = 0;
        shoot_control.fric_motor[LEFT].give_current = 0.0f;
        shoot_control.fric_motor[RIGHT].give_current = 0.0f;
        shoot_control.fric_status = FALSE;

    }
    else
    {
        shoot_laser_on(); //���⿪��
        //����Ħ����ת��
        shoot_control.fric_motor[LEFT].speed_set = -shoot_fric_grade[2];
        shoot_control.fric_motor[RIGHT].speed_set = shoot_fric_grade[2];

//        //����ģʽ ����17mm����������ٺ���������
//        if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
//            shoot_id1_17mm_speed_and_cooling_control(&shoot_control);


        if(shoot_control.shoot_mode == SHOOT_READY_BULLET || shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
            trigger_motor_turn_back();  //�����õĲ�����ת�Ƕ�,ת��Ϊ�ٶ�,�ҷ�ֹ����

        //���㲦���ֵ��PID
        PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
        //����Ħ���ֵ��PID
        PID_calc(&shoot_control.fric_speed_pid[LEFT], shoot_control.fric_motor[LEFT].speed, shoot_control.fric_motor[LEFT].speed_set);
        PID_calc(&shoot_control.fric_speed_pid[RIGHT], shoot_control.fric_motor[RIGHT].speed, shoot_control.fric_motor[RIGHT].speed_set);    
        
        //ȷ��Ħ����δ�ﵽ���ת�ٲ���ת������
        if(shoot_control.shoot_mode < SHOOT_READY_BULLET)
        {
            shoot_control.given_current = 0;
        }

        //���÷��͵���
        shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
        shoot_control.fric_motor[LEFT].give_current = shoot_control.fric_speed_pid[LEFT].out;
        shoot_control.fric_motor[RIGHT].give_current = shoot_control.fric_speed_pid[RIGHT].out;

    }
    
}

/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
  * @param[in]      void
  * @retval         vod
  */
static void shoot_set_mode(shoot_control_t *shoot_control)
{
    if ( shoot_control == NULL)
    {
        return;
    }   

    //���ģʽ�£��л��Զ����ƺ�ң��������
    if (switch_is_up(shoot_control->shoot_rc->rc.s[shoot_MODE_CHANNEL]))
    {    
        shoot_control->shoot_control_way = AUTO;
    }
    else if (switch_is_mid(shoot_control->shoot_rc->rc.s[shoot_MODE_CHANNEL]))
    {
        shoot_control->shoot_control_way = RC;
    }
    if(shoot_control->shoot_control_way == RC)
    {
        static int8_t last_s = RC_SW_UP;        //��¼��һ��ң��������ֵ

        //�ϲ��жϣ� һ�ο������ٴιر�
        if ((switch_is_up(shoot_control->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control->shoot_mode == SHOOT_STOP))
        {
            shoot_control->shoot_mode = SHOOT_READY_FRIC;
        }
        else if ((switch_is_up(shoot_control->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control->shoot_mode != SHOOT_STOP))
        {
            shoot_control->shoot_mode = SHOOT_STOP;    
        }

       

        shoot_control->shoot_last_key_v = shoot_control->shoot_rc->key.v;  




    

       // Ħ�����ٶȴﵽһ��ֵ,�ſɿ�������  Ϊ�˱��ڲ���,����������Ҫһ��Ħ���ֵ���ﵽ��������Ҫ��Ϳ��Կ�������
       if(shoot_control->shoot_mode == SHOOT_READY_FRIC &&(abs(shoot_control->fric_motor[LEFT].fric_motor_measure->speed_rpm)>abs(shoot_control->fric_motor[LEFT].require_speed) || abs(shoot_control->fric_motor[RIGHT].fric_motor_measure->speed_rpm)>abs(shoot_control->fric_motor[RIGHT].require_speed)))
       {
           shoot_control->fric_status = TRUE;     
            shoot_control->shoot_mode = SHOOT_READY_BULLET;
       }
         if(shoot_control->shoot_mode == SHOOT_READY_BULLET && shoot_control->key == SWITCH_TRIGGER_ON)
        {
            shoot_control->shoot_mode = SHOOT_READY;
        }
        else if(shoot_control->shoot_mode == SHOOT_READY && shoot_control->key == SWITCH_TRIGGER_OFF)
        {
            shoot_control->shoot_mode = SHOOT_READY_BULLET;
        }
        else if(shoot_control->shoot_mode == SHOOT_READY)
        {
            //�²�һ�λ�����갴��һ�Σ��������״̬
            if ((switch_is_down(shoot_control->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)) || IF_MOUSE_SINGAL_PRESSED_L )
            {
                shoot_control->shoot_mode = SHOOT_BULLET;
            }
        }
        else if(shoot_control->shoot_mode == SHOOT_DONE)
        {
            if(shoot_control->key == SWITCH_TRIGGER_OFF)
            {
                shoot_control->key_time++;
                if(shoot_control->key_time > SHOOT_DONE_KEY_OFF_TIME)
                {
                    shoot_control->key_time = 0;
                    shoot_control->shoot_mode = SHOOT_READY_BULLET;
                }
            }
            else
            {
                shoot_control->key_time = 0;
                shoot_control->shoot_mode = SHOOT_BULLET;
            }
        }
    
        
        //�������Ħ����ͬʱ���ߣ�Ϊ�˱��ڵ��ԣ���ʱע��
        // if(!toe_is_error(REFEREE_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.heat_limit))
        // {
        //     if(shoot_control.shoot_mode == SHOOT_BULLET || shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        //     {
        //         shoot_control.shoot_mode =SHOOT_READY_BULLET;
        //     }
        // }
        //�����̨״̬�� ����״̬���͹ر����
        if (gimbal_cmd_to_shoot_stop())
        {
            shoot_control->shoot_mode = SHOOT_STOP;
        }

        last_s = shoot_control->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
    }
    else if(shoot_control->shoot_control_way == AUTO)
    {
        shoot_control->shoot_mode = SHOOT_READY_FRIC;
        //Ħ�����ٶȴﵽһ��ֵ,�ſɿ�������  Ϊ�˱��ڲ���,����������Ҫһ��Ħ���ֵ���ﵽ��������Ҫ��Ϳ��Կ�������
        if(shoot_control->shoot_mode == SHOOT_READY_FRIC &&(abs(shoot_control->fric_motor[LEFT].fric_motor_measure->speed_rpm)>abs(shoot_control->fric_motor[LEFT].require_speed) || abs(shoot_control->fric_motor[RIGHT].fric_motor_measure->speed_rpm)>abs(shoot_control->fric_motor[RIGHT].require_speed)))
        {
            shoot_control->fric_status = TRUE;     
            shoot_control->shoot_mode = SHOOT_READY_BULLET;
        }
        else if(shoot_control->shoot_mode == SHOOT_READY_BULLET && shoot_control->key == SWITCH_TRIGGER_ON)
        {
            shoot_control->shoot_mode = SHOOT_READY;
        }
        else if(shoot_control->shoot_mode == SHOOT_READY && shoot_control->key == SWITCH_TRIGGER_OFF)
        {
            shoot_control->shoot_mode = SHOOT_READY_BULLET;
        }
        else if(shoot_control->shoot_mode == SHOOT_READY)
        {
            //ʶ��װ�װ��Ե�򵥷�
            if ( VisionRecvData_shoot.identify_target == TRUE&&VisionRecvData_shoot.centre_lock == FALSE )
            {
                shoot_control->shoot_mode = SHOOT_BULLET;
            }
        }
        else if(shoot_control->shoot_mode == SHOOT_DONE)
        {
            if(shoot_control->key == SWITCH_TRIGGER_OFF)
            {
                shoot_control->key_time++;
                if(shoot_control->key_time > SHOOT_DONE_KEY_OFF_TIME)
                {
                    shoot_control->key_time = 0;
                    shoot_control->shoot_mode = SHOOT_READY_BULLET;
                }
            }
            else
            {
                shoot_control->key_time = 0;
                shoot_control->shoot_mode = SHOOT_BULLET;
            }
        }
        if(shoot_control->shoot_mode > SHOOT_READY_FRIC)
        {
            //ʶ��װ�װ�����������
            if (VisionRecvData_shoot.identify_target == TRUE&&VisionRecvData_shoot.centre_lock == TRUE)
            {
                shoot_control->shoot_mode = SHOOT_CONTINUE_BULLET;
            }
            else if(shoot_control->shoot_mode == SHOOT_CONTINUE_BULLET)
            {
                shoot_control->shoot_mode =SHOOT_READY_BULLET;
            }
        }

    }

        

}

static void trigger_motor_turn_back(void)
{
    if( shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.speed_set = shoot_control.trigger_speed_set;
    }
    else
    {
        shoot_control.speed_set = -shoot_control.trigger_speed_set;
    }

    if(fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.block_time++;
        shoot_control.reverse_time = 0;
    }
    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time++;
    }
    else
    {
        shoot_control.block_time = 0;
    }
}

/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)
{

    //ÿ�β����ĽǶ�
    if (shoot_control.move_flag == 0)
    {
        shoot_control.set_angle = rad_format(shoot_control.angle + TRIGGER_ONCE);
        shoot_control.move_flag = 1;
    }
    if(shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode = SHOOT_DONE;
    }
    //����Ƕ��ж�
    if (rad_format(shoot_control.set_angle - shoot_control.angle) > 0.05f)
    {
        //û����һֱ������ת�ٶ�
        shoot_control.trigger_speed_set = shoot_grigger_grade[1] * SHOOT_TRIGGER_DIRECTION;
        trigger_motor_turn_back();
    }
    else
    {
        shoot_control.move_flag = 0;
        shoot_control.shoot_mode = SHOOT_READY;
    }
}

/**
  * @brief          ���ִ�ʱ,��̨Ҫֹͣ�˶�
  * @param[in]      none
  * @retval         1: no move 0:normal
  */

bool_t shoot_cmd_to_gimbal_stop(void)
{
    if (shoot_control.magazine_status == TRUE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}