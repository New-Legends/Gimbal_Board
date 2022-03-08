/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             ���̿�������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "gimbal_task.h"
#include "cmsis_os.h"

#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "referee_control.h"



#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }


/**
  * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     chassis_move_init:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init);


/**
  * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
  * @param[out]     chassis_move_mode:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode);


/**
  * @brief          ����ģʽ�ı䣬��Щ������Ҫ�ı䣬������̿���yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰ����yaw�Ƕ�
  * @param[out]     chassis_move_transit:"chassis_move"����ָ��.
  * @retval         none
  */
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);

/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update);

/**
  * @brief          
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control);

/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

//�����˶�����
chassis_move_t chassis_move;


/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
 
void chassis_task(void const *pvParameters)
{
    //����һ��ʱ��
    vTaskDelay(CHASSIS_TASK_INIT_TIME);

    //���̳�ʼ��
    chassis_init(&chassis_move);
    //�жϵ��̵���Ƿ�����
    while (toe_is_error(CHASSIS_MOTOR_TOE))
    {
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }

    while (1)
    {
        //���õ��̿���ģʽ
        chassis_set_mode(&chassis_move);
        //ģʽ�л����ݱ���
        chassis_mode_change_control_transit(&chassis_move);
        //�������ݸ���
        chassis_feedback_update(&chassis_move);
        //���̿���������
        chassis_set_contorl(&chassis_move);
        //���̿���PID����
        chassis_control_loop(&chassis_move);

        //���͵����淢������������һ���� ��shoot_task������

        //ϵͳ��ʱ
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


/**
  * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     chassis_move_init:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    //�����ٶȻ�pidֵ
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    //���������˲�
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    uint8_t i;

    //���̿���״̬Ϊԭʼ
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    //��ȡң����ָ��
    chassis_move_init->chassis_RC = get_remote_control_point();
    //��ȡ��������̬��ָ��
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    //��ȡ��̨�������ָ��
    // chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
    // chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
    
    //��ȡ���̵������ָ�룬��ʼ��PID 
    chassis_move_init->motor_chassis.chassis_motor_measure = get_chassis_motor_measure_point();
    PID_init(&chassis_move_init->motor_speed_pid, PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);

    //��һ���˲�����б����������
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    //��� ��С�ٶ�
    chassis_move_init->max_wheel_speed = MAX_WHEEL_SPEED;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //��¼��һ�μ���ֵ
    chassis_move_init->chassis_last_key_v = 0;

    //���õ��̳�ʼ���Ʒ�ʽ
    chassis_move_init->chassis_control_way = RC;

    //Ĭ�ϳ�ʼ״̬����Ϊʶ�𵽣���ʼ���������˶�
    chassis_move_init->left_light_sensor = 0;
    chassis_move_init->right_light_sensor = 0;
    chassis_move_init->direction = LEFT;

    //update data
    //����һ������
    chassis_feedback_update(chassis_move_init);
}


/**
  * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
  * @param[out]     chassis_move_mode:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    //in file "chassis_behaviour.c"
    chassis_behaviour_mode_set(chassis_move_mode);
}


/**
  * @brief          ����ģʽ�ı䣬��Щ������Ҫ�ı䣬������̿���yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰ����yaw�Ƕ�
  * @param[out]     chassis_move_transit:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    chassis_move_update->motor_chassis.speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis.chassis_motor_measure->speed_rpm;

    //���µ���ƽ���ٶ�y
    chassis_move_update->vy = chassis_move_update->motor_chassis.speed * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;

    //���������̬�Ƕ�, �����������������������ⲿ�ִ���
    chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) - chassis_move_update->chassis_yaw_motor->relative_angle);
    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);
    chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
    

    //���¹�紫��������
    chassis_move_update->left_light_sensor = !(HAL_GPIO_ReadPin(left_light_sensor_GPIO_Port, left_light_sensor_Pin));
    chassis_move_update->right_light_sensor = !(HAL_GPIO_ReadPin(right_light_sensor_GPIO_Port, right_light_sensor_Pin));

    //�����紫��������,ֹͣ�����˶�            ��ʱδ���
    //if 
}

/**
  * @brief          ����ң����ͨ��ֵ����������ͺ����ٶ�
  *                 
  * @param[out]     vx_set: �����ٶ�ָ��
  * @param[out]     vy_set: �����ٶ�ָ��
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
  * @retval         none
  */
void chassis_rc_to_control_vector(fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vy_set == NULL)
    {
        return;
    }
    
    int16_t vy_channel;
    fp32 vy_set_channel;
    //�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

    //���̿���
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
    }

    //һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
    //ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}

/**
  * @brief          ���õ��̿�������ֵ, ���˶�����ֵ��ͨ��chassis_behaviour_control_set�������õ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }

    fp32 vy_set = 0.0f;
    //��ȡ������������ֵ
    chassis_behaviour_control_set(&vy_set, chassis_move_control);

    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        //��angle_set�� ����ת�ٶȿ���
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        //��ԭʼģʽ������ֵ�Ƿ��͵�CAN����
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    }
}




/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{

    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed = 0.0f;
    wheel_speed = chassis_move_control_loop->vy_set;


    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        chassis_move_control_loop->motor_chassis.give_current = (int16_t)(wheel_speed);

        //raw����ֱ�ӷ���
        return;
    }
    
    //�������ӿ�������ٶȣ�������������ٶ�
    chassis_move_control_loop->motor_chassis.speed_set = wheel_speed;
    temp = fabs(chassis_move_control_loop->motor_chassis.speed_set);
    if (max_vector < temp)
    {
        max_vector = temp;
    }


    if (max_vector > chassis_move_control_loop->max_wheel_speed)
    {
        vector_rate = chassis_move_control_loop->max_wheel_speed / max_vector;
        chassis_move_control_loop->motor_chassis.speed_set *= vector_rate; 
    }

    //����pid
    PID_calc(&chassis_move_control_loop->motor_speed_pid, chassis_move_control_loop->motor_chassis.speed, chassis_move_control_loop->motor_chassis.speed_set);
    
    //���ʿ���
    //chassis_power_control(chassis_move_control_loop);

    //��ֵ����ֵ
    chassis_move_control_loop->motor_chassis.give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid.out);
    
}
