  /**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      according to remote control, change the chassis behaviour.
  *             ����ң������ֵ������������Ϊ��
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================
        
    ���Ҫ���һ���µ���Ϊģʽ
    1.���ȣ���chassis_behaviour.h�ļ��У� ���һ������Ϊ������ chassis_behaviour_e
    erum
    {  
        ...
        ...
        CHASSIS_XXX_XXX, // ����ӵ�
    }chassis_behaviour_e,

    2. ʵ��һ���µĺ��� chassis_xxx_xxx_control(fp32 *vx, fp32 *vy, fp32 *wz, chassis_move_t * chassis )
        "vx,vy,wz" �����ǵ����˶�����������
        ��һ������: 'vx' ͨ�����������ƶ�,��ֵ ǰ���� ��ֵ ����
        �ڶ�������: 'vy' ͨ�����ƺ����ƶ�,��ֵ ����, ��ֵ ����
        ����������: 'wz' �����ǽǶȿ��ƻ�����ת�ٶȿ���
        ������µĺ���, ���ܸ� "vx","vy",and "wz" ��ֵ��Ҫ���ٶȲ���
    3.  ��"chassis_behaviour_mode_set"��������У�����µ��߼��жϣ���chassis_behaviour_mode��ֵ��CHASSIS_XXX_XXX
        �ں���������"else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)" ,Ȼ��ѡ��һ�ֵ��̿���ģʽ
        4��:
        CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW : 'vx' and 'vy'���ٶȿ��ƣ� 'wz'�ǽǶȿ��� ��̨�͵��̵���ԽǶ�
        �����������"xxx_angle_set"������'wz'
        CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW : 'vx' and 'vy'���ٶȿ��ƣ� 'wz'�ǽǶȿ��� ���̵������Ǽ�����ľ��ԽǶ�
        �����������"xxx_angle_set"
        CHASSIS_VECTOR_NO_FOLLOW_YAW : 'vx' and 'vy'���ٶȿ��ƣ� 'wz'����ת�ٶȿ���
        CHASSIS_VECTOR_RAW : ʹ��'vx' 'vy' and 'wz'ֱ�����Լ�������ֵĵ���ֵ������ֵ��ֱ�ӷ��͵�can ������
    4.  ��"chassis_behaviour_control_set" ������������
        else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)
        {
            chassis_xxx_xxx_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
        }
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */


#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "arm_math.h"
#include "referee.h"


#include "gimbal_behaviour.h"

/**
  * @brief          ������������Ϊ״̬���£�����ģʽ��raw���ʶ��趨ֵ��ֱ�ӷ��͵�can�����Ϲʶ����趨ֵ������Ϊ0
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      vy_set���ҵ��ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      wz_set��ת���ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_zero_force_control(fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);



/**
  * @brief          ���̲��ƶ�����Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set��ת���ٶȣ���ת�ٶ��ǿ��Ƶ��̵ĵ��̽��ٶ�
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_no_move_control(fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);


/**
  * @brief          �����Զ����Ƶ���Ϊ״̬����
  * @author         RM
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_no_follow_yaw_control(fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);


/**
  * @brief          ���̿�������Ϊ״̬���£�����ģʽ��rawԭ��״̬���ʶ��趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶȣ���ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set ��ת�ٶȣ� ��ֵ ��ʱ����ת����ֵ ˳ʱ����ת
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         none
  */

static void chassis_open_set_control(fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);


//���⣬���������Ϊģʽ����
chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
chassis_behaviour_e last_chassis_behaviour_mode = CHASSIS_NO_MOVE;

/**
  * @brief          ͨ���߼��жϣ���ֵ"chassis_behaviour_mode"������ģʽ
  * @param[in]      chassis_move_mode: ��������
  * @retval         none
  */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    last_chassis_behaviour_mode = chassis_behaviour_mode;

    //ң��������ģʽ
    if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {    
       chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
    }
    else if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
       chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
    }
    else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    }
    
    //����̨��ĳЩģʽ�»��ߵ��ִ򿪣����ʼ���� ���̲���
    if (gimbal_cmd_to_chassis_stop())
    {
        chassis_behaviour_mode = CHASSIS_NO_MOVE;
    }
    
    //����Լ����߼��жϽ�����ģʽ


    //������Ϊģʽѡ��һ�����̿���ģʽ
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW; 
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; 
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
    }
   
}



/**
  * @brief          ���ÿ�����.���ݲ�ͬ���̿���ģʽ��������������Ʋ�ͬ�˶�.������������棬����ò�ͬ�Ŀ��ƺ���.
  * @param[out]     vx_set, ͨ�����������ƶ�.
  * @param[out]     vy_set, ͨ�����ƺ����ƶ�.
  * @param[out]     wz_set, ͨ��������ת�˶�.
  * @param[in]      chassis_move_rc_to_vector,  ��������������Ϣ.
  * @retval         none
  */

void chassis_behaviour_control_set(fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{

    if (vy_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(vy_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_no_move_control(vy_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_no_follow_yaw_control(vy_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN)
    {
        chassis_open_set_control(vy_set, chassis_move_rc_to_vector);
    }
    
}


/**
  * @brief          ������������Ϊ״̬���£�����ģʽ��raw���ʶ��趨ֵ��ֱ�ӷ��͵�can�����Ϲʶ����趨ֵ������Ϊ0
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      vy_set���ҵ��ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      wz_set��ת���ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */

static void chassis_zero_force_control(fp32 *vy_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vy_can_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    *vy_can_set = 0.0f;

}


/**
  * @brief          ���̲��ƶ�����Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set��ת���ٶȣ���ת�ٶ��ǿ��Ƶ��̵ĵ��̽��ٶ�
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */

static void chassis_no_move_control(fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vy_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    *vy_set = 0.0f;

}
int sj_count =1000;
int sj_flag =0;

/**
  * @brief          ���̸������yaw����Ϊ״̬���£�����ģʽ�Ǹ�����̽Ƕȣ�������ת�ٶȻ���ݽǶȲ���������ת�Ľ��ٶ�
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      angle_set�������õ�yaw����Χ -PI��PI
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */

static void chassis_no_follow_yaw_control(fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    
    if (vy_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }   

    //���̲�����yawģʽ�£��л��Զ����ƺ�ң��������
    if (switch_is_up(chassis_move_rc_to_vector->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {    
        chassis_move_rc_to_vector->chassis_control_way = AUTO;
    }
    else if (switch_is_mid(chassis_move_rc_to_vector->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        chassis_move_rc_to_vector->chassis_control_way = RC;
    }


    //ң��������
    if(chassis_move_rc_to_vector->chassis_control_way == RC)
    {
        //ң������ͨ��ֵ�Լ����̰��� �ó� һ������µ��ٶ��趨ֵ
        chassis_rc_to_control_vector(vy_set, chassis_move_rc_to_vector);
    }
    else if(chassis_move_rc_to_vector->chassis_control_way == AUTO)  //�Զ��������
    {
        output_state();
        if(field_event_outpost == 1){//ǰ��վ���,ͣ���ұ�
            if(chassis_move_rc_to_vector->left_light_sensor == TRUE && chassis_move_rc_to_vector->right_light_sensor == TRUE)
            {
                chassis_move_rc_to_vector->direction = NO_MOVE;
            }
            else if(chassis_move_rc_to_vector->left_light_sensor == FALSE && chassis_move_rc_to_vector->right_light_sensor == TRUE)
            {
                chassis_move_rc_to_vector->direction = NO_MOVE;
            }
            else if(chassis_move_rc_to_vector->left_light_sensor == TRUE && chassis_move_rc_to_vector->right_light_sensor == FALSE)
            {
                chassis_move_rc_to_vector->direction = RIGHT;
            }
            else if(chassis_move_rc_to_vector->left_light_sensor == FALSE && chassis_move_rc_to_vector->right_light_sensor == FALSE)
            {
                chassis_move_rc_to_vector->direction =chassis_move_rc_to_vector->direction ;
            }

        }
        if(field_event_outpost == 0){//ǰ��վ�����٣���ʼѲ��
            //���̻���Ѳ�߹켣
            /*
            ���ʶ�� �ұ�ʶ��    ��ֹ����
            ���δʶ�� �ұ�ʶ��  ��������
            ���ʶ�� �ұ�δʶ��  ��������
            ���δʶ�� �ұ�δʶ�� ����ԭ״̬
            */
            if(chassis_move_rc_to_vector->left_light_sensor == TRUE && chassis_move_rc_to_vector->right_light_sensor == TRUE)
            {
                chassis_move_rc_to_vector->direction = NO_MOVE;
            }
            else if(chassis_move_rc_to_vector->left_light_sensor == FALSE && chassis_move_rc_to_vector->right_light_sensor == TRUE)
            {
                chassis_move_rc_to_vector->direction = LEFT;
            }
            else if(chassis_move_rc_to_vector->left_light_sensor == TRUE && chassis_move_rc_to_vector->right_light_sensor == FALSE)
            {
                chassis_move_rc_to_vector->direction = RIGHT;
            }
            else if(chassis_move_rc_to_vector->left_light_sensor == FALSE && chassis_move_rc_to_vector->right_light_sensor == FALSE)
            {
                chassis_move_rc_to_vector->direction = chassis_move_rc_to_vector->direction;
                if(sj_count>0)
                {
                    sj_count--;
                    if(sj_count==0)
                    {
                        sj_flag=1;
                        if(chassis_move_rc_to_vector->direction == RIGHT)
                        {
                            chassis_move_rc_to_vector->direction = LEFT;
                        }
                        else if(chassis_move_rc_to_vector->direction == LEFT)
                        {
                            chassis_move_rc_to_vector->direction = RIGHT;
                        }
                    }            
                }
                if(sj_flag==1)
                {
                    sj_flag=0;
                    sj_count=1000;
                }   
            }
        }


        //���ݲ�ͬ��������ٶȵȼ�
        if(if_hit())
        {
            *vy_set = CHASSIS_HIGH_SPEED;
        }
        else
        {
            *vy_set = CHASSIS_MID_SPEED;
        }

        //��ֹ��������ȫ����
        //if()


        //���ݷ����������
        if(chassis_move_rc_to_vector->direction == LEFT)
             *vy_set = *vy_set;
        else if(chassis_move_rc_to_vector->direction == RIGHT)
             *vy_set = -*vy_set;
        else if(chassis_move_rc_to_vector->direction == NO_MOVE)
             *vy_set = 0;
    }
   
}


/**
  * @brief          ���̿�������Ϊ״̬���£�����ģʽ��rawԭ��״̬���ʶ��趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶȣ���ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set ��ת�ٶȣ� ��ֵ ��ʱ����ת����ֵ ˳ʱ����ת
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         none
  */

static void chassis_open_set_control(fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vy_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    *vy_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    return;
}


