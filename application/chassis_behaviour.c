  /**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      according to remote control, change the chassis behaviour.
  *             根据遥控器的值，决定底盘行为。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================
        
    如果要添加一个新的行为模式
    1.首先，在chassis_behaviour.h文件中， 添加一个新行为名字在 chassis_behaviour_e
    erum
    {  
        ...
        ...
        CHASSIS_XXX_XXX, // 新添加的
    }chassis_behaviour_e,

    2. 实现一个新的函数 chassis_xxx_xxx_control(fp32 *vx, fp32 *vy, fp32 *wz, chassis_move_t * chassis )
        "vx,vy,wz" 参数是底盘运动控制输入量
        第一个参数: 'vx' 通常控制纵向移动,正值 前进， 负值 后退
        第二个参数: 'vy' 通常控制横向移动,正值 左移, 负值 右移
        第三个参数: 'wz' 可能是角度控制或者旋转速度控制
        在这个新的函数, 你能给 "vx","vy",and "wz" 赋值想要的速度参数
    3.  在"chassis_behaviour_mode_set"这个函数中，添加新的逻辑判断，给chassis_behaviour_mode赋值成CHASSIS_XXX_XXX
        在函数最后，添加"else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)" ,然后选择一种底盘控制模式
        4种:
        CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW : 'vx' and 'vy'是速度控制， 'wz'是角度控制 云台和底盘的相对角度
        你可以命名成"xxx_angle_set"而不是'wz'
        CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW : 'vx' and 'vy'是速度控制， 'wz'是角度控制 底盘的陀螺仪计算出的绝对角度
        你可以命名成"xxx_angle_set"
        CHASSIS_VECTOR_NO_FOLLOW_YAW : 'vx' and 'vy'是速度控制， 'wz'是旋转速度控制
        CHASSIS_VECTOR_RAW : 使用'vx' 'vy' and 'wz'直接线性计算出车轮的电流值，电流值将直接发送到can 总线上
    4.  在"chassis_behaviour_control_set" 函数的最后，添加
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
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
  * @author         RM
  * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
  * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
  * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_zero_force_control(fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);



/**
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_no_move_control(fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);


/**
  * @brief          底盘自动控制的行为状态机下
  * @author         RM
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_no_follow_yaw_control(fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);


/**
  * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
  * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         none
  */

static void chassis_open_set_control(fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);


//留意，这个底盘行为模式变量
chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
chassis_behaviour_e last_chassis_behaviour_mode = CHASSIS_NO_MOVE;

/**
  * @brief          通过逻辑判断，赋值"chassis_behaviour_mode"成哪种模式
  * @param[in]      chassis_move_mode: 底盘数据
  * @retval         none
  */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    last_chassis_behaviour_mode = chassis_behaviour_mode;

    //遥控器设置模式
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
    
    //当云台在某些模式下或者弹仓打开，像初始化， 底盘不动
    if (gimbal_cmd_to_chassis_stop())
    {
        chassis_behaviour_mode = CHASSIS_NO_MOVE;
    }
    
    //添加自己的逻辑判断进入新模式


    //根据行为模式选择一个底盘控制模式
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
  * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
  * @param[out]     vx_set, 通常控制纵向移动.
  * @param[out]     vy_set, 通常控制横向移动.
  * @param[out]     wz_set, 通常控制旋转运动.
  * @param[in]      chassis_move_rc_to_vector,  包括底盘所有信息.
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
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
  * @author         RM
  * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
  * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
  * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
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
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
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
  * @brief          底盘跟随底盘yaw的行为状态机下，底盘模式是跟随底盘角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      angle_set底盘设置的yaw，范围 -PI到PI
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_no_follow_yaw_control(fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    
    if (vy_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }   

    //底盘不跟随yaw模式下，切换自动控制和遥控器控制
    if (switch_is_up(chassis_move_rc_to_vector->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {    
        chassis_move_rc_to_vector->chassis_control_way = AUTO;
    }
    else if (switch_is_mid(chassis_move_rc_to_vector->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        chassis_move_rc_to_vector->chassis_control_way = RC;
    }


    //遥控器控制
    if(chassis_move_rc_to_vector->chassis_control_way == RC)
    {
        //遥控器的通道值以及键盘按键 得出 一般情况下的速度设定值
        chassis_rc_to_control_vector(vy_set, chassis_move_rc_to_vector);
    }
    else if(chassis_move_rc_to_vector->chassis_control_way == AUTO)  //自动程序控制
    {
        output_state();
        if(field_event_outpost == 1){//前哨站存活,停在右边
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
        if(field_event_outpost == 0){//前哨站被击毁，开始巡逻
            //底盘基础巡逻轨迹
            /*
            左边识别 右边识别    静止不动
            左边未识别 右边识别  方向向左
            左边识别 右边未识别  方向向右
            左边未识别 右边未识别 保持原状态
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


        //根据不同情况设置速度等级
        if(if_hit())
        {
            *vy_set = CHASSIS_HIGH_SPEED;
        }
        else
        {
            *vy_set = CHASSIS_MID_SPEED;
        }

        //防止缓冲能量全用完
        //if()


        //根据方向设置输出
        if(chassis_move_rc_to_vector->direction == LEFT)
             *vy_set = *vy_set;
        else if(chassis_move_rc_to_vector->direction == RIGHT)
             *vy_set = -*vy_set;
        else if(chassis_move_rc_to_vector->direction == NO_MOVE)
             *vy_set = 0;
    }
   
}


/**
  * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
  * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      chassis_move_rc_to_vector底盘数据
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


