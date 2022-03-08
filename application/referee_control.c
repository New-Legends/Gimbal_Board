/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_control.c/h
  * @brief      chassis power control.底盘功率控制
  * @note       this is only controling 80 w power, mainly limit motor current set.
  *             if power limit is 40w, reduce the value JUDGE_TOTAL_CURRENT_LIMIT 
  *             and POWER_CURRENT_LIMIT, and chassis max speed (include max_vx_speed, min_vx_speed)
  *             只控制80w功率，主要通过控制电机电流设定值,如果限制功率是40w，减少
  *             JUDGE_TOTAL_CURRENT_LIMIT和POWER_CURRENT_LIMIT的值，还有底盘最大速度
  *             (包括max_vx_speed, min_vx_speed)
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "referee_control.h"
#include "referee.h"
#include "arm_math.h"
#include "detect_task.h"




#define POWER_LIMIT         80.0f
#define WARNING_POWER       40.0f   
#define WARNING_POWER_BUFF  50.0f   

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4, 
#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f



/*
17mm射速上限 15 18 30 m/s
17mm热量上限 50 100 150 280 400
17mm热量冷却 10 20 30 40 60 80
一发17mm 10热量

42mm射速上限 10 16 m/s
42mm热量上限 100 200 300 350 500
42mm热量冷却 20 40 60 80 100 120
一发42mm 100热量
*/

#define FRIC_REFEREE_PARA  0.5            //射速裁判规定数值转实际输入
#define GRIGGER_SPEED_TO_RADIO  0.8      //射频裁判规定数值转实际输入


//通过读取裁判数据,直接修改射速和射频等级
//射速等级  摩擦电机
fp32 shoot_fric_grade[4] = {0, 15*FRIC_REFEREE_PARA, 18*FRIC_REFEREE_PARA, 30*FRIC_REFEREE_PARA};

//射频等级 拨弹电机
fp32 shoot_grigger_grade[6] = {0, 5.0f*GRIGGER_SPEED_TO_RADIO, 10.0f*GRIGGER_SPEED_TO_RADIO, 15.0f*GRIGGER_SPEED_TO_RADIO, 28.0f*GRIGGER_SPEED_TO_RADIO, 40.0f*GRIGGER_SPEED_TO_RADIO};

 //拨盘等级 摩擦轮等级
uint8_t grigger_speed_grade;
uint8_t fric_speed_grade;

/**
  * @brief          限制功率，主要限制电机电流
  * @param[in]      chassis_power_control: 底盘数据
  * @retval         none
  */
void chassis_power_control(chassis_move_t *chassis_power_control)
{
    fp32 chassis_power = 0.0f;
    fp32 chassis_power_buffer = 0.0f;
    fp32 total_current_limit = 0.0f;
    fp32 total_current = 0.0f;
    uint8_t robot_id = get_robot_id();
    if(toe_is_error(REFEREE_TOE))
    {
        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
    }
    else if(robot_id == RED_ENGINEER || robot_id == BLUE_ENGINEER || robot_id == 0)
    {
        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
    }
    else
    {
        get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);
        //功率超过80w 和缓冲能量小于60j,因为缓冲能量小于60意味着功率超过80w
        if(chassis_power_buffer < WARNING_POWER_BUFF)
        {
            fp32 power_scale;
            if(chassis_power_buffer > 5.0f)
            {
                //缩小WARNING_POWER_BUFF
                power_scale = chassis_power_buffer / WARNING_POWER_BUFF;
            }
            else
            {
                //only left 10% of WARNING_POWER_BUFF
                power_scale = 5.0f / WARNING_POWER_BUFF;
            }
            //缩小
            total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
        }
        else
        {
            //功率大于WARNING_POWER
            if(chassis_power > WARNING_POWER)
            {
                fp32 power_scale;
                //功率小于80w
                if(chassis_power < POWER_LIMIT)
                {
                    //scale down
                    //缩小
                    power_scale = (POWER_LIMIT - chassis_power) / (POWER_LIMIT - WARNING_POWER);
                    
                }
                //功率大于80w
                else
                {
                    power_scale = 0.0f;
                }
                
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
            }
            //功率小于WARNING_POWER
            else
            {
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
            }
        }
    }

    
    total_current = 0.0f;
    //计算原本电机电流设定
    total_current += fabs(chassis_power_control->motor_speed_pid.out);
   
    

    if(total_current > total_current_limit)
    {
        fp32 current_scale = total_current_limit / total_current;
        chassis_power_control->motor_speed_pid.out*=current_scale;
    }
}

//17mm枪口热量上限, 17mm枪口实时热量
uint16_t id1_17mm_cooling_limit;
uint16_t id1_17mm_cooling_heat;
//17mm枪口枪口射速上限,17mm实时射速
uint16_t id1_17mm_speed_limit; 
fp32 bullet_speed;
/**
  * @brief          限制17mm发射机构射速和射频，主要限制电机电流 默认枪口ID为1,如果需要ID2,自行修改和添加
  * @param[in]      shoot_heat0_speed_and_cooling_control: 发送机构数据
  * @retval         none
  */
 void shoot_id1_17mm_speed_and_cooling_control(shoot_control_t *shoot_heat0_speed_and_cooling_control)
{

    if(toe_is_error(REFEREE_TOE))
    {
        grigger_speed_grade = 1;
        fric_speed_grade = 1;
    }
    else
    {
        get_shooter_id1_17mm_cooling_limit_and_heat(&id1_17mm_cooling_limit,&id1_17mm_cooling_heat);   //获取17mm枪口热量上限, 17mm枪口实时热量
        get_shooter_id1_17mm_speed_limit_and_bullet_speed(&id1_17mm_speed_limit, &bullet_speed); // 获取17mm枪口枪口射速上限,17mm实时射速

        //根据热量和射速上限修改等级
        //热量
        if(id1_17mm_cooling_limit <= 50)
            grigger_speed_grade = 1;
        else if(id1_17mm_cooling_limit <= 100)
            grigger_speed_grade = 2;
        else if(id1_17mm_cooling_limit <= 150)
            grigger_speed_grade = 3;
        else if(id1_17mm_cooling_limit <= 280)
            grigger_speed_grade = 4;
        else if(id1_17mm_cooling_limit <= 400)
            grigger_speed_grade = 5;

        //射速
        if(id1_17mm_speed_limit <= 15)
            fric_speed_grade = 1;
        else if(id1_17mm_speed_limit <= 18)
            fric_speed_grade = 2;
        else if(id1_17mm_speed_limit <= 30)
            fric_speed_grade = 3;


        //当调试射速和射频等级数组时可以暂时注释
        //根据当前热量和射速修改等级,确保不会因超限扣血,

        //热量 当剩余热量低于30,强制制动
        if(id1_17mm_cooling_limit - id1_17mm_cooling_heat <= 20 && grigger_speed_grade!=0)
            grigger_speed_grade = 0 ;

        
        //射速 超射速,强制降低摩擦轮转速
        if(bullet_speed > id1_17mm_speed_limit)
            fric_speed_grade -- ;

    }

    //对拨盘电机输入控制值
    shoot_heat0_speed_and_cooling_control->trigger_speed_set = shoot_grigger_grade[grigger_speed_grade] * SHOOT_TRIGGER_DIRECTION;
    //对摩擦轮电机输入控制值
    shoot_heat0_speed_and_cooling_control->fric_motor[LEFT].speed_set = -shoot_fric_grade[fric_speed_grade];
    shoot_heat0_speed_and_cooling_control->fric_motor[RIGHT].speed_set = shoot_fric_grade[fric_speed_grade];
   
}