#include "Shoot.h"

#include "main.h"

#include "bsp_fric.h"
#include "user_lib.h"

#ifdef __cplusplus //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
extern "C"
{
#include "bsp_laser.h"
}
#endif


#include "Communicate.h"
#include "detect_task.h"

#define shoot_fric1_on(pwm) fric1_on((pwm)) //摩擦轮1pwm宏定义
#define shoot_fric2_on(pwm) fric2_on((pwm)) //摩擦轮2pwm宏定义
#define shoot_fric_off() fric_off()         //关闭两个摩擦轮

#define shoot_laser_on() laser_on()   //激光开启宏定义
#define shoot_laser_off() laser_off() //激光关闭宏定义
//微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)



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

// //便于测试,把值调低
// #define FRIC_REFEREE_PARA 0.1      //射速裁判规定数值转实际输入
// #define GRIGGER_SPEED_TO_RADIO 0.2 //射频裁判规定数值转实际输入

//通过读取裁判数据,直接修改射速和射频等级
//射速等级  摩擦电机
fp32 shoot_fric_grade[4] = {0, 15*FRIC_REFEREE_PARA, 18*FRIC_REFEREE_PARA, 30*FRIC_REFEREE_PARA};

//射频等级 拨弹电机
fp32 shoot_grigger_grade[6] = {0, 5.0f*GRIGGER_SPEED_TO_RADIO, 10.0f*GRIGGER_SPEED_TO_RADIO, 15.0f*GRIGGER_SPEED_TO_RADIO, 28.0f*GRIGGER_SPEED_TO_RADIO, 40.0f*GRIGGER_SPEED_TO_RADIO};

//拨盘等级 摩擦轮等级
uint8_t grigger_speed_grade;
uint8_t fric_speed_grade;

Shoot shoot;

/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @param[in]      void
  * @retval         返回空
  */
void Shoot::init()
{
    //遥控器指针
    shoot_rc = remote_control.get_remote_control_point();
    last_shoot_rc = remote_control.get_last_remote_control_point();

    //记录上一次按键值
    shoot_last_key_v = 0;

    //设置初试模式
    shoot_mode = SHOOT_STOP;

    //摩擦轮电机
    for(int i = 0; i < 2; ++i)
    {
        //获取电机数据
        fric_motor[i].init(can_receive.get_shoot_motor_measure_point(i));
        //初始化PID
        fp32 fric_speed_pid_parm[5] = {FRIC_SPEED_PID_KP, FRIC_SPEED_PID_KI, FRIC_SPEED_PID_KD, FRIC_PID_MAX_IOUT, FRIC_PID_MAX_OUT};
        fric_motor[i].speed_pid.init(PID_SPEED, fric_speed_pid_parm, &fric_motor[i].speed, &fric_motor[i].speed_set, NULL);
        fric_motor[i].speed_pid.pid_clear();

        //设置最大 最小值  左摩擦轮顺时针转 右摩擦轮逆时针转
        fric_motor[i].max_speed = FRIC_MAX_SPEED;
        fric_motor[i].min_speed = -FRIC_MAX_SPEED;
        fric_motor[i].require_speed = -FRIC_MAX_REQUUIRE_SPEED;
    }

    trigger_motor.init(can_receive.get_shoot_motor_measure_point(TRIGGER));
    //初始化PID
    fp32 trigger_speed_pid_parm[5] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD, TRIGGER_BULLET_PID_MAX_IOUT, TRIGGER_BULLET_PID_MAX_OUT};
    trigger_motor.speed_pid.init(PID_SPEED, trigger_speed_pid_parm, &trigger_motor.speed, &trigger_motor.speed_set, NULL);
    trigger_motor.angle_pid.pid_clear();
    // //TODO,此处限幅,暂时不设置
    // //设置最大 最小值  左摩擦轮顺时针转 右摩擦轮逆时针转
    // trigger_motor.max_speed = FRIC_MAX_SPEED_RMP;
    // trigger_motor.min_speed = -FRIC_MAX_SPEED_RMP;
    // trigger_motor.require_speed = -FRIC_REQUIRE_SPEED_RMP;

    //摩擦轮,弹仓舵机,限位舵机状态
    fric_status = FALSE;
    cover_status = FALSE;
    limit_switch_status = FALSE;


    //更新数据
    feedback_update();

    //TODO 此处先添加,后面可能会删去
    trigger_motor.angle = trigger_motor.motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
	trigger_motor.ecd_count = 0;
    trigger_motor.current_give = 0;
    trigger_motor.angle_set = trigger_motor.angle;
    trigger_motor.speed = 0.0f;
    trigger_motor.speed_set = 0.0f;
		
    move_flag = 0;
    key_time = 0;
}

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
bool_t temp_a;
bool_t temp_b;
bool_t temp_c;

void Shoot::set_mode()
{
    static int8_t last_s = RC_SW_UP; //记录上一次遥控器按键值

    //上拨判断， 一次开启，再次关闭
    if ((switch_is_up(shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_mode == SHOOT_STOP))
    {
        shoot_mode = SHOOT_READY_FRIC;
    }
    else if ((switch_is_up(shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_mode != SHOOT_STOP))
    {
        shoot_mode = SHOOT_STOP;
    }

    temp_a = if_key_pessed(shoot_rc, 'G');
    temp_b = if_key_pessed(last_shoot_rc, 'G');
    temp_c = KEY_FRIC;

    //处于中档， 可以使用键盘开启摩擦轮
    if (switch_is_mid(shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && KEY_FRIC && shoot_mode == SHOOT_STOP)
    {
        shoot_mode = SHOOT_READY_FRIC;
    }
    //处于中档， 可以使用键盘关闭摩擦轮
    else if (switch_is_mid(shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && KEY_FRIC && shoot_mode != SHOOT_STOP)
    {
        shoot_mode = SHOOT_STOP;
    }

    //为了便于测试,右按键为下时关闭摩擦轮
    if (switch_is_down(shoot_rc->rc.s[0]))
    {
        shoot_mode = SHOOT_STOP;
    }





    //TODO 这里为了赶进度,对于abs函数使用了一种非常粗糙的方法完成,因为使用库函数abs编译器无法匹配正确的重载;后续需要修改
    //摩擦轮速度达到一定值,才可开启拨盘  为了便于测试,这里至少需要一个摩擦轮电机达到拨盘启动要求就可以开启拨盘
    if (shoot_mode == SHOOT_READY_FRIC && (abs_int16(fric_motor[LEFT_FRIC].motor_measure->speed_rpm) > abs_fp32(fric_motor[LEFT_FRIC].require_speed) || abs_int16(fric_motor[RIGHT_FRIC].motor_measure->speed_rpm) > abs_fp32(fric_motor[RIGHT_FRIC].require_speed)))
    {
        fric_status = TRUE;
        shoot_mode = SHOOT_READY_BULLET;
    }
    else if (shoot_mode == SHOOT_READY_BULLET && key == SWITCH_TRIGGER_ON)
    {
        shoot_mode = SHOOT_READY;
    }
    else if (shoot_mode == SHOOT_READY && key == SWITCH_TRIGGER_OFF)
    {
        shoot_mode = SHOOT_READY_BULLET;
    }
    else if (shoot_mode == SHOOT_READY)
    {
        //下拨一次或者鼠标按下一次，进入射击状态
        if ((switch_is_down(shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)) || if_mouse_pessed(shoot_rc, 'L'))
        {
            shoot_mode = SHOOT_BULLET;
        }
    }
    else if (shoot_mode == SHOOT_DONE)
    {
        if (key == SWITCH_TRIGGER_OFF)
        {
            key_time++;
            if (key_time > SHOOT_DONE_KEY_OFF_TIME)
            {
                key_time = 0;
                shoot_mode = SHOOT_READY_BULLET;
            }
        }
        else
        {
            key_time = 0;
            shoot_mode = SHOOT_BULLET;
        }
    }

    if (shoot_mode > SHOOT_READY_FRIC)
    {
        //鼠标长按一直进入射击状态 保持连发
        if ((press_l_time == PRESS_LONG_TIME) || (press_r_time == PRESS_LONG_TIME) || (rc_s_time == RC_S_LONG_TIME))
        {
            shoot_mode = SHOOT_CONTINUE_BULLET;
        }
        else if (shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            shoot_mode = SHOOT_READY_BULLET;
        }
    }

    // //检测两个摩擦轮同时上线，为了便于调试，暂时注释
    // if(!toe_is_error(REFEREE_TOE) && (heat + SHOOT_HEAT_REMAIN_VALUE > heat_limit))
    // {
    //     if(shoot_mode == SHOOT_BULLET || shoot_mode == SHOOT_CONTINUE_BULLET)
    //     {
    //         shoot_mode =SHOOT_READY_BULLET;
    //     }
    // }

    //TODO此处还未更新
    // //如果云台状态是 无力状态，就关闭射击
    // if (gimbal_cmd_to_shoot_stop())
    // {
    //     shoot_mode = SHOOT_STOP;
    // }

    last_s = shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}





/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
void Shoot::feedback_update()
{
    shoot_last_key_v = shoot_rc->key.v;
    
    //更新摩擦轮电机速度
    fric_motor[LEFT_FRIC].speed = -fric_motor[LEFT_FRIC].motor_measure->speed_rpm * FRIC_RPM_TO_SPEED;
    fric_motor[RIGHT_FRIC].speed = fric_motor[RIGHT_FRIC].motor_measure->speed_rpm * FRIC_RPM_TO_SPEED;

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    // 拨弹轮电机速度滤波一下
	static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
    //二阶低通滤波
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (trigger_motor.motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    trigger_motor.speed = speed_fliter_3;

    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    if (trigger_motor.motor_measure->ecd - trigger_motor.motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        trigger_motor.ecd_count--;
    }
    else if (trigger_motor.motor_measure->ecd - trigger_motor.motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        trigger_motor.ecd_count++;
    }

    if (trigger_motor.ecd_count == FULL_COUNT)
    {
        trigger_motor.ecd_count = -(FULL_COUNT - 1);
    }
    else if (trigger_motor.ecd_count == -FULL_COUNT)
    {
        trigger_motor.ecd_count = FULL_COUNT - 1;
    }

    //计算输出轴角度
    trigger_motor.angle = (trigger_motor.ecd_count * ECD_RANGE + trigger_motor.motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
    //TODO 此处没有安装微动开关,暂时把key设置为1
    //微动开关 
    key = 0;
    //鼠标按键
    last_press_l = press_l;
    last_press_r = press_r;
    press_l = shoot_rc->mouse.press_l;
    press_r = shoot_rc->mouse.press_r;


    //长按计时
    if (press_l)
    {
        if (press_l_time < PRESS_LONG_TIME)
        {
            press_l_time++;
        }
    }
    else
    {
        press_l_time = 0;
    }

    //射击开关下档时间计时
    if (shoot_mode != SHOOT_STOP && switch_is_down(shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {

        if (rc_s_time < RC_S_LONG_TIME)
        {
            rc_s_time++;
        }
    }
    else
    {
        rc_s_time = 0;
    }

}

/**
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
  */
void Shoot::set_control()
{
    if (shoot_mode == SHOOT_STOP)
    {
        //设置拨弹轮的速度
        trigger_motor.speed_set = 0.0f;
    }
    else if (shoot_mode == SHOOT_READY_FRIC)
    {
        //设置拨弹轮的速度
        trigger_motor.speed_set = 0.0f;
    }
    else if (shoot_mode == SHOOT_READY_BULLET)
    {
        if (key == SWITCH_TRIGGER_OFF)
        {
            //设置拨弹轮的拨动速度,并开启堵转反转处理
            trigger_motor.speed_set = shoot_grigger_grade[1] * SHOOT_TRIGGER_DIRECTION;
        }
        else
        {
            trigger_motor.speed_set = 0.0f;
        }
        trigger_motor.speed_pid.data.max_out = TRIGGER_READY_PID_MAX_OUT;
        trigger_motor.speed_pid.data.max_iout = TRIGGER_READY_PID_MAX_IOUT;
    }
    else if (shoot_mode == SHOOT_READY)
    {
        //设置拨弹轮的速度
        trigger_motor.speed_set = 0.0f;
    }
    else if (shoot_mode == SHOOT_BULLET)
    {
        trigger_motor.speed_pid.data.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        trigger_motor.speed_pid.data.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control();
    }
    else if (shoot_mode == SHOOT_CONTINUE_BULLET)
    {
        //设置拨弹轮的拨动速度,并开启堵转反转处理
        trigger_motor.speed_set = shoot_grigger_grade[2] * SHOOT_TRIGGER_DIRECTION;
    }
    else if (shoot_mode == SHOOT_DONE)
    {
        trigger_motor.speed_set = 0.0f;
    }

}

/**
  * @brief          PID计算
  * @param[in]      void
  * @retval         
  */
void Shoot::solve()
{
    //TODO 此处应该对速度设置值进行操作,而不是电流值,后续进行修改,有可能摩擦轮停止旋转速度慢和这个有关
    if (shoot_mode == SHOOT_STOP)
    {
        shoot_laser_off();
        fric_status = FALSE;

        trigger_motor.speed_set = 0;
        fric_motor[LEFT_FRIC].speed_set = 0;
        fric_motor[RIGHT_FRIC].speed_set = 0;
    }
    else
    {
        shoot_laser_on(); //激光开启
        //设置摩擦轮转速
        fric_motor[LEFT_FRIC].speed_set = shoot_fric_grade[2];
        fric_motor[RIGHT_FRIC].speed_set = shoot_fric_grade[2];

        //连发模式 控制17mm发射机构射速和热量控制
        if(shoot_mode == SHOOT_CONTINUE_BULLET)
            cooling_ctrl();

        if (shoot_mode == SHOOT_READY_BULLET || shoot_mode == SHOOT_CONTINUE_BULLET)
            trigger_motor_turn_back(); //将设置的拨盘旋转角度,转化为速度,且防止卡弹

        //确保摩擦轮未达到最低转速不会转动拨盘
        if (shoot_mode < SHOOT_READY_BULLET)
        {
            trigger_motor.current_set = 0;
        }
    }

    if (fric_motor[LEFT_FRIC].speed_set > fric_motor[LEFT_FRIC].max_speed)
        fric_motor[LEFT_FRIC].speed_set = fric_motor[LEFT_FRIC].max_speed;
    else if(fric_motor[LEFT_FRIC].speed_set < fric_motor[LEFT_FRIC].min_speed)
        fric_motor[LEFT_FRIC].speed_set = fric_motor[LEFT_FRIC].min_speed;

    if (fric_motor[RIGHT_FRIC].speed_set > fric_motor[RIGHT_FRIC].max_speed)
        fric_motor[RIGHT_FRIC].speed_set = fric_motor[RIGHT_FRIC].max_speed;
    else if (fric_motor[RIGHT_FRIC].speed_set < fric_motor[RIGHT_FRIC].min_speed)
        fric_motor[RIGHT_FRIC].speed_set = fric_motor[RIGHT_FRIC].min_speed;

    //计算PID
    fric_motor[LEFT_FRIC].current_set = fric_motor[LEFT_FRIC].speed_pid.pid_calc();
    fric_motor[RIGHT_FRIC].current_set = fric_motor[RIGHT_FRIC].speed_pid.pid_calc();
    trigger_motor.current_set = trigger_motor.speed_pid.pid_calc();
}

/**
* @brief          发射机构弹速和热量控制
* @param[in]      void
* @retval         
*/
void Shoot::cooling_ctrl()
{
    //TODO 暂时未完善
    //17mm枪口热量上限, 17mm枪口实时热量
    uint16_t id1_17mm_cooling_limit;
    uint16_t id1_17mm_cooling_heat;
    uint16_t id1_17mm_cooling_rate;

    //17mm枪口枪口射速上限,17mm实时射速
    uint16_t id1_17mm_speed_limit;
    fp32 bullet_speed;


    if (toe_is_error(REFEREE_TOE))
    {
        grigger_speed_grade = 1;
        fric_speed_grade = 1;
    }
    else
    {
        //更新裁判数据
        id1_17mm_cooling_limit = can_receive.gimbal_receive.id1_17mm_cooling_limit;
        id1_17mm_cooling_heat = can_receive.gimbal_receive.id1_17mm_cooling_heat;
        id1_17mm_cooling_rate = can_receive.gimbal_receive.id1_17mm_cooling_rate;

        id1_17mm_speed_limit = can_receive.gimbal_receive.id1_17mm_speed_limit;
        bullet_speed = can_receive.gimbal_receive.bullet_speed;

        //根据热量和射速上限修改等级
        //热量
        if (id1_17mm_cooling_limit <= 50)
            grigger_speed_grade = 1;
        else if (id1_17mm_cooling_limit <= 100)
            grigger_speed_grade = 2;
        else if (id1_17mm_cooling_limit <= 150)
            grigger_speed_grade = 3;
        else if (id1_17mm_cooling_limit <= 280)
            grigger_speed_grade = 4;
        else if (id1_17mm_cooling_limit <= 400)
            grigger_speed_grade = 5;

        //射速
        if (id1_17mm_speed_limit <= 15)
            fric_speed_grade = 1;
        else if (id1_17mm_speed_limit <= 18)
            fric_speed_grade = 2;
        else if (id1_17mm_speed_limit <= 30)
            fric_speed_grade = 3;

        //当调试射速和射频等级数组时可以暂时注释
        //根据当前热量和射速修改等级,确保不会因超限扣血,

        //热量 当剩余热量低于30,强制制动
        if (id1_17mm_cooling_limit - id1_17mm_cooling_heat <= 20 && grigger_speed_grade != 0)
            grigger_speed_grade = 0;

        //射速 超射速,强制降低摩擦轮转速
        if (bullet_speed > id1_17mm_speed_limit)
            fric_speed_grade--;
    }

    //对拨盘电机输入控制值
    trigger_motor.speed_set = shoot_grigger_grade[grigger_speed_grade] * SHOOT_TRIGGER_DIRECTION;
    //对摩擦轮电机输入控制值
    fric_motor[LEFT_FRIC].speed_set = -shoot_fric_grade[fric_speed_grade];
    fric_motor[RIGHT_FRIC].speed_set = shoot_fric_grade[fric_speed_grade];

}

void Shoot::output()
{
    fric_motor[LEFT_FRIC].current_give = -(int16_t)(fric_motor[LEFT_FRIC].current_set);
    fric_motor[RIGHT_FRIC].current_give = (int16_t)(fric_motor[RIGHT_FRIC].current_set);
    trigger_motor.current_give = trigger_motor.current_set;

//电流输出控制,通过调整宏定义控制
#if SHOOT_FRIC_MOTOR_NO_CURRENT
    fric_motor[LEFT_FRIC].current_give = 0;
    fric_motor[RIGHT_FRIC].current_give = 0;

#endif

#if SHOOT_TRIGGER_MOTOR_NO_CURRENT
    trigger_motor.current_give = 0;
#endif

    //发送电流
    can_receive.can_cmd_shoot_motor_motor(fric_motor[LEFT_FRIC].current_give, fric_motor[RIGHT_FRIC].current_give, trigger_motor.current_give, 0);
}

    /**
* @brief          拨盘电机反转控制
* @param[in]      void
* @retval         返回can控制值
*/
void Shoot::trigger_motor_turn_back()
{
    if (block_time < BLOCK_TIME)
    {
        trigger_motor.speed_set = trigger_motor.speed_set;
    }
    else
    {
        trigger_motor.speed_set = -trigger_motor.speed_set;
    }

    if (fabs(trigger_motor.speed) < BLOCK_TRIGGER_SPEED && block_time < BLOCK_TIME)
    {
        block_time++;
        reverse_time = 0;
    }
    else if (block_time == BLOCK_TIME && reverse_time < REVERSE_TIME)
    {
        reverse_time++;
    }
    else
    {
        block_time = 0;
    }
}

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
void Shoot::shoot_bullet_control()
{

    //每次拨动的角度
    if (move_flag == 0)
    {
        trigger_motor.angle_set = rad_format(trigger_motor.angle + TRIGGER_ONCE);
        move_flag = 1;
    }
    if (key == SWITCH_TRIGGER_OFF)
    {
        shoot_mode = SHOOT_DONE;
    }
    //到达角度判断
    if (rad_format(trigger_motor.angle_set - trigger_motor.angle) > 0.05f)
    {
        //没到达一直设置旋转速度
        trigger_motor.speed_set = shoot_grigger_grade[1] * SHOOT_TRIGGER_DIRECTION;
        trigger_motor_turn_back();
    }
    else
    {
        move_flag = 0;
        shoot_mode = SHOOT_READY;
    }
}

/**
  * @brief          弹仓打开时,云台要停止运动
  * @param[in]      none
  * @retval         1: no move 0:normal
  */

bool_t Shoot::shoot_cmd_to_gimbal_stop()
{
    if (cover_status == TRUE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}