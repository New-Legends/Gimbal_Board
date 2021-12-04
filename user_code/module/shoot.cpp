//
// Created by summerpray on 2021/11/30.
//

#include "shoot.h"

#include "math.h"

/*这部分是裁判，后续需要删除*/
#define FRIC_REFEREE_PARA 0.5      //射速裁判规定数值转实际输入
#define GRIGGER_SPEED_TO_RADIO 0.8 //射频裁判规定数值转实际输入

//射频等级 拨弹电机
fp32 shoot_grigger_grade[6] = {0, 5.0f * GRIGGER_SPEED_TO_RADIO, 10.0f * GRIGGER_SPEED_TO_RADIO, 15.0f * GRIGGER_SPEED_TO_RADIO, 28.0f * GRIGGER_SPEED_TO_RADIO, 40.0f * GRIGGER_SPEED_TO_RADIO};
//射速等级  摩擦电机
fp32 shoot_fric_grade[4] = {0, 15 * FRIC_REFEREE_PARA, 18 * FRIC_REFEREE_PARA, 30 * FRIC_REFEREE_PARA};
/*这部分是裁判，后续需要删除*/

//发射机构对象
shoot Shoot;

static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
static const fp32 Fric_speed_pid[3] = {FRIC_SPEED_PID_KP, FRIC_SPEED_PID_KI, FRIC_SPEED_PID_KD};

void shoot::init(void)
{
    //设置发射机构默认状态
    shoot_mode = SHOOT_STOP;
    //电机指针 拨弹 摩擦轮
    trigger_motor_measure = Can.get_trigger_motor_measure_point();
    fric_motor[LEFT].fric_motor_measure = Can.get_shoot_motor_measure_point(LEFT);
    fric_motor[RIGHT].fric_motor_measure = Can.get_shoot_motor_measure_point(RIGHT);

    //初始化PID
    PID_init(&trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_init(&fric_speed_pid[LEFT], PID_POSITION, Fric_speed_pid, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
    PID_init(&fric_speed_pid[RIGHT], PID_POSITION, Fric_speed_pid, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);

    //设置最大 最小值  左摩擦轮顺时针转 右摩擦轮逆时针转
    fric_motor[LEFT].max_speed = FRIC_MAX_SPEED_RMP;
    fric_motor[LEFT].min_speed = -FRIC_MAX_SPEED_RMP;
    fric_motor[LEFT].require_speed = -FRIC_REQUIRE_SPEED_RMP;

    fric_motor[RIGHT].max_speed = FRIC_MAX_SPEED_RMP;
    fric_motor[RIGHT].min_speed = -FRIC_MAX_SPEED_RMP;
    fric_motor[RIGHT].require_speed = -FRIC_REQUIRE_SPEED_RMP;

    //摩擦轮,弹仓舵机,限位舵机状态
    fric_status = FALSE;
    magazine_status = FALSE;
    limit_switch_status = FALSE;

    //记录上一次按键值
    shoot_last_key_v = 0;

    //更新数据
    feedback_update();

    button_key = 0;
    ecd_count = 0;
    angle = trigger_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    given_current = 0;
    move_flag = 0;
    set_angle = angle;
    speed_t = 0.0f;
    speed_set = 0.0f;
    key_time = 0;
}

void shoot::set_mode()
{
    static int8_t last_s = RC_SW_UP; //记录上一次遥控器按键值

    //上拨判断， 一次开启，再次关闭
    if ((switch_is_up(remote_control.rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_mode == SHOOT_STOP))
    {
        shoot_mode = SHOOT_READY_FRIC;
    }
    else if ((switch_is_up(remote_control.rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_mode != SHOOT_STOP))
    {
        shoot_mode = SHOOT_STOP;
    }

    //处于中档， 可以使用键盘开启摩擦轮
    if (switch_is_mid(remote_control.rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]) && KEY_FRIC && shoot_mode == SHOOT_STOP)
    {
        shoot_mode = SHOOT_READY_FRIC;
    }
    //处于中档， 可以使用键盘关闭摩擦轮
    else if (switch_is_mid(remote_control.rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]) && KEY_FRIC && shoot_mode != SHOOT_STOP)
    {
        shoot_mode = SHOOT_STOP;
    }

    shoot_last_key_v = remote_control.rc_ctrl.key.v;

    //摩擦轮速度达到一定值,才可开启拨盘  为了便于测试,这里至少需要一个摩擦轮电机达到拨盘启动要求就可以开启拨盘
    // && (remote_control.RC_abs(fric_motor[LEFT].fric_motor_measure->speed_rpm) > remote_control.RC_abs(fric_motor[LEFT].require_speed) || remote_control.RC_abs(fric_motor[RIGHT].fric_motor_measure->speed_rpm) > remote_control.RC_abs(fric_motor[RIGHT].require_speed))
    if (shoot_mode == SHOOT_READY_FRIC)
    {
        fric_status = TRUE;
        shoot_mode = SHOOT_READY_BULLET;
    }
    else if (shoot_mode == SHOOT_READY_BULLET && button_key == SWITCH_TRIGGER_ON)
    {
        shoot_mode = SHOOT_READY;
    }
    else if (shoot_mode == SHOOT_READY && button_key == SWITCH_TRIGGER_OFF)
    {
        shoot_mode = SHOOT_READY_BULLET;
    }
    else if (shoot_mode == SHOOT_READY)
    {
        //下拨一次或者鼠标按下一次，进入射击状态
        if ((switch_is_down(remote_control.rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)) || IF_MOUSE_SINGAL_PRESSED_L)
        {
            shoot_mode = SHOOT_BULLET;
        }
    }
    else if (shoot_mode == SHOOT_DONE)
    {
        if (button_key == SWITCH_TRIGGER_OFF)
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

    //检测两个摩擦轮同时上线，为了便于调试，暂时注释
    // if(!toe_is_error(REFEREE_TOE) && (heat + SHOOT_HEAT_REMAIN_VALUE > heat_limit))
    // {
    //     if(shoot_mode == SHOOT_BULLET || shoot_mode == SHOOT_CONTINUE_BULLET)
    //     {
    //         shoot_mode =SHOOT_READY_BULLET;
    //     }
    // }

    //如果云台状态是 无力状态，就关闭射击
    // if (gimbal.gimbal_cmd_to_shoot_stop())
    // {
    //     shoot_mode = SHOOT_STOP;
    // }

    last_s = remote_control.rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL];
}

bool_t key_flag_test = 0;

void shoot::feedback_update(void)
{
    //更新摩擦轮电机速度

    fric_motor[LEFT].speed = fric_motor[LEFT].fric_motor_measure->speed_rpm * FRIC_RPM_TO_SPEED;
    fric_motor[RIGHT].speed = fric_motor[RIGHT].fric_motor_measure->speed_rpm * FRIC_RPM_TO_SPEED;

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //二阶低通滤波
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (trigger_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    speed_t = speed_fliter_3;

    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    if (trigger_motor_measure->ecd - trigger_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        ecd_count--;
    }
    else if (trigger_motor_measure->ecd - trigger_motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        ecd_count++;
    }

    if (ecd_count == FULL_COUNT)
    {
        ecd_count = -(FULL_COUNT - 1);
    }
    else if (ecd_count == -FULL_COUNT)
    {
        ecd_count = FULL_COUNT - 1;
    }

    //计算输出轴角度
    angle = (ecd_count * ECD_RANGE + trigger_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
    //微动开关
    button_key = 0;
    key_flag_test = BUTTEN_TRIG_PIN;
    //鼠标按键
    last_press_l = press_l;
    last_press_r = press_r;
    press_l = remote_control.rc_ctrl.mouse.press_l;
    press_r = remote_control.rc_ctrl.mouse.press_r;
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
    if (shoot_mode != SHOOT_STOP && switch_is_down(remote_control.rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]))
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
void shoot::set_control(void)
{
    if (shoot_mode == SHOOT_STOP)
    {
        //设置拨弹轮的速度
        speed_set = 0.0f;
    }
    else if (shoot_mode == SHOOT_READY_FRIC)
    {
        //设置拨弹轮的速度
        speed_set = 0.0f;
    }
    else if (shoot_mode == SHOOT_READY_BULLET)
    {
        if (button_key == SWITCH_TRIGGER_OFF)
        {
            //设置拨弹轮的拨动速度,并开启堵转反转处理
            trigger_speed_set = shoot_grigger_grade[1] * SHOOT_TRIGGER_DIRECTION;
        }
        else
        {
            trigger_speed_set = 0.0f;
            speed_set = 0.0f;
        }
        trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
        trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
    }
    else if (shoot_mode == SHOOT_READY)
    {
        //设置拨弹轮的速度
        speed_set = 0.0f;
    }
    else if (shoot_mode == SHOOT_BULLET)
    {
        trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        bullet_control();
    }
    else if (shoot_mode == SHOOT_CONTINUE_BULLET)
    {
        //设置拨弹轮的拨动速度,并开启堵转反转处理
        trigger_speed_set = shoot_grigger_grade[2] * SHOOT_TRIGGER_DIRECTION;
    }
    else if (shoot_mode == SHOOT_DONE)
    {
        speed_set = 0.0f;
    }

    if (shoot_mode == SHOOT_STOP)
    {
        given_current = 0;
        fric_motor[LEFT].give_current = 0.0f;
        fric_motor[RIGHT].give_current = 0.0f;
        fric_status = FALSE;
    }
    else
    {
        //设置摩擦轮转速
        fric_motor[LEFT].speed_set = -shoot_fric_grade[2];
        fric_motor[RIGHT].speed_set = shoot_fric_grade[2];

        //        //连发模式 控制17mm发射机构射速和热量控制
        //        if(shoot_mode == SHOOT_CONTINUE_BULLET)
        //            shoot_id1_17mm_speed_and_cooling_control(&shoot_control);

        if (shoot_mode == SHOOT_READY_BULLET || shoot_mode == SHOOT_CONTINUE_BULLET)
            trigger_motor_turn_back(); //将设置的拨盘旋转角度,转化为速度,且防止卡弹

        //计算拨弹轮电机PID
        PID_calc(&trigger_motor_pid, speed_t, speed_set);
        //计算摩擦轮电机PID
        PID_calc(&fric_speed_pid[LEFT], fric_motor[LEFT].speed, fric_motor[LEFT].speed_set);
        PID_calc(&fric_speed_pid[RIGHT], fric_motor[RIGHT].speed, fric_motor[RIGHT].speed_set);

        //确保摩擦轮未达到最低转速不会转动拨盘
        if (shoot_mode < SHOOT_READY_BULLET)
        {
            given_current = 0;
        }

        //设置发送电流
        given_current = (int16_t)(trigger_motor_pid.out);
        fric_motor[LEFT].give_current = fric_speed_pid[LEFT].out;
        fric_motor[RIGHT].give_current = fric_speed_pid[RIGHT].out;
    }
}

void shoot::bullet_control(void)
{
    //每次拨动的角度
    if (move_flag == 0)
    {
        set_angle = rad_format(angle + TRIGGER_ONCE);
        move_flag = 1;
    }
    if (button_key == SWITCH_TRIGGER_OFF)
    {
        shoot_mode = SHOOT_DONE;
    }
    //到达角度判断
    if (rad_format(set_angle - angle) > 0.05f)
    {
        //没到达一直设置旋转速度
        trigger_speed_set = shoot_grigger_grade[1] * SHOOT_TRIGGER_DIRECTION / 10;
        trigger_motor_turn_back();
    }
    else
    {
        move_flag = 0;
        shoot_mode = SHOOT_READY;
    }
}

void shoot::trigger_motor_turn_back(void)
{
    if (block_time < BLOCK_TIME)
    {
        speed_set = trigger_speed_set;
    }
    else
    {
        speed_set = -trigger_speed_set;
    }

    if (fabs(speed_t) < BLOCK_TRIGGER_SPEED && block_time < BLOCK_TIME)
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

bool_t shoot::cmd_to_gimbal_stop(void)
{
    if (magazine_status == TRUE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}