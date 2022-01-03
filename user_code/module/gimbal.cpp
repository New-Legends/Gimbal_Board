//
// Created by summerpray on 2021/11/2.
//

#include "Gimbal.h"
#include "Can_receive.h"
#include "math.h"
#include "ins_task.h"
#include "First_order_filter.h"

//底盘模块 对象
Gimbal gimbal;

//motor enconde value format, range[0-8191]
//电机编码值规整 0—8191
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
  * @param          输入的遥控器值
  * @param          输出的死区处理后遥控器值
  * @param          死区值
  */
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
  * @brief          初始化云台
  * @Author         summerpray
  */
void Gimbal::init() {
    //遥控器数据指针获取
    gimbal_RC = remote_control.get_remote_control_point();
    gimbal_last_key_v = 0;

    //设置初试状态机
    gimbal_behaviour_mode = GIMBAL_ZERO_FORCE;
    last_gimbal_behaviour_mode = gimbal_behaviour_mode;
    
    /*----------------------yaw电机数据------------------------------*/
    gimbal_yaw_motor.init(can_receive.get_gimbal_motor_measure_point(YAW));
    gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_yaw_motor.gimbal_motor_mode;

    //初始化pid
    fp32 yaw_speed_pid_parm[5] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD, YAW_SPEED_PID_MAX_IOUT, YAW_SPEED_PID_MAX_OUT};
    gimbal_yaw_motor.speed_pid.init(PID_SPEED, yaw_speed_pid_parm, &gimbal_yaw_motor.speed, &gimbal_yaw_motor.speed_set, NULL);
    fp32 yaw_absoulute_angle_pid_parm[5] = {YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_MAX_OUT};
    gimbal_yaw_motor.absolute_angle_pid.init(PID_ANGLE, yaw_absoulute_angle_pid_parm, &gimbal_yaw_motor.absolute_angle, &gimbal_yaw_motor.absolute_angle_set, 0);
    fp32 yaw_relative_angle_pid_parm[5] = {YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_MAX_OUT};
    gimbal_yaw_motor.relative_angle_pid.init(PID_ANGLE, yaw_relative_angle_pid_parm, &gimbal_yaw_motor.relative_angle, &gimbal_yaw_motor.relative_angle_set, 0);

    //设置舵向电机角度限幅和中值
    gimbal_yaw_motor.max_absolute_angle = MAX_ABSOULATE_YAW;
    gimbal_yaw_motor.min_absolute_angle = MIN_ABSOULATE_YAW;

    //设置舵向电机初试编码中值
    gimbal_yaw_motor.offset_ecd = YAW_OFFSET;


    /*----------------------pitch电机数据------------------------------*/
    gimbal_pitch_motor.init(can_receive.get_gimbal_motor_measure_point(PITCH));
    gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_pitch_motor.gimbal_motor_mode;

    //初始化pid
    fp32 pitch_speed_pid_parm[5] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD, PITCH_SPEED_PID_MAX_IOUT, PITCH_SPEED_PID_MAX_OUT};
    gimbal_pitch_motor.speed_pid.init(PID_SPEED, pitch_speed_pid_parm, &gimbal_pitch_motor.speed, &gimbal_pitch_motor.speed_set, NULL);
    fp32 pitch_absoulute_angle_pid_parm[5] = {PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT};
    gimbal_pitch_motor.absolute_angle_pid.init(PID_ANGLE, pitch_absoulute_angle_pid_parm, &gimbal_pitch_motor.absolute_angle, &gimbal_pitch_motor.absolute_angle_set, 0);
    fp32 pitch_relative_angle_pid_parm[5] = {PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_MAX_OUT};
    gimbal_pitch_motor.relative_angle_pid.init(PID_ANGLE, pitch_relative_angle_pid_parm, &gimbal_pitch_motor.relative_angle, &gimbal_pitch_motor.relative_angle_set, 0);

    //设置舵向电机角度限幅和中值
    gimbal_pitch_motor.max_absolute_angle = MAX_ABSOULATE_PITCH;
    gimbal_pitch_motor.min_absolute_angle = MIN_ABSOULATE_PITCH;

    //设置舵向电机初试编码中值
    gimbal_pitch_motor.offset_ecd = PITCH_OFFSET;

    //更新云台数据
    feedback_update();

    gimbal_yaw_motor.absolute_angle_set = gimbal_yaw_motor.absolute_angle;
    gimbal_yaw_motor.relative_angle_set = gimbal_yaw_motor.relative_angle;
    gimbal_yaw_motor.speed_set = gimbal_yaw_motor.speed_set;

    gimbal_pitch_motor.absolute_angle_set = gimbal_pitch_motor.absolute_angle;
    gimbal_pitch_motor.relative_angle_set = gimbal_pitch_motor.relative_angle;
    gimbal_pitch_motor.speed_set = gimbal_pitch_motor.speed_set;
}

/**
  * @brief          更新云台数据
  * @Author         summerpray
  */
void Gimbal::feedback_update()
{
    //切换模式数据保存
    //TODO:思考一下pitch和yaw真的需要分开保存吗
    //yaw电机状态机切换保存数据
    if (gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_yaw_motor.current_set = gimbal_yaw_motor.current_give;
    }
    else if (gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_yaw_motor.absolute_angle_set = gimbal_yaw_motor.absolute_angle;
    }
    else if (gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_yaw_motor.relative_angle_set = gimbal_yaw_motor.relative_angle;
    }
    gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_yaw_motor.gimbal_motor_mode;

    //pitch电机状态机切换保存数据
    if (gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_pitch_motor.current_set = gimbal_pitch_motor.current_give;
    }
    else if (gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_pitch_motor.absolute_angle_set = gimbal_pitch_motor.absolute_angle;
    }
    else if (gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_pitch_motor.relative_angle_set = gimbal_pitch_motor.relative_angle;
    }

    gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_pitch_motor.gimbal_motor_mode;


    //记录上一次遥控器值
    gimbal_last_key_v = gimbal_RC->key.v;

    //云台数据更新
    //yaw电机
    gimbal_pitch_motor.absolute_angle = *(gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);

#if YAW_TURN
    gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(gimbal_yaw_motor.motor_measure_t->ecd,
                                                                 gimbal_yaw_motor.offset_ecd);
#else
    gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(gimbal_yaw_motor.motor_measure->ecd,
                                                                 gimbal_yaw_motor.offset_ecd);
#endif

    gimbal_yaw_motor.speed = cos(gimbal_pitch_motor.relative_angle) * (*(gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET)) - sin(gimbal_pitch_motor.relative_angle) * (*(gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));

    //pitch电机
    gimbal_pitch_motor.absolute_angle = *(gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);

#if YAW_TURN
    gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(gimbal_pitch_motor.motor_measure_t->ecd,
                                                                   gimbal_pitch_motor.offset_ecd);
#else
    gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(gimbal_pitch_motor.motor_measure->ecd,
                                                                  gimbal_pitch_motor.offset_ecd);
#endif

    gimbal_pitch_motor.speed = *(gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);
}

/**
  * @brief          设置云台控制模式,主要在'behaviour_mode_set'函数中改变
  * @Author         summerpray
  */
void Gimbal::set_mode() {
    //虽然这边就一个函数,但还是先留着,不要不知好歹
    behaviour_mode_set();
}


void Gimbal::behaviour_mode_set() {

    //云台行为状态机设置
    behavour_set();

    //accoring to gimbal_behaviour, set motor control mode
    //根据云台行为状态机设置电机状态机
    if (gimbal_behaviour_mode == GIMBAL_ZERO_FORCE)
    {
        gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour_mode == GIMBAL_INIT)
    {
        gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour_mode == GIMBAL_CALI)
    {
        gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour_mode == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if (gimbal_behaviour_mode == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour_mode == GIMBAL_MOTIONLESS)
    {
        gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
}

/**
  * @brief          云台行为状态机设置
  * @Author         summerpray
  */
void Gimbal::behavour_set(){
    //TODO:校准模式未写

    //初始化模式判断是否到达中值位置
    if (gimbal_behaviour_mode == GIMBAL_INIT){
        static uint16_t init_time = 0;
        static uint16_t init_stop_time = 0;
        init_time++;

        if ((fabs(gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR &&
             fabs(gimbal_pitch_motor.absolute_angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR)){
            if (init_stop_time < GIMBAL_INIT_STOP_TIME){
                init_stop_time++;
            }
        }
        else{
            if (init_time < GIMBAL_INIT_TIME){
                init_time++;
            }
        }

        //超过初始化最大时间，或者已经稳定到中值一段时间，退出初始化状态开关打下档，或者掉线
        //TODO:掉线未写
        if (init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME &&
            !switch_is_down(gimbal_RC->rc.s[GIMBAL_MODE_CHANNEL]))
        {
            return;
        }
        else
        {
            init_stop_time = 0;
            init_time = 0;
        }

    }

    //开关控制 云台状态
    if (switch_is_down(gimbal_RC->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        gimbal_behaviour_mode = GIMBAL_ZERO_FORCE;
    }
    else if (switch_is_mid(gimbal_RC->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        gimbal_behaviour_mode = GIMBAL_RELATIVE_ANGLE;
    }
    else if (switch_is_up(gimbal_RC->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        gimbal_behaviour_mode = GIMBAL_ABSOLUTE_ANGLE;
    }

    //TODO:此处要设置一个遥控器离线检测使云台不上电
    /*
    if( toe_is_error(DBUS_TOE))
    {
        gimbal_behaviour_mode = GIMBAL_ZERO_FORCE;
    }
    */

    //enter init mode
    //判断进入init状态机
    {
        if (last_gimbal_behaviour_mode == GIMBAL_ZERO_FORCE && gimbal_behaviour_mode != GIMBAL_ZERO_FORCE)
        {
            gimbal_behaviour_mode = GIMBAL_INIT;
        }
        last_gimbal_behaviour_mode = gimbal_behaviour_mode;
    }
}

/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  * @Author         summerpray
  */
fp32 Gimbal::motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd){
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}


/**
  * @brief          设置云台控制设定值，控制值是通过behaviour_control_set函数设置的
  * @retval         none
  * Author          summerpray
  */
void Gimbal::set_control() {
    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;

    behaviour_control_set(&add_yaw_angle, &add_pitch_angle);

    //TODO:这么一想好像yaw用陀螺仪的时候pitch用编码器还是得分开写

    // yaw电机模式控制 
    if (gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw模式下，直接发送控制值
        gimbal_yaw_motor.current_set = add_yaw_angle;
    }
    else if (gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyro模式下，陀螺仪角度控制
        absolute_angle_limit(&gimbal_yaw_motor, add_yaw_angle);
    }
    else if (gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //enconde模式下，电机编码角度控制
        relative_angle_limit(&gimbal_yaw_motor, add_yaw_angle);
    }

    //pitch电机模式控制
    if (gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw模式下，直接发送控制值
        gimbal_pitch_motor.current_set = add_pitch_angle;
    }
    else if (gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyro模式下，陀螺仪角度控制
        absolute_angle_limit(&gimbal_pitch_motor, add_pitch_angle);
    }
    else if (gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //enconde模式下，电机编码角度控制
        relative_angle_limit(&gimbal_pitch_motor, add_pitch_angle);
    }

    //TODO:但其实吧 写一块好像也行
}

/**
  * @brief          云台行为控制
  * Author          summerpray
  */
void Gimbal::behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch){

    if (add_yaw == NULL || add_pitch == NULL)
    {
        return;
    }

    if (gimbal_behaviour_mode == GIMBAL_ZERO_FORCE)
    {
        gimbal_zero_force_control(add_yaw, add_pitch);
    }
    else if (gimbal_behaviour_mode == GIMBAL_INIT)
    {
        gimbal_init_control(add_yaw, add_pitch);
    }
    else if (gimbal_behaviour_mode == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_absolute_angle_control(add_yaw, add_pitch);
    }
    else if (gimbal_behaviour_mode == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_relative_angle_control(add_yaw, add_pitch);
    }
    else if (gimbal_behaviour_mode == GIMBAL_MOTIONLESS)
    {
        gimbal_motionless_control(add_yaw, add_pitch);
    }
}

/***************************(C) GIMBAL control *******************************/
/**
  * @brief          当云台行为模式是GIMBAL_ZERO_FORCE, 这个函数会被调用,云台控制模式是raw模式.原始模式意味着
  *                 设定值会直接发送到CAN总线上,这个函数将会设置所有为0.
  * @param[in]      yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      pitch:发送pitch电机的原始值，会直接通过can 发送到电机
  * @retval         none
  */
void Gimbal::gimbal_zero_force_control(fp32 *yaw, fp32 *pitch) {
    if (yaw == NULL || pitch == NULL)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;
}

/**
  * @brief          云台初始化控制，电机是陀螺仪角度控制，云台先抬起pitch轴，后旋转yaw轴
  * @param[out]     yaw轴角度控制，为角度的增量 单位 rad
  * @param[out]     pitch轴角度控制，为角度的增量 单位 rad
  * @retval         返回空
  */
void Gimbal::gimbal_init_control(fp32 *yaw, fp32 *pitch) {
    if (yaw == NULL || pitch == NULL)
    {
        return;
    }

    //初始化状态控制量计算
    if (fabs(INIT_PITCH_SET - gimbal_pitch_motor.absolute_angle) > GIMBAL_INIT_ANGLE_ERROR)
    {
        *pitch = (INIT_PITCH_SET - gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = 0.0f;
    }
    else
    {
        *pitch = (INIT_PITCH_SET - gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = (INIT_YAW_SET - gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
    }
}

/**
  * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
  * @param[out]     yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[out]     pitch:pitch轴角度控制，为角度的增量 单位 rad
  * @retval         none
  */
void Gimbal::gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch){
    if (yaw == NULL || pitch == NULL)
    {
        return;
    }

    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(gimbal_RC->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(gimbal_RC->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

    *yaw = yaw_channel * YAW_RC_SEN - gimbal_RC->mouse.x * YAW_MOUSE_SEN;
    *pitch = pitch_channel * PITCH_RC_SEN + gimbal_RC->mouse.y * PITCH_MOUSE_SEN;


    {
        static uint16_t last_turn_keyboard = 0;
        static uint8_t gimbal_turn_flag = 0;
        static fp32 gimbal_end_angle = 0.0f;

        if ((gimbal_RC->key.v & TURN_KEYBOARD) && !(last_turn_keyboard & TURN_KEYBOARD))
        {
            if (gimbal_turn_flag == 0)
            {
                gimbal_turn_flag = 1;
                //保存掉头的目标值
                gimbal_end_angle = rad_format(gimbal_yaw_motor.absolute_angle + PI);
            }
        }
        last_turn_keyboard = gimbal_RC->key.v ;

        if (gimbal_turn_flag)
        {
            //不断控制到掉头的目标值，正转，反装是随机
            if (rad_format(gimbal_end_angle - gimbal_yaw_motor.absolute_angle) > 0.0f)
            {
                *yaw += TURN_SPEED;
            }
            else
            {
                *yaw -= TURN_SPEED;
            }
        }
        //到达pi （180°）后停止
        if (gimbal_turn_flag && fabs(rad_format(gimbal_end_angle - gimbal_yaw_motor.absolute_angle)) < 0.01f)
        {
            gimbal_turn_flag = 0;
        }
    }
}

/**
  * @brief          云台编码值控制，电机是相对角度控制，
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @retval         none
  */
void Gimbal::gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch){
    if (yaw == NULL || pitch == NULL)
    {
        return;
    }
    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(gimbal_RC->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(gimbal_RC->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

    *yaw = yaw_channel * YAW_RC_SEN - gimbal_RC->mouse.x * YAW_MOUSE_SEN;
    *pitch = pitch_channel * PITCH_RC_SEN + gimbal_RC->mouse.y * PITCH_MOUSE_SEN;
}

/**
  * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @retval         none
  */
void Gimbal::gimbal_motionless_control(fp32 *yaw, fp32 *pitch) {
    if (yaw == NULL || pitch == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}

/***************************(C) GIMBAL control *******************************/

/***************************(C)  MOTOR control *******************************/
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @retval         none
  * @Author         summerpray
  */
void Gimbal::solve()
{
    //yaw电机
    if (gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        motor_raw_angle_control(&gimbal_yaw_motor);
    }
    else if (gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        motor_absolute_angle_control(&gimbal_yaw_motor);
    }
    else if (gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        motor_relative_angle_control(&gimbal_yaw_motor);
    }

    //pitch电机
    if (gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        motor_raw_angle_control(&gimbal_pitch_motor);
    }
    else if (gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        motor_absolute_angle_control(&gimbal_pitch_motor);
    }
    else if (gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        motor_relative_angle_control(&gimbal_pitch_motor);
    }
}

/**
  * @brief         输出电流
  * @retval         none
  * @Author         summerpray
  */
void Gimbal::output()
{

#if YAW_TURN
    gimbal_yaw_motor->current_give = -(int16_t)(gimbal_yaw_motor->current_set);
#else
    gimbal_yaw_motor.current_give = (int16_t)(gimbal_yaw_motor.current_set);
#endif

#if PITCH_TURN
    gimbal_pitch_motor.current_give = -(int16_t)(gimbal_pitch_motor.current_set);
#else
    gimbal_pitch_motor->current_give = (int16_t)(gimbal_pitch_motor->current_set);
#endif

//电流控制
#ifdef GIMBAL_YAW_MOTOR_NO_CURRENT
    gimbal_yaw_motor->current_give = 0;
#endif

#ifdef GIMBAL_PITCH_MOTOR_NO_CURRENT
    gimbal_pitch_motor->current_give = 0;
#endif

    can_receive.can_cmd_gimbal_motor(gimbal_yaw_motor.current_give, gimbal_pitch_motor.current_give, 0, 0);
    //TODO:看到这两个can发送机械的大兄弟有什么要说的嘛
}


/**
  * @brief          云台控制模式:GIMBAL_MOTOR_RAW，电流值直接发送到CAN总线.
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
      */ 
   
void     Gimbal::motor_raw_angle_control(Gimbal_motor *gimbal_motor) {
if (gimbal_motor == NULL)
{    
    return;
}
        gimbal_motor->current_set = gimbal_motor->current_set;
}
    
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
      */ 
void Gimbal::motor_relative_angle_control(Gimbal_motor *gimbal_motor)
{    
    if (gimbal_motor == NULL)
    {    
        return;
    }
    
    //角度环，速度环串级pid调试
    gimbal_motor->speed_set = gimbal_motor->relative_angle_pid.pid_calc();
    gimbal_motor->current_set = gimbal_motor->speed_pid.pid_calc();
}
    
/**
  * @brief          云台控制模式:GIMBAL_speed，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
      */ 
void Gimbal::motor_absolute_angle_control(Gimbal_motor *gimbal_motor)
{    
    if (gimbal_motor == NULL)
    {    
        return;
    }

    //角度环，速度环串级pid调试
    gimbal_motor->speed_set = gimbal_motor->absolute_angle_pid.pid_calc();
    gimbal_motor->current_set = gimbal_motor->speed_pid.pid_calc();

}
    
/**
  * @brief          云台控制模式:GIMBAL_speed，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
      */  
    
void Gimbal::absolute_angle_limit(Gimbal_motor *gimbal_motor,fp32 add){
    static fp32 bias_angle;
    static fp32 angle_set;
    //now angle error
    //当前控制误差角度
    bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
    //relative angle + angle error + add_angle > max_relative angle
    //云台相对角度+ 误差角度 + 新增角度 如果大于 最大机械角度
    if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
{    
        //如果是往最大机械角度控制方向
        if (add > 0.0f)
    {    
            //calculate max add_angle
            //计算出一个最大的添加角度，
            add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
        }
    }
    else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle)
{    
        if (add < 0.0f)
    {    
            add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
        }
    }
    angle_set = gimbal_motor->absolute_angle_set;
    gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}
    
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
      */ 
   
void Gimbal::relative_angle_limit(Gimbal_motor *gimbal_motor, fp32 add) {
    
        gimbal_motor->relative_angle_set += add;
        //是否超过最大 最小值
        if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
        {
            gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
        }
        else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
        {
            gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
        }
    }

    
    /*****************************(C) CALI GIMBAL *******************************/
/**
  * @brief          云台校准计算，将校准记录的中值,最大 最小值
  * @param[out]     yaw 中值 指针
  * @param[out]     pitch 中值 指针
  * @param[out]     yaw 最大相对角度 指针
  * @param[out]     yaw 最小相对角度 指针
  * @param[out]     pitch 最大相对角度 指针
  * @param[out]     pitch 最小相对角度 指针
  * @retval         none
      */
    
void Gimbal::calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch){
    if (gimbal_cali == NULL || yaw_offset == NULL || pitch_offset == NULL || max_yaw == NULL || min_yaw == NULL || max_pitch == NULL || min_pitch == NULL)
        {
            return;
    }
    
    int16_t temp_max_ecd = 0, temp_min_ecd = 0, temp_ecd = 0;

#if YAW_TURN
    temp_ecd = gimbal_cali->min_yaw_ecd - gimbal_cali->max_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ECD_RANGE;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd + (temp_ecd / 2);

    ecd_format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = -motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = -motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#else

    temp_ecd = gimbal_cali->max_yaw_ecd - gimbal_cali->min_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ECD_RANGE;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd - (temp_ecd / 2);
    
    ecd_format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#endif

#if PITCH_TURN

    temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
    temp_max_ecd = gimbal_cali->max_pitch_ecd + temp_ecd;
    temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
    temp_min_ecd = gimbal_cali->min_pitch_ecd + temp_ecd;

    ecd_format(temp_max_ecd);
    ecd_format(temp_min_ecd);

    temp_ecd = temp_max_ecd - temp_min_ecd;

    if (temp_ecd > HALF_ECD_RANGE)
    {
        temp_ecd -= ECD_RANGE;
    }
    else if (temp_ecd < -HALF_ECD_RANGE)
    {
        temp_ecd += ECD_RANGE;
    }

    if (temp_max_ecd > temp_min_ecd)
    {
        temp_min_ecd += ECD_RANGE;
    }

    temp_ecd = temp_max_ecd - temp_ecd / 2;

    ecd_format(temp_ecd);

    *pitch_offset = temp_ecd;

    *max_pitch = -motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
    *min_pitch = -motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);

#else
    temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
    temp_max_ecd = gimbal_cali->max_pitch_ecd - temp_ecd;
    temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
    temp_min_ecd = gimbal_cali->min_pitch_ecd - temp_ecd;

    ecd_format(temp_max_ecd);
    ecd_format(temp_min_ecd);

    temp_ecd = temp_max_ecd - temp_min_ecd;

    if (temp_ecd > HALF_ECD_RANGE)
    {
        temp_ecd -= ECD_RANGE;
    }
    else if (temp_ecd < -HALF_ECD_RANGE)
    {
        temp_ecd += ECD_RANGE;
    }

    temp_ecd = temp_max_ecd - temp_ecd / 2;

    ecd_format(temp_ecd);

    *pitch_offset = temp_ecd;

    *max_pitch = motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
    *min_pitch = motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);
#endif
}

/**
  * @brief          云台设备校准
  * @param[in][out] cali:指针指向云台数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
void Gimbal::set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch){
    gimbal_yaw_motor.offset_ecd = yaw_offset;
    gimbal_yaw_motor.max_relative_angle = max_yaw;
    gimbal_yaw_motor.min_relative_angle = min_yaw;

    gimbal_pitch_motor.offset_ecd = pitch_offset;
    gimbal_pitch_motor.max_relative_angle = max_pitch;
    gimbal_pitch_motor.min_relative_angle = min_pitch;
}

/**
  * @brief          云台校准计算，将校准记录的中值,最大 最小值返回
  * @param[out]     yaw 中值 指针
  * @param[out]     pitch 中值 指针
  * @param[out]     yaw 最大相对角度 指针
  * @param[out]     yaw 最小相对角度 指针
  * @param[out]     pitch 最大相对角度 指针
  * @param[out]     pitch 最小相对角度 指针
  * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
  * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
  */
bool_t Gimbal::cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_cali.step == 0)
    {
        gimbal_cali.step             = GIMBAL_CALI_START_STEP;
        //保存进入时候的数据，作为起始数据，来判断最大，最小值
        gimbal_cali.max_pitch        = gimbal_pitch_motor.absolute_angle;
        gimbal_cali.max_pitch_ecd    = gimbal_pitch_motor.motor_measure->ecd;
        gimbal_cali.max_yaw          = gimbal_yaw_motor.absolute_angle;
        gimbal_cali.max_yaw_ecd      = gimbal_yaw_motor.motor_measure->ecd;
        gimbal_cali.min_pitch        = gimbal_pitch_motor.absolute_angle;
        gimbal_cali.min_pitch_ecd    = gimbal_pitch_motor.motor_measure->ecd;
        gimbal_cali.min_yaw          = gimbal_yaw_motor.absolute_angle;
        gimbal_cali.min_yaw_ecd      = gimbal_yaw_motor.motor_measure->ecd;
        return 0;
    }
    else if (gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        calc_gimbal_cali(&gimbal_cali, yaw_offset, pitch_offset, max_yaw, min_yaw, max_pitch, min_pitch);
        (*max_yaw) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_yaw) += GIMBAL_CALI_REDUNDANT_ANGLE;
        (*max_pitch) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_pitch) += GIMBAL_CALI_REDUNDANT_ANGLE;
        gimbal_yaw_motor.offset_ecd              = *yaw_offset;
        gimbal_yaw_motor.max_relative_angle      = *max_yaw;
        gimbal_yaw_motor.min_relative_angle      = *min_yaw;
        gimbal_pitch_motor.offset_ecd            = *pitch_offset;
        gimbal_pitch_motor.max_relative_angle    = *max_pitch;
        gimbal_pitch_motor.min_relative_angle    = *min_pitch;
        gimbal_cali.step = 0;
        return 1;
    }
    else
    {
        return 0;
    }
}



/**
  * @brief          云台在某些行为下，需要射击停止
  * @param[in]      none
  * @retval         1: no move 0:normal
  */

bool_t Gimbal::gimbal_cmd_to_shoot_stop(void)
{
    if (gimbal_behaviour_mode == GIMBAL_INIT || gimbal_behaviour_mode == GIMBAL_CALI || gimbal_behaviour_mode == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}