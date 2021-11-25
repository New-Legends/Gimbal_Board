//
// Created by summerpray on 2021/11/2.
//

#include <first_order_filter.h>
#include "M_Gimbal.h"
#include "math.h"

extern remote_control RC;
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
void M_Gimbal::init() {
    //电机速度环PID
    static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};

    //电机陀螺仪角度环PID
    static const fp32 Pitch_angle_pid[3] = {PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD};
    static const fp32 Yaw_angle_pid[3] = {YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD};

    //电机编码器角度环PID
    static const fp32 Pitch_encode_pid[3] = {PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD};
    static const fp32 Yaw_encode_pid[3] = {YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD};

    gimbal_yaw_motor.gimbal_motor_measure = gimbal_can.get_gimbal_motor_measure_point(YAW);
    gimbal_pitch_motor.gimbal_motor_measure = gimbal_can.get_gimbal_motor_measure_point(PITCH);

    //TODO: 在INS初始化移植完毕后取消注释这里
    gimbal_INT_angle_point = get_INS_angle_point();
    gimbal_INT_gyro_point = get_gyro_data_point();

    //遥控器数据指针获取
    Rc.data = &rc_ctrl;

    //初始化电机控制模式
    gimbal_motor_mode = last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;

    //初始化yaw电机pid
    gimbal_PID_init(&gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
    gimbal_PID_init(&gimbal_yaw_motor.gimbal_motor_relative_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);
    PID_init(&gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_encode_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT,YAW_ENCODE_RELATIVE_PID_MAX_IOUT);

    //初始化pitch电机pid
    gimbal_PID_init(&gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
    gimbal_PID_init(&gimbal_pitch_motor.gimbal_motor_relative_angle_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
    PID_init(&gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_encode_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT,PITCH_ENCODE_RELATIVE_PID_MAX_IOUT);

    //定义yaw和pitch的限位
    //TODO:需要测试,先关闭
    //max_yaw = 0.0;
    //min_yaw = 0.0;
    //max_pitch = 0.0;
    //min_pitch = 0.0;

    //更新云台数据
    feedback_update();

    gimbal_yaw_motor.absolute_angle_set = gimbal_yaw_motor.absolute_angle;
    gimbal_yaw_motor.relative_angle_set = gimbal_yaw_motor.relative_angle;
    gimbal_yaw_motor.motor_gyro_set = gimbal_yaw_motor.motor_gyro;

    gimbal_pitch_motor.absolute_angle_set = gimbal_pitch_motor.absolute_angle;
    gimbal_pitch_motor.relative_angle_set = gimbal_pitch_motor.relative_angle;
    gimbal_pitch_motor.motor_gyro_set = gimbal_pitch_motor.motor_gyro;
}

/**
  * @brief          更新云台数据
  * @Author         summerpray
  */
void M_Gimbal::feedback_update()
{

    //云台数据更新
    gimbal_pitch_motor.absolute_angle = *(gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);

#if PITCH_TURN
    gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                   gimbal_pitch_motor.offset_ecd);
#else

    gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                   gimbal_pitch_motor.offset_ecd);
#endif

    gimbal_pitch_motor.motor_gyro = *(gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);

    gimbal_yaw_motor.absolute_angle = *(gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);

#if YAW_TURN
    gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                gimbal_yaw_motor.offset_ecd);

#else
    gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                gimbal_yaw_motor.offset_ecd);
#endif
    gimbal_yaw_motor.motor_gyro = cos(double(gimbal_pitch_motor.relative_angle)) * (*(gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
                                - sin(double(gimbal_pitch_motor.relative_angle)) * (*(gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
}

/**
  * @brief          设置云台控制模式,主要在'behaviour_mode_set'函数中改变
  * @Author         summerpray
  */
void M_Gimbal::set_mode() {
    //虽然这边就一个函数,但还是先留着,不要不知好歹
    behaviour_mode_set();
}


void M_Gimbal::behaviour_mode_set() {

    //云台行为状态机设置
    behavour_set();

    //根据云台行为状态机设置电机状态机
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE){
        gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_INIT){
        gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE){
        gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE){
        gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS){
        gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }

}

/**
  * @brief          云台行为状态机设置
  * @Author         summerpray
  */
void M_Gimbal::behavour_set(){
    //TODO:校准模式未写

    //初始化模式判断是否到达中值位置
    if (gimbal_behaviour == GIMBAL_INIT){
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
            !switch_is_down(Rc.data->rc.s[GIMBAL_MODE_CHANNEL]))
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
    if (switch_is_down(Rc.data->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }
    else if (switch_is_mid(Rc.data->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        gimbal_behaviour = GIMBAL_RELATIVE_ANGLE;
    }
    else if (switch_is_up(Rc.data->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
    }

    //TODO:此处要设置一个遥控器离线检测使云台不上电
    /*
    if( toe_is_error(DBUS_TOE))
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }
    */

    //enter init mode
    //判断进入init状态机
    {
        static gimbal_behaviour_e last_gimbal_behaviour = GIMBAL_ZERO_FORCE;
        if (last_gimbal_behaviour == GIMBAL_ZERO_FORCE && gimbal_behaviour != GIMBAL_ZERO_FORCE)
        {
            gimbal_behaviour = GIMBAL_INIT;
        }
        last_gimbal_behaviour = gimbal_behaviour;
    }
}

/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  * @Author         summerpray
  */
fp32 M_Gimbal::motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd){
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
  * @brief          云台切换模式数据保存
  * @Author         summerpray
  */
void M_Gimbal::mode_change_control_transit(){

    //切换模式数据保存
    //TODO:思考一下pitch和yaw真的需要分开保存吗
    if (last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_yaw_motor.raw_cmd_current = gimbal_yaw_motor.current_set = gimbal_yaw_motor.given_current;
    }
    else if (last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_yaw_motor.absolute_angle_set = gimbal_yaw_motor.absolute_angle;
    }
    else if (last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_yaw_motor.relative_angle_set = gimbal_yaw_motor.relative_angle;
    }
    last_gimbal_motor_mode = gimbal_motor_mode;

}

/**
  * @brief          设置云台控制设定值，控制值是通过behaviour_control_set函数设置的
  * @retval         none
  * Author          summerpray
  */
void M_Gimbal::set_control() {
    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;

    behaviour_control_set(&add_yaw_angle, &add_pitch_angle);

    //TODO:这么一想好像yaw用陀螺仪的时候pitch用编码器还是得分开写
    //yaw电机模式控制
    if (gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw模式下，直接发送控制值
        gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
        gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
    }
    else if (gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyro模式下，yaw陀螺仪角度控制
        absolute_angle_limit(&gimbal_yaw_motor, add_yaw_angle);
        //gyro模式下，pitch还是编码器模式控制
        absolute_angle_limit(&gimbal_pitch_motor, add_pitch_angle);
    }
    else if (gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //enconde模式下，电机编码角度控制
        relative_angle_limit(&gimbal_yaw_motor, add_yaw_angle);
        relative_angle_limit(&gimbal_pitch_motor, add_pitch_angle);
    }

    //TODO:但其实吧 写一块好像也行
}

/**
  * @brief          云台行为控制
  * Author          summerpray
  */
void M_Gimbal::behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch){

    if (add_yaw == NULL || add_pitch == NULL)
    {
        return;
    }

    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_zero_force_control(add_yaw, add_pitch);
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_init_control(add_yaw, add_pitch);
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_absolute_angle_control(add_yaw, add_pitch);
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_relative_angle_control(add_yaw, add_pitch);
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
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
void M_Gimbal::gimbal_zero_force_control(fp32 *yaw, fp32 *pitch) {
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
void M_Gimbal::gimbal_init_control(fp32 *yaw, fp32 *pitch) {
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
void M_Gimbal::gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch){
    if (yaw == NULL || pitch == NULL)
    {
        return;
    }

    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(Rc.data->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(Rc.data->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

    *yaw = yaw_channel * YAW_RC_SEN - Rc.data->mouse.x * YAW_MOUSE_SEN;
    *pitch = pitch_channel * PITCH_RC_SEN + Rc.data->mouse.y * PITCH_MOUSE_SEN;


    {
        static uint16_t last_turn_keyboard = 0;
        static uint8_t gimbal_turn_flag = 0;
        static fp32 gimbal_end_angle = 0.0f;

        if ((Rc.data->key.v & TURN_KEYBOARD) && !(last_turn_keyboard & TURN_KEYBOARD))
        {
            if (gimbal_turn_flag == 0)
            {
                gimbal_turn_flag = 1;
                //保存掉头的目标值
                gimbal_end_angle = rad_format(gimbal_yaw_motor.absolute_angle + PI);
            }
        }
        last_turn_keyboard = Rc.data->key.v ;

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
void M_Gimbal::gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch){
    if (yaw == NULL || pitch == NULL)
    {
        return;
    }
    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(Rc.data->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(Rc.data->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

    *yaw = yaw_channel * YAW_RC_SEN - Rc.data->mouse.x * YAW_MOUSE_SEN;
    *pitch = pitch_channel * PITCH_RC_SEN + Rc.data->mouse.y * PITCH_MOUSE_SEN;
}

/**
  * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @retval         none
  */
void M_Gimbal::gimbal_motionless_control(fp32 *yaw, fp32 *pitch) {
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
void M_Gimbal::gimbal_control_loop(){

    if (gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        motor_raw_angle_control(&gimbal_yaw_motor);
        motor_raw_angle_control(&gimbal_pitch_motor);
    }
    else if (gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        motor_absolute_angle_control(&gimbal_yaw_motor);
        motor_relative_angle_control(&gimbal_pitch_motor);
    }
    else if (gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        motor_relative_angle_control(&gimbal_yaw_motor);
        motor_relative_angle_control(&gimbal_pitch_motor);
    }

}

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_RAW，电流值直接发送到CAN总线.
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
void M_Gimbal::motor_raw_angle_control(motor_6020 *gimbal_motor) {
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
void M_Gimbal::motor_relative_angle_control(motor_6020 *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    //角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set =PID_calc(&gimbal_motor->gimbal_motor_gyro_pid,gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    //控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
void M_Gimbal::motor_absolute_angle_control(motor_6020 *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    //角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    //控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
void M_Gimbal::absolute_angle_limit(motor_6020 *gimbal_motor,fp32 add){
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
void M_Gimbal::relative_angle_limit(motor_6020 *gimbal_motor, fp32 add) {

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
/***************************(C)  MOTOR control *******************************/



/*****************************(C) GIMBAL PID *******************************/
/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @retval         none
  */
/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @retval         none
  */
void M_Gimbal::gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

fp32 M_Gimbal::gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta){
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(pid->out, pid->max_out);
    return pid->out;

    //TODO:这边要写一个最大值限制函数
}


void M_Gimbal::PID_clear(pid_type_def *gimbal_pid_clear) {
    if (gimbal_pid_clear == NULL)
        return;
    gimbal_pid_clear->error[3] = gimbal_pid_clear->set = gimbal_pid_clear->fdb = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}


/*****************************(C) GIMBAL PID *******************************/