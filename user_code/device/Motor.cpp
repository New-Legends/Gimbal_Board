#include "Motor.h"

#include "Pid.h"
#include "Can_receive.h"

void M3508_motor::init(const motor_measure_t *motor_measure_)
{
    motor_measure = motor_measure_;
}

void G6020_motor::init(const motor_measure_t *motor_measure_)
{
    motor_measure = motor_measure_;
}

void Gimbal_motor::init(const motor_measure_t *motor_measure_)
{
    motor_measure = motor_measure_;
}

void Firc_motor::init(const motor_measure_t *motor_measure_)
{
    motor_measure = motor_measure_;
}

void Trigger_motor::init(const motor_measure_t *motor_measure_)
{
    motor_measure = motor_measure_;
}

void Cover_motor::init(const motor_measure_t *motor_measure_)
{
    motor_measure = motor_measure_;
}
