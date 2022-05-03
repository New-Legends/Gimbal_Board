#ifndef CONFIG_H
#define CONFIG_H

/*------------------------陀螺仪---------------------------*/


/*------------------------云台---------------------------*/
//云台电机无电流输出
#define GIMBAL_YAW_MOTOR_NO_CURRENT 0
#define GIMBAL_PITCH_MOTOR_NO_CURRENT 0

//云台电机debug模式
#define GIMBAL_DEBUG_MODE 1
#define GIMBAL_VISION_OPEN 0

//下云台是否接了接收器,1为未接，通讯开启
#define GIMBAL_REMOTE_OPEN 1

/*-----------------------发射机构------------------------------*/
//摩擦轮电机无电流输出
#define SHOOT_FRIC_MOTOR_NO_CURRENT 0

//拨弹电机无电流输出
#define SHOOT_TRIGGER_MOTOR_NO_CURRENT 0

//手动设置射频
#define SHOOT_SET_TRIGGER_SPEED_BY_HAND 1

//激光是否开启
#define SHOOT_LASER_OPEN 0




/*--------------------按键-------------------------------------*/
// turn 180°
//掉头180 按键 单击V
#define KEY_PRESSED_GIMBAL_TURN_180 'B'

//开启和关闭摩擦轮  单击
#define KEY_PRESSED_SHOOT_FRIC 'G'
//#define KEY_PRESSED_SHOOT_FRIC KEY_PRESSED_OFFSET_G

//弹仓开关 长按R打开弹仓 单击R关闭弹仓
#define KEY_PRESSED_SHOOT_COVER 'R'

//提高射频  长按ctrl+Z
#define KEY_PRESSED_SHOOT_TRIGGER_SPEED_UP  'Z'

//降低射频  长按ctrl+X
#define KEY_PRESSED_SHOOT_TRIGGER_SPEED_DOWN 'X'


#endif