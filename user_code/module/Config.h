#ifndef CONFIG_H
#define CONFIG_H

/*----------------------底盘---------------------------*/
//底盘动力电机有电流输出
#define CHASSIS_MOTIVE_MOTOR_HAVE_CURRENT 1
//底盘舵向电机有电流输出
#define CHASSIS_RUDDER_MOTOR_HAVE_CURRENT 1


/*---------------------通信-----------------------------*/
//底盘独立遥控器是否开启 下板通信时关闭设置为0
#define CHASSIS_REMOTE_OPEN 0
//UI 是否开启
#define UI_OPEN 1

/*---------------------按键---------------------------*/
//底盘小陀螺 单击F
#define KEY_PRESSED_CHASSIS_TOP     'F'

//底盘摇摆  单击C
#define KEY_PRESSED_CHASSIS_SWING   'C'

//底盘45度角 单击V
#define KEY_PRESSED_CHASSIS_PISA    'V'

//底盘超级电容加速 单击SHIFT !代表shift
#define KEY_PRESSED_CHASSIS_SUPER_CAP    '!'

//底盘前后左右控制按键
#define KEY_PRESSED_CHASSIS_FRONT  'W'
#define KEY_PRESSED_CHASSIS_BACK   'S'
#define KEY_PRESSED_CHASSIS_LEFT   'A'
#define KEY_PRESSED_CHASSIS_RIGHT  'D'


#endif