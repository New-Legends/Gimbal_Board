/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "struct_typedef.h"
#include "bsp_rc.h"




#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)




/* ----------------------- Data Struct ------------------------------------- */
typedef __packed struct
{
        __packed struct
        {
                int16_t ch[5];
                char s[2];
        } rc;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __packed struct
        {
                uint16_t v;
        } key;

} RC_ctrl_t;

extern RC_ctrl_t rc_ctrl;
extern RC_ctrl_t last_rc_ctrl;


/*
        记录当前鼠标值
*/
#define    IF_MOUSE_PRESSED_L         (  rc_ctrl.mouse.press_l !=0)
#define    IF_MOUSE_PRESSED_R         (  rc_ctrl.mouse.press_r !=0)



/*
        记录当前鼠标值
*/
#define    LAST_IF_MOUSE_PRESSED_L         (  last_rc_ctrl.mouse.press_l !=0)
#define    LAST_IF_MOUSE_PRESSED_R         (  last_rc_ctrl.mouse.press_r !=0)

/* 
        单击鼠标
*/
#define IF_MOUSE_SINGAL_PRESSED_L          (IF_MOUSE_PRESSED_L && !LAST_IF_MOUSE_PRESSED_L)
#define IF_MOUSE_SINGAL_PRESSED_R          (IF_MOUSE_PRESSED_R && !LAST_IF_MOUSE_PRESSED_R)


/* 
        记录当前值
        检测键盘按键状态   
        若对应按键被按下，则逻辑表达式的值为1，否则为0 */
#define    IF_KEY_PRESSED         (  rc_ctrl.key.v  )
#define    IF_KEY_PRESSED_W       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_W)    != 0 )
#define    IF_KEY_PRESSED_S       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_S)    != 0 )
#define    IF_KEY_PRESSED_A       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_A)    != 0 )
#define    IF_KEY_PRESSED_D       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_D)    != 0 )
#define    IF_KEY_PRESSED_Q       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)    != 0 )
#define    IF_KEY_PRESSED_E       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)    != 0 )
#define    IF_KEY_PRESSED_G       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_G)    != 0 )
#define    IF_KEY_PRESSED_X       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_X)    != 0 )
#define    IF_KEY_PRESSED_Z       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z)    != 0 )
#define    IF_KEY_PRESSED_C       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_C)    != 0 )
#define    IF_KEY_PRESSED_B       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_B)    != 0 )
#define    IF_KEY_PRESSED_V       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_V)    != 0 )
#define    IF_KEY_PRESSED_F       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_F)    != 0 )
#define    IF_KEY_PRESSED_R       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_R)    != 0 )
#define    IF_KEY_PRESSED_CTRL    ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL) != 0 )
#define    IF_KEY_PRESSED_SHIFT   ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT) != 0 )

/*
由于各任务线程的时间不同,所以更新的遥控器指针也不同,暂时还没有想到解决这个问题的方法,所以全局的last_rc_ctrl暂时不用
,代替的方案是每个需要遥控器指令的任务自带一个遥控器指针,随自身任务更新周期更新.
*/

/* 
        记录上一次值
        检测键盘按键状态 
        若对应按键被按下，则逻辑表达式的值为1，否则为0 */
#define LAST_KEY_PRESSED (last_rc_ctrl.key.v)
#define    LAST_IF_KEY_PRESSED_W       ( (last_rc_ctrl.key.v & KEY_PRESSED_OFFSET_W)    != 0 )
#define    LAST_IF_KEY_PRESSED_S       ( (last_rc_ctrl.key.v & KEY_PRESSED_OFFSET_S)    != 0 )
#define    LAST_IF_KEY_PRESSED_A       ( (last_rc_ctrl.key.v & KEY_PRESSED_OFFSET_A)    != 0 )
#define    LAST_IF_KEY_PRESSED_D       ( (last_rc_ctrl.key.v & KEY_PRESSED_OFFSET_D)    != 0 )
#define    LAST_IF_KEY_PRESSED_Q       ( (last_rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)    != 0 )
#define    LAST_IF_KEY_PRESSED_E       ( (last_rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)    != 0 )
#define    LAST_IF_KEY_PRESSED_G       ( (last_rc_ctrl.key.v & KEY_PRESSED_OFFSET_G)    != 0 )
#define    LAST_IF_KEY_PRESSED_X       ( (last_rc_ctrl.key.v & KEY_PRESSED_OFFSET_X)    != 0 )
#define    LAST_IF_KEY_PRESSED_Z       ( (last_rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z)    != 0 )
#define    LAST_IF_KEY_PRESSED_C       ( (last_rc_ctrl.key.v & KEY_PRESSED_OFFSET_C)    != 0 )
#define    LAST_IF_KEY_PRESSED_B       ( (last_rc_ctrl.key.v & KEY_PRESSED_OFFSET_B)    != 0 )
#define    LAST_IF_KEY_PRESSED_V       ( (last_rc_ctrl.key.v & KEY_PRESSED_OFFSET_V)    != 0 )
#define    LAST_IF_KEY_PRESSED_F       ( (last_rc_ctrl.key.v & KEY_PRESSED_OFFSET_F)    != 0 )
#define    LAST_IF_KEY_PRESSED_R       ( (last_rc_ctrl.key.v & KEY_PRESSED_OFFSET_R)    != 0 )
#define    LAST_IF_KEY_PRESSED_CTRL    ( (last_rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL) != 0 )
#define    LAST_IF_KEY_PRESSED_SHIFT   ( (last_rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT) != 0 )


/* 
        检测是否为单击
        检测键盘按键状态 
        若对应按键被单击，则逻辑表达式的值为1，否则为0 */
#define    IF_KEY_SINGAL_PRESSED_W       ( IF_KEY_PRESSED_W && !LAST_IF_KEY_PRESSED_W )
#define    IF_KEY_SINGAL_PRESSED_S       ( IF_KEY_PRESSED_S && !LAST_IF_KEY_PRESSED_S )
#define    IF_KEY_SINGAL_PRESSED_A       ( IF_KEY_PRESSED_A && !LAST_IF_KEY_PRESSED_A )
#define    IF_KEY_SINGAL_PRESSED_D       ( IF_KEY_PRESSED_D && !LAST_IF_KEY_PRESSED_D )
#define    IF_KEY_SINGAL_PRESSED_Q       ( IF_KEY_PRESSED_Q && !LAST_IF_KEY_PRESSED_Q )
#define    IF_KEY_SINGAL_PRESSED_E       ( IF_KEY_PRESSED_E && !LAST_IF_KEY_PRESSED_E )
#define    IF_KEY_SINGAL_PRESSED_G       ( IF_KEY_PRESSED_G && !LAST_IF_KEY_PRESSED_G )
#define    IF_KEY_SINGAL_PRESSED_X       ( IF_KEY_PRESSED_X && !LAST_IF_KEY_PRESSED_X )
#define    IF_KEY_SINGAL_PRESSED_Z       ( IF_KEY_PRESSED_Z && !LAST_IF_KEY_PRESSED_Z )
#define    IF_KEY_SINGAL_PRESSED_C       ( IF_KEY_PRESSED_C && !LAST_IF_KEY_PRESSED_C )
#define    IF_KEY_SINGAL_PRESSED_B       ( IF_KEY_PRESSED_B && !LAST_IF_KEY_PRESSED_B )
#define    IF_KEY_SINGAL_PRESSED_V       ( IF_KEY_PRESSED_V && !LAST_IF_KEY_PRESSED_V )
#define    IF_KEY_SINGAL_PRESSED_F       ( IF_KEY_PRESSED_F && !LAST_IF_KEY_PRESSED_F )
#define    IF_KEY_SINGAL_PRESSED_R       ( IF_KEY_PRESSED_R && !LAST_IF_KEY_PRESSED_R )
#define    IF_KEY_SINGAL_PRESSED_CTRL    ( IF_KEY_PRESSED_CTRL && !LAST_IF_KEY_PRESSED_CTRL )
#define    IF_KEY_SINGAL_PRESSED_SHIFT   ( IF_KEY_PRESSED_SHIFT && !LAST_IF_KEY_PRESSED_SHIFT )


/* ----------------------- Internal Data ----------------------------------- */



extern void remote_control_init(void);
extern const RC_ctrl_t *get_remote_control_point(void);
extern const RC_ctrl_t *get_last_remote_control_point(void);
extern uint8_t RC_data_is_error(void);
extern void slove_RC_lost(void);
extern void slove_data_error(void);
extern void sbus_to_usart1(uint8_t *sbus);
#endif
