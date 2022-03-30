#include "Remote_control.h"

#include "main.h"

#include "bsp_usart.h"


#include "string.h"

void Remote_control::init()
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
    usart1_tx_dma_init();
}

const RC_ctrl_t * Remote_control::get_remote_control_point()
{
    return &rc_ctrl;
}

const RC_ctrl_t *Remote_control::get_last_remote_control_point()
{
    return &last_rc_ctrl;
}



/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_rx_buf[num]: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
void Remote_control::unpack(uint8_t num)
{
    if (sbus_rx_buf[num] == NULL)
    {
        return;
    }

    //保留上一次遥控器值
    last_rc_ctrl = rc_ctrl;

    // if (rc_ctrl_updata >= 50)
    // {
    //     if(rc_ctrl_updata==50)
    //         last_rc_ctrl = *rc_ctrl;             //当遥控器有输入时保留上一次数据
    //     rc_ctrl_updata++;
    //     if(rc_ctrl_updata >= 100)
    //      rc_ctrl_updata = 0;
    // }
    // else
    //     rc_ctrl_updata ++;

    rc_ctrl.rc.ch[0] = (sbus_rx_buf[num][0] | (sbus_rx_buf[num][1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl.rc.ch[1] = ((sbus_rx_buf[num][1] >> 3) | (sbus_rx_buf[num][2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl.rc.ch[2] = ((sbus_rx_buf[num][2] >> 6) | (sbus_rx_buf[num][3] << 2) |          //!< Channel 2
                       (sbus_rx_buf[num][4] << 10)) &
                        0x07ff;
    rc_ctrl.rc.ch[3] = ((sbus_rx_buf[num][4] >> 1) | (sbus_rx_buf[num][5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl.rc.s[0] = ((sbus_rx_buf[num][5] >> 4) & 0x0003);                       //!< Switch left
    rc_ctrl.rc.s[1] = ((sbus_rx_buf[num][5] >> 4) & 0x000C) >> 2;                  //!< Switch right
    rc_ctrl.mouse.x = sbus_rx_buf[num][6] | (sbus_rx_buf[num][7] << 8);                    //!< Mouse X axis
    rc_ctrl.mouse.y = sbus_rx_buf[num][8] | (sbus_rx_buf[num][9] << 8);                    //!< Mouse Y axis
    rc_ctrl.mouse.z = sbus_rx_buf[num][10] | (sbus_rx_buf[num][11] << 8);                  //!< Mouse Z axis
    rc_ctrl.mouse.press_l = sbus_rx_buf[num][12];                                  //!< Mouse Left Is Press ?
    rc_ctrl.mouse.press_r = sbus_rx_buf[num][13];                                  //!< Mouse Right Is Press ?
    rc_ctrl.key.v = sbus_rx_buf[num][14] | (sbus_rx_buf[num][15] << 8);                    //!< KeyBoard value
    rc_ctrl.rc.ch[4] = sbus_rx_buf[num][16] | (sbus_rx_buf[num][17] << 8);                 //NULL

    rc_ctrl.rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

/**
  * @brief          通过usart1发送sbus数据,在usart3_IRQHandle调用
  * @param[in]      sbus: sbus数据, 18字节
  * @retval         none
  */
void Remote_control::sbus_to_usart1(uint8_t num)
{
    static uint8_t usart_tx_buf[20];
    static uint8_t i = 0;
    usart_tx_buf[0] = 0xA6;
    memcpy(usart_tx_buf + 1, sbus_rx_buf[num], 18);
    for (i = 0, usart_tx_buf[19] = 0; i < 19; i++)
    {
        usart_tx_buf[19] += usart_tx_buf[i];
    }
    usart1_tx_dma_enable(usart_tx_buf, 20);
}

uint8_t Remote_control::RC_data_is_error()
{
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = RC_SW_DOWN;
    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;
}



//取正函数
int16_t Remote_control::RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}

void Remote_control::slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}

void Remote_control::slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}


/*-----相比于官方的版本,将宏定义布尔值转化为可传参的函数,方便不同任务内的遥控器调用按键--*/

//是否按下鼠标
bool_t if_mouse_pessed(const RC_ctrl_t *_rc_ctrl, char mouse_num)
{
    bool_t ans = FALSE;
    if (mouse_num == 'L')
        ans = _rc_ctrl->mouse.press_l != 0;
    else if (mouse_num == 'R')
        ans = _rc_ctrl->mouse.press_r != 0;

    return ans;
}

//是否单击鼠标
bool_t if_mouse_singal_pessed(const RC_ctrl_t *_rc_ctrl, const RC_ctrl_t *_last_rc_ctrl, char mouse_num)
{
    bool_t ans = FALSE;
    if (mouse_num == 'L')
        ans = if_mouse_pessed(_rc_ctrl, 'L') && if_mouse_pessed(_last_rc_ctrl, 'L');
    else if (mouse_num == 'R')
        ans = if_mouse_pessed(_rc_ctrl, 'R') && if_mouse_pessed(_last_rc_ctrl, 'R');

    return ans;
}

//是否按下对应按键
bool_t if_key_pessed(const RC_ctrl_t *_rc_ctrl, char key_num)
{
    bool_t ans = FALSE;

    switch (key_num)
    {
    case 'W':
                ans =  ( (_rc_ctrl->key.v & KEY_PRESSED_OFFSET_W) != 0);
                break;

    case 'S':
                ans =  ( (_rc_ctrl->key.v & KEY_PRESSED_OFFSET_S) != 0);
                break;

    case 'A':
                ans =  ( (_rc_ctrl->key.v & KEY_PRESSED_OFFSET_A) != 0);
                break;

    case 'D':
                ans =  ( (_rc_ctrl->key.v & KEY_PRESSED_OFFSET_D) != 0);
                break;

    case 'Q':
                ans =  ( (_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Q) != 0);
                break;

    case 'E':
                ans =  ( (_rc_ctrl->key.v & KEY_PRESSED_OFFSET_E) != 0);
                break;

    case 'G':
                ans =  ( (_rc_ctrl->key.v & KEY_PRESSED_OFFSET_G) != 0);
                break;

    case 'X':
                ans =  ( (_rc_ctrl->key.v & KEY_PRESSED_OFFSET_X) != 0);
                break;

    case 'Z':
                ans =  ( (_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Z) != 0);
                break;

    case 'C':
                ans =  ( (_rc_ctrl->key.v & KEY_PRESSED_OFFSET_C) != 0);
                break;

    case 'B':
                ans =  ( (_rc_ctrl->key.v & KEY_PRESSED_OFFSET_B) != 0);
                break;

    case 'V':
                ans =  ( (_rc_ctrl->key.v & KEY_PRESSED_OFFSET_V) != 0);
                break;

    case 'F':
                ans =  ( (_rc_ctrl->key.v & KEY_PRESSED_OFFSET_F) != 0);
                break;

    case 'R':
                ans =  ( (_rc_ctrl->key.v & KEY_PRESSED_OFFSET_R) != 0);
                break;

    case '$':
                ans =  ( (_rc_ctrl->key.v & KEY_PRESSED_OFFSET_CTRL) != 0);
                break;

    case '!':
                ans =  ( (_rc_ctrl->key.v & KEY_PRESSED_OFFSET_SHIFT) != 0);
                break;

    default:
        break;
    }

    return ans;
}

//是否单击对于按键
bool_t if_key_singal_pessed(const RC_ctrl_t *_rc_ctrl, const RC_ctrl_t *_last_rc_ctrl, char key_num)
{
    bool_t ans = FALSE;

    switch (key_num)
    {
    case 'W':
        ans = if_key_pessed(_rc_ctrl, 'W') && !if_key_pessed(_last_rc_ctrl, 'W');
        break;

    case 'S':
        ans = if_key_pessed(_rc_ctrl, 'S') && !if_key_pessed(_last_rc_ctrl, 'S');
        break;

    case 'A':
        ans = if_key_pessed(_rc_ctrl, 'A') && !if_key_pessed(_last_rc_ctrl, 'A');
        break;

    case 'D':
        ans = if_key_pessed(_rc_ctrl, 'D') && !if_key_pessed(_last_rc_ctrl, 'D');
        break;

    case 'Q':
        ans = if_key_pessed(_rc_ctrl, 'Q') && !if_key_pessed(_last_rc_ctrl, 'Q');
        break;

    case 'E':
        ans = if_key_pessed(_rc_ctrl, 'E') && !if_key_pessed(_last_rc_ctrl, 'E');
        break;

    case 'G':
        ans = if_key_pessed(_rc_ctrl, 'G') && !if_key_pessed(_last_rc_ctrl, 'G');
        break;

    case 'X':
        ans = if_key_pessed(_rc_ctrl, 'X') && !if_key_pessed(_last_rc_ctrl, 'X');
        break;

    case 'Z':
        ans = if_key_pessed(_rc_ctrl, 'Z') && !if_key_pessed(_last_rc_ctrl, 'Z');
        break;

    case 'C':
        ans = if_key_pessed(_rc_ctrl, 'C') && !if_key_pessed(_last_rc_ctrl, 'C');
        break;

    case 'B':
        ans = if_key_pessed(_rc_ctrl, 'B') && !if_key_pessed(_last_rc_ctrl, 'B');
        break;

    case 'V':
        ans = if_key_pessed(_rc_ctrl, 'V') && !if_key_pessed(_last_rc_ctrl, 'V');
        break;

    case 'F':
        ans = if_key_pessed(_rc_ctrl, 'F') && !if_key_pessed(_last_rc_ctrl, 'F');
        break;

    case 'R':
        ans = if_key_pessed(_rc_ctrl, 'R') && !if_key_pessed(_last_rc_ctrl, 'R');
        break;

    case '$':
        ans = if_key_pessed(_rc_ctrl, '$') && !if_key_pessed(_last_rc_ctrl, '$');
        break;

    case '!':
        ans = if_key_pessed(_rc_ctrl, '!') && !if_key_pessed(_last_rc_ctrl, '!');
        break;

    default:
        break;
    }

    return ans;
}
