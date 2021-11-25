//
// Created by WSJ on 2021/11/2.
//

#include "remote_control.h"
#include <cstring>

#include "main.h"
#include "bsp_usart.h"
#include "remote_data.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

remote_control::remote_control(){
    data = &rc_ctrl;
}

void remote_control::RC_unable() {
    __HAL_UART_DISABLE(&huart3);
}

void remote_control::RC_restart(uint16_t dma_buf_num) {
    __HAL_UART_DISABLE(&huart3);
    __HAL_DMA_DISABLE(&hdma_usart3_rx);

    hdma_usart3_rx.Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_usart3_rx);
    __HAL_UART_ENABLE(&huart3);
}

/**
  * @brief          判断遥控器数据是否出错
  * @param[in]      none
  * @retval         none
  */
uint8_t remote_control::RC_data_is_error() const {
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (RC_abs(data->rc.ch[0]) > RC_CHANNAL_ERROR_VALUE) {
        goto error;
    }
    if (RC_abs(data->rc.ch[1]) > RC_CHANNAL_ERROR_VALUE) {
        goto error;
    }
    if (RC_abs(data->rc.ch[2]) > RC_CHANNAL_ERROR_VALUE) {
        goto error;
    }
    if (RC_abs(data->rc.ch[3]) > RC_CHANNAL_ERROR_VALUE) {
        goto error;
    }
    if (data->rc.s[0] == 0) {
        goto error;
    }
    if (data->rc.s[1] == 0) {
        goto error;
    }
    return 0;

    error:
    data->rc.ch[0] = 0;
    data->rc.ch[1] = 0;
    data->rc.ch[2] = 0;
    data->rc.ch[3] = 0;
    data->rc.ch[4] = 0;
    data->rc.s[0] = RC_SW_DOWN;
    data->rc.s[1] = RC_SW_DOWN;
    data->mouse.x = 0;
    data->mouse.y = 0;
    data->mouse.z = 0;
    data->mouse.press_l = 0;
    data->mouse.press_r = 0;
    data->key.v = 0;
    return 1;
}

/**
  * @brief          解决遥控器丢失的情况(直接重启)
  * @param[in]      none
  * @retval         none
  */
void remote_control::slove_RC_lost() {
    RC_restart(SBUS_RX_BUF_NUM);
}

/**
  * @brief          解决遥控器数据错误的情况(直接重启)
  * @param[in]      none
  * @retval         none
  */
void remote_control::slove_data_error() {
    RC_restart(SBUS_RX_BUF_NUM);
}

/**
  * @brief          取正函数
  * @param[in]      value
  * @retval         取正后的数
  */
int16_t remote_control::RC_abs(int16_t value) {
    if (value > 0) {
        return value;
    } else {
        return -value;
    }
}



