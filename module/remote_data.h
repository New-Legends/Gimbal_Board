#ifndef BSP_RC_H
#define BSP_RC_H

#include "struct_typedef.h"
#include "bsp_usart.h"
#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u
//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700
#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

//官方遥控器数据包
typedef struct {
    struct {
        int16_t ch[5];
        char s[2];
    }__attribute__((packed)) rc;
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    }__attribute__((packed)) mouse;
    struct {
        uint16_t v;//value:官方键盘开放的键位
    }__attribute__((packed)) key;

}__attribute__((packed)) RC_ctrl_t;


extern void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

extern void remote_control_init();

void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
RC_ctrl_t *get_remote_control_point();

#endif
