#ifndef BSP_RC_H
#define BSP_RC_H
#ifdef __cplusplus             //���߱��������ⲿ�ִ��밴C���Եĸ�ʽ���б��룬������C++��
extern "C"{
#include "struct_typedef.h"

extern void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void RC_unable(void);
extern void RC_restart(uint16_t dma_buf_num);
}
#endif
#endif
	
