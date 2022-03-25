#ifndef FEED_FORDWARD_H
#define FEED_FORDWARD_H

#include "struct_typedef.h"
//前向反馈
class Feed_forward
{
public:
    fp32 para[3];        //参数
    uint8_t open_flag;
    fp32 out;

    void init(uint8_t _open_flag, fp32 *_para);
    fp32 calc(fp32 input);  //输出函数
};

#endif 
