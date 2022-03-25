#include "Feed_forward.h"

void Feed_forward::init(uint8_t _open_flag, fp32 *_para)
{
    open_flag = _open_flag;
    para[0] = _para[0];
    para[1] = _para[1];
    para[2] = _para[2];
}


fp32 Feed_forward::calc(fp32 input)
{
    out = para[0] * open_flag * (para[1] * input + para[2]);
    return out;
}
