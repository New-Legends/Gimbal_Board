#ifndef SUPER_CAP_H
#define SUPER_CAP_H

#include "main.h"


class Super_Cap {
public:

    float input_vot;      //输入电压
    float cap_vot;        //超级电容电压
    float input_current;  //输入电流
    float target_power;   //目标功率
    bool  cap_change;        //超电电压过低标识符
    
    void init();
    void cap_read_data(float _input_vot, float _cap_vot, float _input_current, float _target_power);
    void read_cap_buff(float *cap_buff);
};


#endif 