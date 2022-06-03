#include "Super_cap.h"
#include "Communicate.h""
#include "cmsis_os.h"


void Super_Cap::init() {

    for (uint8_t i=0; i<10; i++){
        vTaskDelay(1000);
        can_receive.can_cmd_super_cap_power(5000);
    }
}

void Super_Cap::cap_read_data(float _input_vot, float _cap_vot, float _input_current, float _target_power)
{
    input_vot = _input_vot;
    cap_vot  = _cap_vot;      
    input_current =  _input_current;
    target_power = _target_power;
}

void Super_Cap::read_cap_buff(float *_cap_buff)
{
    /*电容能量公式 E= 1/2*C*U*U
    C为电容容值 U是电容两端的电压
    */
   //子电容数量
   int cap_son_num = 10;
   
   *_cap_buff = 0.5*50*(cap_vot/cap_son_num)*(cap_vot/cap_son_num) * cap_son_num;
}
