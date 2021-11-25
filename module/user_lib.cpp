//
// Created by WSJ on 2021/11/2.
//

#include "user_lib.h"

//循环限幅函数
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue) {
    if (maxValue < minValue) {
        return Input;
    }

    if (Input > maxValue) {
        fp32 len = maxValue - minValue;
        while (Input > maxValue) {
            Input -= len;
        }
    } else if (Input < minValue) {
        fp32 len = maxValue - minValue;
        while (Input < minValue) {
            Input += len;
        }
    }
    return Input;
}

//限幅函数
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue) {
    if (Value < minValue)
        return
                minValue;
    else if (Value > maxValue)
        return
                maxValue;
    else
        return
                Value;
}

void abs_limit(fp32 Value, fp32 MaxValue){
    if (Value > MaxValue){
        Value = MaxValue;
    }
}