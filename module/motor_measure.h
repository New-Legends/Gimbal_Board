//
// Created by WSJ on 2021/11/2.
//

#ifndef CLASSIS_BOARD_MOTOR_MEASURE_H
#define CLASSIS_BOARD_MOTOR_MEASURE_H

#ifdef __cplusplus
#include "struct_typedef.h"
#include "main.h"



class motor_measure {
public:
    uint16_t ecd;
    uint16_t speed_rpm;
    uint16_t given_current;
    uint8_t temperate;
    uint16_t last_ecd;

    void get_motor_measure(const uint8_t *data);

};



#endif
#endif //CLASSIS_BOARD_MOTOR_MEASURE_H
