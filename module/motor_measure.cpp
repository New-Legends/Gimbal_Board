//
// Created by WSJ on 2021/11/2.
//

#include "motor_measure.h"

void motor_measure::get_motor_measure(const uint8_t *data) {
    last_ecd = ecd;
    ecd = (uint16_t) ((data)[0] << 8 | (data)[1]);
    speed_rpm = (uint16_t) ((data)[2] << 8 | (data)[3]);
    given_current = (uint16_t) ((data)[4] << 8 | (data)[5]);
    temperate = (data)[6];
}
