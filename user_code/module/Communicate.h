#ifndef COMMUNICAT_H
#define COMMUNICAT_H

#include "cmsis_os.h"
#include "main.h"

#include "Remote_control.h"
#include "Can_receive.h"

typedef enum
{
    LEFT_FRIC = 0,
    RIGHT_FIRC,
    trigger,
    magezine,
    YAW,
    PITCH,
} gimbal_motor_id;




class Communicate
{
public:
    void init();

    void receive();

    void run();


};

extern Remote_control remote_control;

extern Can_receive can_receive;

extern Communicate communicate;


#endif

