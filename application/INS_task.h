#ifndef INS_Task_H
#define INS_Task_H

#define IMU_CONTROL_TIME_MS 2

extern void INS_task(void *pvParameters);

class IMU_data{
public:
    fp32 *INS_gyro;
    fp32 *INS_angle;

    void init();
};

#endif
