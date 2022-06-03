#include "communicate.h"

#include "main.h"
#include "string.h"


#include "bsp_usart.h"
#include "bsp_led.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "CRC8_CRC16.h"
#include "fifo.h"

#ifdef __cplusplus
}
#endif

#include "Chassis.h"
#include "detect_task.h"

#include "Remote_control.h"
#include "Can_receive.h"
#include "Referee.h"
#include "Ui.h"

Remote_control remote_control;
Can_receive can_receive;
Referee referee;
Ui      ui;

Communicate communicate;

void Communicate::init()
{

#if CHASSIS_REMOTE_OPEN
    remote_control.init();
#else
    ;
#endif

    can_receive.init();

    referee.init();

#if UI_OPEN
    ui.init(&referee.Judge_Self_ID, &referee.Judge_SelfClient_ID);
#endif
}

void Communicate::run()
{
    referee.unpack();
    referee.determine_ID();

#if UI_OPEN
    ui.run();
#endif

    //向云台发送裁判数据
    uint16_t temp_id1_17mm_cooling_limit, temp_id1_17mm_cooling_rate, temp_id1_17mm_cooling_heat;
    uint8_t temp_color, temp_robot_id;
    uint16_t temp_id1_17mm_speed_limit;
    fp32 temp_bullet_speed;
    uint8_t temp_chassis_behaviour_mode;

    referee.get_shooter_id1_17mm_cooling_limit_and_heat(&temp_id1_17mm_cooling_limit, &temp_id1_17mm_cooling_heat);
    referee.get_shooter_id1_17mm_cooling_rate(&temp_id1_17mm_cooling_rate);
    referee.get_color(&temp_color);
    referee.get_robot_id(&temp_robot_id);
    referee.get_shooter_id1_17mm_speed_limit_and_bullet_speed(&temp_id1_17mm_speed_limit, &temp_bullet_speed);
    temp_chassis_behaviour_mode = chassis.chassis_behaviour_mode;


    can_receive.send_cooling_and_id_board_com(temp_id1_17mm_cooling_limit, temp_id1_17mm_cooling_rate, temp_id1_17mm_cooling_heat,
                                              temp_color, temp_robot_id);

    can_receive.send_17mm_speed_and_mode_board_com(temp_id1_17mm_speed_limit, temp_bullet_speed, temp_chassis_behaviour_mode);


    cap.cap_read_data(can_receive.cap_receive.input_vot, can_receive.cap_receive.cap_vot, can_receive.cap_receive.input_current,can_receive.cap_receive.target_power);
//TODO _data这里最好使用指针赋值,减少计算量,后续需修改
#if CHASSIS_REMOTE_OPEN
    ;
#else
    //保留上一次遥控器值
    remote_control.last_rc_ctrl = remote_control.rc_ctrl;

    remote_control.rc_ctrl.rc.ch[0] = can_receive.chassis_receive.ch_0;
    remote_control.rc_ctrl.rc.ch[2] = can_receive.chassis_receive.ch_2;
    remote_control.rc_ctrl.rc.ch[3] = can_receive.chassis_receive.ch_3;
    remote_control.rc_ctrl.key.v = can_receive.chassis_receive.v;
    remote_control.rc_ctrl.rc.s[0] = can_receive.chassis_receive.s0;

#endif
}

#ifdef __cplusplus //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
extern "C"
{

    void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
    {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        if (hcan == &CHASSIS_CAN) //接底盘CAN 信息
        {

        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
        switch (rx_header.StdId)
        {
        //底盘动力电机
        case CAN_MOTIVE_FR_MOTOR_ID:
            can_receive.get_motive_motor_measure(MOTIVE_FR_MOTOR, rx_data);
            //detect_hook(CHASSIS_MOTIVE_FR_MOTOR_TOE);
            break;
        case CAN_MOTIVE_FL_MOTOR_ID:
            can_receive.get_motive_motor_measure(MOTIVE_FL_MOTOR, rx_data);
            //detect_hook(CHASSIS_MOTIVE_FL_MOTOR_TOE);
            break;
        case CAN_MOTIVE_BL_MOTOR_ID:
            can_receive.get_motive_motor_measure(MOTIVE_BL_MOTOR, rx_data);
            //detect_hook(CHASSIS_MOTIVE_BL_MOTOR_TOE);
            break;
        case CAN_MOTIVE_BR_MOTOR_ID:
            can_receive.get_motive_motor_measure(MOTIVE_BR_MOTOR, rx_data);
            //detect_hook(CHASSIS_MOTIVE_BR_MOTOR_TOE);
            break;

        //底盘舵向电机
        case CAN_RUDDER_FR_MOTOR_ID:
            can_receive.get_rudder_motor_measure(RUDDER_FR_MOTOR, rx_data);
            //detect_hook(CHASSIS_RUDDER_FR_MOTOR_TOE);
            break;
        case CAN_RUDDER_FL_MOTOR_ID:
            can_receive.get_rudder_motor_measure(RUDDER_FL_MOTOR, rx_data);
            //detect_hook(CHASSIS_RUDDER_FL_MOTOR_TOE);
            break;
        case CAN_RUDDER_BL_MOTOR_ID:
            can_receive.get_rudder_motor_measure(RUDDER_BL_MOTOR, rx_data);
            //detect_hook(CHASSIS_RUDDER_BL_MOTOR_TOE);
            break;
        case CAN_RUDDER_BR_MOTOR_ID:
            can_receive.get_rudder_motor_measure(RUDDER_BR_MOTOR, rx_data);
            //detect_hook(CHASSIS_RUDDER_BR_MOTOR_TOE);
            break;
        case CAN_SUPER_CAP_ID:
            can_receive.get_super_cap_data(rx_data);
            
            break;

        default:
        {
            break;
        }
        }


        }
        else if (hcan == &BOARD_COM_CAN) //接底盘CAN 信息
        {
            HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
            switch (rx_header.StdId)
            {

            case CAN_RC_BOARM_COM_ID:
                can_receive.receive_rc_board_com(rx_data);
                //detect_hook(BOARD_COM);
                break;

            case CAN_GIMBAL_BOARD_COM_ID:
                can_receive.receive_gimbal_board_com(rx_data);
                //detect_hook(BOARD_COM);
                break;

            default:
            {
                break;
            }


            }


        }
        }

    //遥控器串口
    void USART3_IRQHandler(void)
    {
        if (huart3.Instance->SR & UART_FLAG_RXNE) //接收到数据
        {
            __HAL_UART_CLEAR_PEFLAG(&huart3);
        }
        else if (USART3->SR & UART_FLAG_IDLE)
        {
            static uint16_t this_time_rx_len = 0;

            __HAL_UART_CLEAR_PEFLAG(&huart3);

            if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
            {
                /* Current memory buffer used is Memory 0 */

                //disable DMA
                //失效DMA
                __HAL_DMA_DISABLE(&hdma_usart3_rx);

                //get receive data length, length = set_data_length - remain_length
                //获取接收数据长度,长度 = 设定长度 - 剩余长度
                this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

                //reset set_data_lenght
                //重新设定数据长度
                hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

                //set memory buffer 1
                //设定缓冲区1
                hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

                //enable DMA
                //使能DMA
                __HAL_DMA_ENABLE(&hdma_usart3_rx);

                if (this_time_rx_len == RC_FRAME_LENGTH)
                {
                    remote_control.unpack(0);
                    //记录数据接收时间
                    detect_hook(DBUS_TOE);
                    remote_control.sbus_to_usart1(0);
                }
            }
            else
            {
                /* Current memory buffer used is Memory 1 */
                //disable DMA
                //失效DMA
                __HAL_DMA_DISABLE(&hdma_usart3_rx);

                //get receive data length, length = set_data_length - remain_length
                //获取接收数据长度,长度 = 设定长度 - 剩余长度
                this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

                //reset set_data_lenght
                //重新设定数据长度
                hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

                //set memory buffer 0
                //设定缓冲区0
                DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

                //enable DMA
                //使能DMA
                __HAL_DMA_ENABLE(&hdma_usart3_rx);

                if (this_time_rx_len == RC_FRAME_LENGTH)
                {
                    //处理遥控器数据
                    remote_control.unpack(1);
                    //记录数据接收时间
                    detect_hook(DBUS_TOE);
                    remote_control.sbus_to_usart1(1);
                }
            }
        }
    }

    //裁判串口数据
    void USART6_IRQHandler(void)
    {
        static volatile uint8_t res;
        if (USART6->SR & UART_FLAG_IDLE)
        {
            __HAL_UART_CLEAR_PEFLAG(&huart6);

            static uint16_t this_time_rx_len = 0;

            if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
            {
                __HAL_DMA_DISABLE(huart6.hdmarx);
                this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
                __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
                huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
                __HAL_DMA_ENABLE(huart6.hdmarx);
                fifo_s_puts(&referee.referee_fifo, (char *)(referee.usart6_buf[0]), this_time_rx_len);
                detect_hook(REFEREE_TOE);
            } 
            else
            {
                __HAL_DMA_DISABLE(huart6.hdmarx);
                this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
                __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
                huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
                __HAL_DMA_ENABLE(huart6.hdmarx);
                fifo_s_puts(&referee.referee_fifo, (char *)(referee.usart6_buf[1]), this_time_rx_len);
                detect_hook(REFEREE_TOE);
            }
        }
    }
}
#endif