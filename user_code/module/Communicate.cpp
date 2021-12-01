#include "communicate.h"

#include "main.h"
#include "string.h"

#include "bsp_usart.h"
#include "bsp_led.h"

#include "Remote_control.h"
#include "Can_receive.h"

#include "detect_task.h"

Remote_control remote_control;
CAN_Gimbal can_receive;

Communicate communicate;

void Communicate::init()
{
    remote_control.init();
    can_receive.init();
}

#ifdef __cplusplus //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
extern "C"
{

    /**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
    void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
        CAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];

        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

        switch (rx_header.StdId)
        {
        case CAN_LEFT_FRIC_MOTOR_ID:
        case CAN_RIGHT_FRIC_MOTOR_ID:
        case CAN_TRIGGER_MOTOR_ID:
        case CAN_YAW_MOTOR_ID:
        case CAN_PIT_MOTOR_ID:
        {
            static uint8_t i = 0;
            i = rx_header.StdId - CAN_LEFT_FRIC_MOTOR_ID;
            Can.motor[i].get_motor_measure(rx_data);
            detect_hook(CAN_LEFT_FRIC_MOTOR_ID + i);
            break;
        }

        default:
        {
            break;
        }
        }
    }
    // TODO 设备检查未更新
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
                    //detect_hook(DBUS_TOE);
                    remote_control.sbus_to_usart1(1);
                }
            }
        }
    }
}
#endif
