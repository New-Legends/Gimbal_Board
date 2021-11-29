/**
  *************************(C) COPYRIGHT 2021 SUMMERPRAY************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             主要利用陀螺仪bmi088，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过bmi088的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     NOV-06-2021     summerpray      1. doing
  *
  @verbatim
  ==============================================================================
  ==============================================================================
 *      ┌─┐       ┌─┐
 *   ┌──┘ ┴───────┘ ┴──┐
 *   │                 │
 *   │       ───       │
 *   │  ─┬┘       └┬─  │
 *   │                 │
 *   │       ─┴─       │
 *   │                 │
 *   └───┐         ┌───┘
 *       │         │
 *       │         │
 *       │         │
 *       │         └──────────────┐
 *       │                        │
 *       │                        ├─┐
 *       │                        ┌─┘
 *       │                        │
 *       └─┐  ┐  ┌───────┬──┐  ┌──┘
 *         │ ─┤ ─┤       │ ─┤ ─┤
 *         └──┴──┘       └──┴──┘
 *                神兽保佑
 *               代码无BUG!
  ==============================================================================
  ==============================================================================
  @endverbatim
  *************************(C) COPYRIGHT 2021 SUMMERPRAY************************
  */

#include "INS.h"

extern SPI_HandleTypeDef hspi1;

using namespace std;

//实例化对象
INS imu;

/**********************************************(C) 标志位 **************************************************/
volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0; //IMU读取数据标志位
/**********************************************(C) 标志位 **************************************************/
TaskHandle_t INS_task_local_handler; //任务句柄

static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2, 0xFF, 0xFF, 0xFF};

fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];

fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];

fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
fp32 mag_offset[3];

/**
  * @brief          imu任务, 初始化 bmi088, ist8310, 计算欧拉角
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void INS::init(void)
{

    memset(INS_gyro, 0.0f, sizeof(INS_gyro));
    memset(INS_accel, 0.0f, sizeof(INS_accel));
    memset(INS_mag, 0.0f, sizeof(INS_mag));
    memset(INS_quat, 0.0f, sizeof(INS_quat));
    memset(INS_angle, 0.0f, sizeof(INS_angle));
    memset(accel_fliter_1, 0.0f, sizeof(accel_fliter_1));
    memset(accel_fliter_2, 0.0f, sizeof(accel_fliter_2));
    memset(accel_fliter_3, 0.0f, sizeof(accel_fliter_3));

    timing_time = 0.001f;

    osDelay(INS_TASK_INIT_TIME);
    while (BMI088_init())
    {
        osDelay(100);
    }
    while (ist8310_init())
    {
        osDelay(100);
    }
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    //旋转零点漂移
    imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);

    AHRS_init(INS_quat, INS_accel, INS_mag);

    accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];

    //获取当前任务的任务句柄，
    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

    //设置串行接口
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }

    SPI1_DMA_init((unsigned int)gyro_dma_tx_buf, (unsigned int)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    imu_start_dma_flag = 1;
    
}

void INS::INS_Info_Get()
{
    //等待SPI DMA传输
    while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
    {
    }

    if (gyro_update_flag & (1 << IMU_NOTIFY_SHFITS))
    {
        gyro_update_flag &= ~(1 << IMU_NOTIFY_SHFITS);
        BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
    }

    if (accel_update_flag & (1 << IMU_UPDATE_SHFITS))
    {
        accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
        BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
    }

    //TODO:这个任务是监测加速度计的温度，暂时没写

    if (accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
    {
        accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
        BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
        //imu_temp_control(bmi088_real_data.temp);
    }

    //旋转零点漂移
    imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);

    //加速度计低通滤波
    //accel low-pass filter
    accel_fliter_1[0] = accel_fliter_2[0];
    accel_fliter_2[0] = accel_fliter_3[0];

    accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

    accel_fliter_1[1] = accel_fliter_2[1];
    accel_fliter_2[1] = accel_fliter_3[1];

    accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

    accel_fliter_1[2] = accel_fliter_2[2];
    accel_fliter_2[2] = accel_fliter_3[2];

    accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];

    AHRS_update(INS_quat, timing_time, INS_gyro, accel_fliter_3, INS_mag);
    get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);

    //because no use ist8310 and save time, no use
    if (mag_update_flag &= 1 << IMU_DR_SHFITS)
    {
        mag_update_flag &= ~(1 << IMU_DR_SHFITS);
        mag_update_flag |= (1 << IMU_SPI_SHFITS);
        //            ist8310_read_mag(ist8310_real_data.mag);
    }
}

/*********************************************(C) 陀螺仪校准 *************************************************/

void INS::imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310)
{

    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
        mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1] + ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
    }
}

/*********************************************(C) 陀螺仪校准 *************************************************/

/*******************************************(C) 陀螺仪返回参数 ***********************************************/

/**
  * @brief          获取四元数
  * @param[in]      none
  * @retval         INS_quat的指针
  */
const fp32 *INS::get_INS_quat_point(void)
{
    return INS_quat;
}

/**
  * @brief          获取欧拉角, 0:yaw, 1:pitch, 2:roll 单位 rad
  * @param[in]      none
  * @retval         INS_angle的指针
  */
const fp32 *INS::get_INS_angle_point(void)
{
    return INS_angle;
}

/**
  * @brief          获取角速度,0:x轴, 1:y轴, 2:roll轴 单位 rad/s
  * @param[in]      none
  * @retval         INS_gyro的指针
  */
const fp32 *INS::get_gyro_data_point(void)
{
    return INS_gyro;
}

/**
  * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 m/s2
  * @param[in]      none
  * @retval         INS_accel的指针
  */
const fp32 *INS::get_accel_data_point(void)
{
    return INS_accel;
}



/*******************************************(C) 陀螺仪返回参数 ***********************************************/

#ifdef __cplusplus
extern "C"
{
#endif

    /**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
    /**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
    static void imu_cmd_spi_dma(void)
    {
        UBaseType_t uxSavedInterruptStatus;
        uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

        //开启陀螺仪的DMA传输
        if ((gyro_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
            gyro_update_flag |= (1 << IMU_SPI_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
            taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
            return;
        }
        //开启加速度计的DMA传输
        if ((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            accel_update_flag &= ~(1 << IMU_DR_SHFITS);
            accel_update_flag |= (1 << IMU_SPI_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
            taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
            return;
        }

        if ((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
            accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
            taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
            return;
        }
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    }

    void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
    {
        if (GPIO_Pin == INT1_ACCEL_Pin)
        {
            //detect_hook(BOARD_ACCEL_TOE);
            accel_update_flag |= 1 << IMU_DR_SHFITS;
            accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
            if (imu_start_dma_flag)
            {
                imu_cmd_spi_dma();
            }
        }
        else if (GPIO_Pin == INT1_GYRO_Pin)
        {
            //detect_hook(BOARD_GYRO_TOE);
            gyro_update_flag |= 1 << IMU_DR_SHFITS;
            if (imu_start_dma_flag)
            {
                imu_cmd_spi_dma();
            }
        }
        else if (GPIO_Pin == DRDY_IST8310_Pin)
        {
            //detect_hook(BOARD_MAG_TOE);
            mag_update_flag |= 1 << IMU_DR_SHFITS;
        }
        else if (GPIO_Pin == GPIO_PIN_0)
        {

            //wake up the task
            //唤醒任务
            if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
            {
                static BaseType_t xHigherPriorityTaskWoken;
                vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
    }

    void DMA2_Stream2_IRQHandler(void)
    {

        if (__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
        {
            __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

            //gyro read over
            //陀螺仪读取完毕
            if (gyro_update_flag & (1 << IMU_SPI_SHFITS))
            {
                gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
                gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

                HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
            }

            //accel read over
            //加速度计读取完毕
            if (accel_update_flag & (1 << IMU_SPI_SHFITS))
            {
                accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
                accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

                HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
            }
            //temperature read over
            //温度读取完毕
            if (accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
            {
                accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
                accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

                HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
            }

            imu_cmd_spi_dma();

            if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
            {
                gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
                gyro_update_flag |= (1 << IMU_NOTIFY_SHFITS);
                __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
            }
        }
    }

#ifdef __cplusplus
}
#endif
