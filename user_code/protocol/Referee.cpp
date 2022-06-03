#include "Referee.h"


#include "main.h"
#include "bsp_usart.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "CRC8_CRC16.h"
#include "fifo.h" 

#ifdef __cplusplus
}
#endif

extern UART_HandleTypeDef huart6;

unpack_data_t referee_unpack_obj;

void Referee::init(){

    init_referee_struct_data();
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    usart6_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);
}

void Referee::unpack(){

    uint8_t byte = 0;
    uint8_t sof = HEADER_SOF;
    unpack_data_t *p_obj = &referee_unpack_obj;

    while (fifo_s_used(&referee_fifo))
    {
        byte = fifo_s_get(&referee_fifo);
        switch (p_obj->unpack_step)
        {
        case STEP_HEADER_SOF:
        {
            if (byte == sof)
            {
                p_obj->unpack_step = STEP_LENGTH_LOW;
                p_obj->protocol_packet[p_obj->index++] = byte;
            }
            else
            {
                p_obj->index = 0;
            }
        }
        break;

        case STEP_LENGTH_LOW:
        {
            p_obj->data_len = byte;
            p_obj->protocol_packet[p_obj->index++] = byte;
            p_obj->unpack_step = STEP_LENGTH_HIGH;
        }
        break;

        case STEP_LENGTH_HIGH:
        {
            p_obj->data_len |= (byte << 8);
            p_obj->protocol_packet[p_obj->index++] = byte;

            if (p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
            {
                p_obj->unpack_step = STEP_FRAME_SEQ;
            }
            else
            {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;
            }
        }
        break;
        case STEP_FRAME_SEQ:
        {
            p_obj->protocol_packet[p_obj->index++] = byte;
            p_obj->unpack_step = STEP_HEADER_CRC8;
        }
        break;

        case STEP_HEADER_CRC8:
        {
            p_obj->protocol_packet[p_obj->index++] = byte;

            if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
            {
                if (verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE))
                {
                    p_obj->unpack_step = STEP_DATA_CRC16;
                }
                else
                {
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;
                }
            }
        }
        break;

        case STEP_DATA_CRC16:
        {
            if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
            {
                p_obj->protocol_packet[p_obj->index++] = byte;
            }
            if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
            {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;

                if (verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
                {
                    referee_data_solve(p_obj->protocol_packet);
                }
            }
        }
        break;

        default:
        {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
        }
        break;
        }
    }
}





void Referee::init_referee_struct_data(){
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));
    memset(&game_state, 0, sizeof(ext_game_state_t));
    memset(&game_result, 0, sizeof(ext_game_result_t));
    memset(&game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));
    memset(&field_event, 0, sizeof(ext_event_data_t));
    memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
    memset(&supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));
    memset(&referee_warning_t, 0, sizeof(ext_referee_warning_t));
    memset(&robot_state, 0, sizeof(ext_game_robot_state_t));
    memset(&power_heat_data_t, 0, sizeof(ext_power_heat_data_t));
    memset(&game_robot_pos_t, 0, sizeof(ext_game_robot_pos_t));
    memset(&buff_musk_t, 0, sizeof(ext_buff_musk_t));
    memset(&robot_energy_t, 0, sizeof(aerial_robot_energy_t));
    memset(&robot_hurt_t, 0, sizeof(ext_robot_hurt_t));
    memset(&shoot_data_t, 0, sizeof(ext_shoot_data_t));
    memset(&bullet_remaining_t, 0, sizeof(ext_bullet_remaining_t));
    memset(&student_interactive_data_t, 0, sizeof(ext_student_interactive_data_t));
}

void Referee::referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
    case GAME_STATE_CMD_ID:
    {
        memcpy(&game_state, frame + index, sizeof(ext_game_state_t));
    }
    break;
    case GAME_RESULT_CMD_ID:
    {
        memcpy(&game_result, frame + index, sizeof(game_result));
    }
    break;
    case GAME_ROBOT_HP_CMD_ID:
    {
        memcpy(&game_robot_HP_t, frame + index, sizeof(ext_game_robot_HP_t));
    }
    break;

    case FIELD_EVENTS_CMD_ID:
    {
        memcpy(&field_event, frame + index, sizeof(field_event));
    }
    break;
    case SUPPLY_PROJECTILE_ACTION_CMD_ID:
    {
        memcpy(&supply_projectile_action_t, frame + index, sizeof(supply_projectile_action_t));
    }
    break;
    case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
    {
        memcpy(&supply_projectile_booking_t, frame + index, sizeof(supply_projectile_booking_t));
    }
    break;
    case REFEREE_WARNING_CMD_ID:
    {
        memcpy(&referee_warning_t, frame + index, sizeof(ext_referee_warning_t));
    }
    break;

    case ROBOT_STATE_CMD_ID:
    {
        memcpy(&robot_state, frame + index, sizeof(robot_state));
    }
    break;
    case POWER_HEAT_DATA_CMD_ID:
    {
        memcpy(&power_heat_data_t, frame + index, sizeof(power_heat_data_t));
    }
    break;
    case ROBOT_POS_CMD_ID:
    {
        memcpy(&game_robot_pos_t, frame + index, sizeof(game_robot_pos_t));
    }
    break;
    case BUFF_MUSK_CMD_ID:
    {
        memcpy(&buff_musk_t, frame + index, sizeof(buff_musk_t));
    }
    break;
    case AERIAL_ROBOT_ENERGY_CMD_ID:
    {
        memcpy(&robot_energy_t, frame + index, sizeof(robot_energy_t));
    }
    break;
    case ROBOT_HURT_CMD_ID:
    {
        memcpy(&robot_hurt_t, frame + index, sizeof(robot_hurt_t));
    }
    break;
    case SHOOT_DATA_CMD_ID:
    {
        memcpy(&shoot_data_t, frame + index, sizeof(shoot_data_t));
    }
    break;
    case BULLET_REMAINING_CMD_ID:
    {
        memcpy(&bullet_remaining_t, frame + index, sizeof(ext_bullet_remaining_t));
    }
    break;
    case STUDENT_INTERACTIVE_DATA_CMD_ID:
    {
        memcpy(&student_interactive_data_t, frame + index, sizeof(student_interactive_data_t));
    }
    break;
    default:
    {
        break;
    }
    }
}

//返回机器人ID
void Referee::get_robot_id(uint8_t *color)
{
    *color = robot_state.robot_id;
}

//************************功率控制***********************************
//底盘输出功率,底盘功率缓存
void Referee::get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
    *power = power_heat_data_t.chassis_power;
    *buffer = power_heat_data_t.chassis_power_buffer;
}

//底盘输出功率上限
void Referee::get_chassis_power_limit(fp32 *power_limit)
{
    *power_limit = robot_state.chassis_power_limit;
}

//17mm枪口热量上限, 17mm枪口实时热量 默认ID1
void Referee::get_shooter_id1_17mm_cooling_limit_and_heat(uint16_t *id1_17mm_cooling_limit, uint16_t *id1_17mm_cooling_heat)
{
    *id1_17mm_cooling_limit = robot_state.shooter_id1_17mm_cooling_limit;
    *id1_17mm_cooling_heat = power_heat_data_t.shooter_id1_17mm_cooling_heat;
}

//17mm枪口枪口射速上限,17mm实时射速 默认ID1
void Referee::get_shooter_id1_17mm_speed_limit_and_bullet_speed(uint16_t *id1_17mm_speed_limit, fp32 *bullet_speed)
{
    *id1_17mm_speed_limit = robot_state.shooter_id1_17mm_speed_limit;
    *bullet_speed = shoot_data_t.bullet_speed;
}

//17mm枪口热量冷却 默认ID1
void Referee::get_shooter_id1_17mm_cooling_rate(uint16_t *id1_17mm_cooling_rate)
{
    *id1_17mm_cooling_rate = robot_state.shooter_id1_17mm_cooling_rate;
}

//42mm枪口热量上限, 42mm枪口实时热量
void Referee::get_shooter_id1_42mm_cooling_limit_and_heat(uint16_t *id1_42mm_cooling_limit, uint16_t *id1_42mm_cooling_heat)
{
    *id1_42mm_cooling_limit = robot_state.shooter_id1_42mm_cooling_limit;
    *id1_42mm_cooling_heat = power_heat_data_t.shooter_id1_42mm_cooling_heat;
}

//42mm枪口枪口射速上限,42mm实时射速
void Referee::get_shooter_id1_42mm_speed_limit_and_bullet_speed(uint16_t *id1_42mm_speed_limit, uint16_t *bullet_speed)
{
    *id1_42mm_speed_limit = robot_state.shooter_id1_42mm_speed_limit;
    *bullet_speed = shoot_data_t.bullet_speed;
}

//42mm枪口热量冷却
void Referee::get_shooter_id1_42mm_cooling_rate(uint16_t *id1_42mm_cooling_rate)
{
    *id1_42mm_cooling_rate = robot_state.shooter_id1_42mm_cooling_rate;
}

//当前血量
void Referee::get_remain_hp(uint16_t *remain_HP)
{
    *remain_HP = robot_state.remain_HP;
}

//是否被击打
bool_t Referee::if_hit()
{
    static uint16_t hp_detect_time = 0; //血量检测间隔
    static uint16_t last_hp = 0;

    //初始化血量记录
    if (last_hp == 0)
        last_hp = robot_state.remain_HP;

    if (hp_detect_time++ > 300)
    {
        last_hp = robot_state.remain_HP;
        hp_detect_time = 0;
    }

    //受到高于10点的伤害,开始扭腰
    if (last_hp - robot_state.remain_HP >= 10)
        return TRUE;
    else
        return FALSE;
}

/**
  * @brief  判断自己红蓝方
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
void Referee::get_color(uint8_t *color)
{
    Judge_Self_ID = robot_state.robot_id; //读取当前机器人ID

    if (robot_state.robot_id > 10)
    {
        *color = BLUE;
    }
    else
    {
        *color = RED;
    }
}
/**
  * @brief  判断自身ID，选择客户端ID
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
void Referee::determine_ID(void)
{
    get_color(&Color);
    if (Color == BLUE)
    {
        Judge_SelfClient_ID = 0x0110 + (Judge_Self_ID - 0x10); //计算客户端ID
    }
    else if (Color == RED)
    {
        Judge_SelfClient_ID = 0x0100 + Judge_Self_ID; //计算客户端ID
    }
}
