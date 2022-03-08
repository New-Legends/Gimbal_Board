#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"


frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

ext_game_state_t game_state;
ext_game_result_t game_result;
ext_game_robot_HP_t game_robot_HP_t;

ext_event_data_t field_event;
ext_supply_projectile_action_t supply_projectile_action_t;
ext_supply_projectile_booking_t supply_projectile_booking_t;
ext_referee_warning_t referee_warning_t;


ext_game_robot_state_t robot_state;
ext_power_heat_data_t power_heat_data_t;
ext_game_robot_pos_t game_robot_pos_t;
ext_buff_musk_t buff_musk_t;
aerial_robot_energy_t robot_energy_t;
ext_robot_hurt_t robot_hurt_t;
ext_shoot_data_t shoot_data_t;
ext_bullet_remaining_t bullet_remaining_t;
ext_student_interactive_data_t student_interactive_data_t;

uint8_t Judge_Self_ID;//当前机器人的ID
uint16_t Judge_SelfClient_ID;//发送者机器人对应的客户端ID

int field_event_outpost;



void init_referee_struct_data(void)
{
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

void referee_data_solve(uint8_t *frame)
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

//判断前哨站状态
void output_state(void){
    if(game_robot_HP_t.red_outpost_HP>0){
        field_event_outpost=1;
    }
    else 
    {
        field_event_outpost=0;
    }
}



//返回机器人ID
uint8_t get_robot_id(void)
{
    return robot_state.robot_id;
}


//************************功率控制***********************************
//底盘输出功率,底盘功率缓存
void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
    *power = power_heat_data_t.chassis_power;
    *buffer = power_heat_data_t.chassis_power_buffer;
}


//17mm枪口热量上限, 17mm枪口实时热量 默认ID1
void get_shooter_id1_17mm_cooling_limit_and_heat(uint16_t *id1_17mm_cooling_limit, uint16_t *id1_17mm_cooling_heat)
{
    *id1_17mm_cooling_limit = robot_state.shooter_id1_17mm_cooling_limit;
    *id1_17mm_cooling_heat = power_heat_data_t.shooter_id1_17mm_cooling_heat;
}


//17mm枪口枪口射速上限,17mm实时射速 默认ID1
void get_shooter_id1_17mm_speed_limit_and_bullet_speed(uint16_t *id1_17mm_speed_limit, fp32 *bullet_speed)
{
    *id1_17mm_speed_limit = robot_state.shooter_id1_17mm_speed_limit;
    *bullet_speed = shoot_data_t.bullet_speed;
}

//17mm枪口热量冷却 默认ID1
void get_shooter_id1_17mm_cooling_rate(uint16_t *id1_17mm_cooling_rate)
{   
    *id1_17mm_cooling_rate = robot_state.shooter_id1_17mm_cooling_rate;
}

//42mm枪口热量上限, 42mm枪口实时热量
void get_shooter_id1_42mm_cooling_limit_and_heat(uint16_t *id1_42mm_cooling_limit, uint16_t *id1_42mm_cooling_heat)
{
    *id1_42mm_cooling_limit = robot_state.shooter_id1_42mm_cooling_limit;
    *id1_42mm_cooling_heat = power_heat_data_t.shooter_id1_42mm_cooling_heat;
}

//42mm枪口枪口射速上限,42mm实时射速
void get_shooter_id1_42mm_speed_limit_and_bullet_speed(uint16_t *id1_42mm_speed_limit, uint16_t *bullet_speed)
{
    *id1_42mm_speed_limit = robot_state.shooter_id1_42mm_speed_limit;
    *bullet_speed = shoot_data_t.bullet_speed;
}


//42mm枪口热量冷却
void get_shooter_id1_42mm_cooling_rate(uint16_t *id1_42mm_cooling_rate)
{   
    *id1_42mm_cooling_rate = robot_state.shooter_id1_42mm_cooling_rate;
}



//当前血量
uint16_t get_remain_hp()
{
    return robot_state.remain_HP;

}

//是否被击打
bool_t if_hit()
{
    static uint16_t hp_detect_time = 0;    //血量检测间隔
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
    if(last_hp - robot_state.remain_HP >= 10)
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
 


/**
  * @brief  判断自己红蓝方
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */

bool_t is_red_or_blue(void)
{
	Judge_Self_ID = robot_state.robot_id;//读取当前机器人ID
	
	if(robot_state.robot_id > 10)
	{
		return BLUE;
	}
	else 
	{
		return RED;
	}
}
/**
  * @brief  判断自身ID，选择客户端ID
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
void determine_ID(void)
{
    bool_t Color;
	Color = is_red_or_blue();
	if(Color == BLUE)
	{
		Judge_SelfClient_ID = 0x0110 + (Judge_Self_ID-0x10);//计算客户端ID
	}
	else if(Color == RED)
	{
		Judge_SelfClient_ID = 0x0100 + Judge_Self_ID;//计算客户端ID
	}
}



