#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

#include "protocol.h"

#define RED   0
#define BLUE  1

typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
    BLUE_HERO       = 11,
    BLUE_ENGINEER   = 12,
    BLUE_STANDARD_1 = 13,
    BLUE_STANDARD_2 = 14,
    BLUE_STANDARD_3 = 15,
    BLUE_AERIAL     = 16,
    BLUE_SENTRY     = 17,
} robot_id_t;
typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress_t;
typedef __packed struct //0001
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
} ext_game_state_t;

typedef __packed struct //0002
{
    uint8_t winner;
} ext_game_result_t;
typedef __packed struct
{
    uint16_t red_1_robot_HP;	// 红1英雄机器人血量(未上场及罚下血量为0)
	uint16_t red_2_robot_HP;	// 红2工程机器人血量
	uint16_t red_3_robot_HP;	// 红3步兵机器人血量
	uint16_t red_4_robot_HP;	// 红4步兵机器人血量
	uint16_t red_5_robot_HP;	// 红5步兵机器人血量
	uint16_t red_7_robot_HP;	// 红7哨兵机器人血量
	uint16_t red_outpost_HP;	// 红方前哨站血量
	uint16_t red_base_HP;		// 红方基地血量
	uint16_t blue_1_robot_HP;	// 蓝1英雄机器人血量
	uint16_t blue_2_robot_HP;	// 蓝2工程机器人血量
	uint16_t blue_3_robot_HP;	// 蓝3步兵机器人血量
	uint16_t blue_4_robot_HP;	// 蓝4步兵机器人血量
	uint16_t blue_5_robot_HP;	// 蓝5步兵机器人血量
	uint16_t blue_7_robot_HP;	// 蓝7哨兵机器人血量
	uint16_t blue_outpost_HP;	// 蓝方前哨站血量
	uint16_t blue_base_HP;		// 蓝方基地血量	
} ext_game_robot_HP_t;
typedef __packed struct//0x0101
{
    uint32_t others_1 : 10;
    uint32_t outpost  : 1;
    uint32_t others_2 : 21;
//	uint32_t event_type;
} ext_event_data_t;

typedef __packed struct //0x0102
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;


typedef __packed struct //0x0103
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_num;
} ext_supply_projectile_booking_t;

typedef __packed struct
{
    uint8_t level;
    uint8_t foul_robot_id;
} ext_referee_warning_t;
typedef __packed struct //0x0201
{
    uint8_t robot_id; 
    uint8_t robot_level; 
    uint16_t remain_HP; 
    uint16_t max_HP; 

    uint16_t shooter_id1_17mm_cooling_rate;
    uint16_t shooter_id1_17mm_cooling_limit;
    uint16_t shooter_id1_17mm_speed_limit;

    uint16_t shooter_id2_17mm_cooling_rate; 
    uint16_t shooter_id2_17mm_cooling_limit;
    uint16_t shooter_id2_17mm_speed_limit;

    uint16_t shooter_id1_42mm_cooling_rate; 
    uint16_t shooter_id1_42mm_cooling_limit;
    uint16_t shooter_id1_42mm_speed_limit;

    uint16_t chassis_power_limit; 
    uint8_t mains_power_gimbal_output : 1; 
    uint8_t mains_power_chassis_output : 1; 
    uint8_t mains_power_shooter_output : 1;

} ext_game_robot_state_t;

typedef __packed struct //0x0202
{
    uint16_t chassis_volt;        //底盘输出电压
    uint16_t chassis_current;     //底盘输出电流
    fp32 chassis_power;          //底盘输出功率
    uint16_t chassis_power_buffer; //底盘功率缓冲
    uint16_t shooter_id1_17mm_cooling_heat;  //1号17mm 枪口热量
    uint16_t shooter_id2_17mm_cooling_heat;  //2号17mm枪口热量
    uint16_t shooter_id1_42mm_cooling_heat;  //42mm 枪口热量
} ext_power_heat_data_t;

typedef __packed struct //0x0203
{
    fp32 x;
    fp32 y;
    fp32 z;
    fp32 yaw;
} ext_game_robot_pos_t;

typedef __packed struct //0x0204
{
    uint8_t power_rune_buff;
} ext_buff_musk_t;

typedef __packed struct //0x0205
{
    uint8_t energy_point;
    uint8_t attack_time;
} aerial_robot_energy_t;

typedef __packed struct //0x0206
{
    uint8_t armor_type : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

typedef __packed struct //0x0207
{
    uint8_t bullet_type;  //子弹类型: 1：17mm弹丸 2：42mm弹丸
    uint8_t shooter_id;  //发射机构ID： 1：1号17mm发射机构 2：2号17mm发射机构 3：42mm 发射机构
    uint8_t bullet_freq; //子弹射频 单位 Hz
    fp32 bullet_speed;  //子弹射速 单位 m/s
} ext_shoot_data_t;

typedef __packed struct
{
    uint16_t bullet_remaining_num_17mm; //17mm子弹剩余发射数目 
    uint16_t bullet_remaining_num_42mm; // 42mm子弹剩余发射数目
    uint16_t coin_remaining_num;        //剩余金币数量
} ext_bullet_remaining_t;
typedef __packed struct //0x0301
{
    uint16_t send_ID;
    uint16_t receiver_ID;
    uint16_t data_cmd_id;
    uint16_t data_len;
    uint8_t *data;
} ext_student_interactive_data_t;

typedef __packed struct
{
    fp32 data1;
    fp32 data2;
    fp32 data3;
    uint8_t data4;
} custom_data_t;


typedef __packed struct
{
    uint8_t data[64];
} ext_up_stream_data_t;

typedef __packed struct
{
    uint8_t data[32];
} ext_download_stream_data_t;


extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);

extern void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer);

extern uint8_t get_robot_id(void);
extern void output_state(void);
extern int field_event_outpost;


//17mm枪口热量上限, 17mm枪口实时热量 默认ID1
extern void get_shooter_id1_17mm_cooling_limit_and_heat(uint16_t *id1_17mm_cooling_limit, uint16_t *id1_17mm_cooling_heat);
//17mm枪口枪口射速上限,17mm实时射速 默认ID1
extern void get_shooter_id1_17mm_speed_limit_and_bullet_speed(uint16_t *id1_17mm_speed_limit, fp32 *bullet_speed);
//17mm枪口热量冷却 默认ID1
extern void get_shooter_id1_17mm_cooling_rate(uint16_t *id1_17mm_cooling_rate);


//42mm枪口热量上限, 42mm枪口实时热量
extern void get_shooter_id1_42mm_cooling_limit_and_heat(uint16_t *id1_42mm_cooling_limit, uint16_t *id1_42mm_cooling_heat);
//42mm枪口枪口射速上限,42mm实时射速
extern void get_shooter_id1_42mm_speed_limit_and_bullet_speed(uint16_t *id1_42mm_speed_limit, uint16_t *bullet_speed);
//42mm枪口热量冷却
extern void get_shooter_id1_42mm_cooling_rate(uint16_t *id1_42mm_cooling_rate);;


extern void determine_ID(void);
extern bool_t is_red_or_blue(void);

extern bool_t if_hit(void);
#endif
