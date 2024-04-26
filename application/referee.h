#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

#include "protocol.h"


typedef enum
{
    RED_HERO = 1,
    RED_ENGINEER = 2,
    RED_STANDARD_1 = 3,
    RED_STANDARD_2 = 4,
    RED_STANDARD_3 = 5,
    RED_AERIAL = 6,
    RED_SENTRY = 7,
    BLUE_HERO = 11,
    BLUE_ENGINEER = 12,
    BLUE_STANDARD_1 = 13,
    BLUE_STANDARD_2 = 14,
    BLUE_STANDARD_3 = 15,
    BLUE_AERIAL = 16,
    BLUE_SENTRY = 17,
} robot_id_t;
typedef enum
{
    PROGRESS_UNSTART = 0,
    PROGRESS_PREPARE = 1,
    PROGRESS_SELFCHECK = 2,
    PROGRESS_5sCOUNTDOWN = 3,
    PROGRESS_BATTLE = 4,
    PROGRESS_CALCULATING = 5,
} game_progress_t;
typedef __packed struct // 0001 比赛状态数据，固定以 1Hz 频率发送
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} ext_game_state_t;

typedef __packed struct // 0002 比赛结果数据，比赛结束触发发送
{
    uint8_t winner;
} ext_game_result_t;
typedef __packed struct // 0003 机器人血量数据，固定以 3Hz 频率发送
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;

typedef __packed struct // 0101 场地事件数据
{
    uint32_t event_type;
} ext_event_data_t;

typedef __packed struct // 0x0102  补给站动作标识数据，补给站丸释放时触发发送
{
    uint8_t reserved;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

typedef __packed struct // 0x0104 裁判警告数据，己方判罚/判负时触发发送
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
} ext_referee_warning_t;

typedef __packed struct // 0x0105 飞镖发射相关数据
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;
} dart_info_t;

typedef __packed struct // 0x0201 机器人性能体系数据
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} ext_game_robot_status_t;

typedef __packed struct // 0x0202 实时底盘功率和枪口热量数据
{
    uint16_t chassis_voltage;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_1_barrel_heat;
    uint16_t shooter_17mm_2_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
} ext_power_heat_data_t;

typedef __packed struct // 0x0203 机器人位置数据
{
    float x;
    float y;
    float angle;
} ext_robot_pos_t;

typedef __packed struct // 0x0204 机器人增益数据
{
    uint8_t recovery_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
} ext_buff_t;

typedef __packed struct // 0x0205 空中支援时间数据
{
    uint8_t airforce_status;
    uint8_t time_remain;
} ext_air_support_data_t;

typedef __packed struct // 0x0206 伤害状态数据，伤害发生后发送
{
    uint8_t armor_id : 4;
    uint8_t HP_deduction_reason : 4;
} ext_hurt_data_t;

typedef __packed struct // 0x0207 实时射击数据，弹丸发射后发送
{
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float initial_speed;
} ext_shoot_data_t;

typedef __packed struct // 0x0208 允许发弹量
{
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
} ext_projectile_allowance_t;

typedef __packed struct // 0x0209 机器人 RFID 模块状态
{
    uint32_t rfid_status;
} ext_rfid_status_t;

typedef __packed struct // 0x020A 飞镖选手端指令数据
{
    uint8_t dart_launch_opening_status;
    uint8_t reserved;
    uint16_t target_change_time;
    uint16_t latest_launch_cmd_time;
} ext_dart_client_cmd_t;

typedef __packed struct // 0x020B 地面机器人位置数据
{
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float standard_5_x;
    float standard_5_y;
} ext_ground_robot_position_t;

typedef __packed struct // 0x020C 雷达标记进度数据
{
    uint8_t mark_hero_progress;
    uint8_t mark_engineer_progress;
    uint8_t mark_standard_3_progress;
    uint8_t mark_standard_4_progress;
    uint8_t mark_standard_5_progress;
    uint8_t mark_sentry_progress;
} ext_radar_mark_data_t;

typedef __packed struct // 0x020D 哨兵自主决策信息同步
{
    uint32_t sentry_info;
} ext_sentry_info_t;

typedef __packed struct // 0x020E 雷达自主决策信息同步
{
    uint8_t radar_info;
} ext_radar_info_t;

typedef __packed struct // 0x0301
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[];
} ext_robot_interaction_data_t;

typedef __packed struct // 0x0302
{
		
    uint8_t data[30];
} ext_custom_robot_data_t;

typedef __packed struct // 0x0303
{
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint8_t cmd_source;
} ext_map_command_t;

typedef __packed struct // 0x0304
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t left_button_down;
    uint8_t right_button_down;
    __packed struct
    {
        uint16_t W : 1;
        uint16_t S : 1;
        uint16_t A : 1;
        uint16_t D : 1;
        uint16_t Q : 1;
        uint16_t E : 1;
        uint16_t R : 1;
        uint16_t F : 1;
        uint16_t G : 1;
        uint16_t Z : 1;
        uint16_t X : 1;
        uint16_t C : 1;
        uint16_t V : 1;
        uint16_t B : 1;
        uint16_t SHIFT : 1;
        uint16_t CTRL : 1;
    } keyboard_value;
    uint16_t reserved;
} ext_robot_key_command_t; // 图传键鼠结构体定义

typedef __packed struct // 0x0305
{
    uint16_t target_robot_id;
    float target_position_x;
    float target_position_y;
} ext_map_robot_data_t;

typedef __packed struct // 0x0306
{
    uint16_t key_value;
    uint16_t x_position : 12;
    uint16_t mouse_left : 4;
    uint16_t y_position : 12;
    uint16_t mouse_right : 4;
    uint16_t reserved;
} ext_custom_client_data_t;

typedef __packed struct // 0x0307
{
    uint8_t intention;
    uint16_t start_position_x;
    uint16_t start_position_y;
    int8_t delta_x[49];
    int8_t delta_y[49];
    uint16_t sender_id;
} ext_map_data_t;

typedef __packed struct // 0x0308
{
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];
} ext_custom_info_t;

extern ext_game_state_t game_state;
extern ext_game_result_t game_result;
extern ext_game_robot_HP_t game_robot_HP_t;
extern ext_event_data_t field_event;
extern ext_supply_projectile_action_t supply_projectile_action_t;
extern ext_referee_warning_t referee_warning_t;
extern ext_game_robot_status_t robot_state;
extern ext_power_heat_data_t power_heat_data_t;
extern ext_robot_pos_t robot_pos_t;
extern ext_buff_t buff_t;
extern ext_air_support_data_t air_support_data_t;
extern ext_hurt_data_t robot_hurt_t;
extern ext_shoot_data_t shoot_data_t;
extern ext_projectile_allowance_t projectile_allowance_t;
extern ext_rfid_status_t rfid_status_t;
extern ext_dart_client_cmd_t dart_client_cmd_t;
extern ext_ground_robot_position_t ground_robot_position_t;
extern ext_radar_mark_data_t radar_mark_data_t;
extern ext_sentry_info_t sentry_info_t;
extern ext_radar_info_t radar_info_t;
extern ext_robot_interaction_data_t robot_interaction_data_t;
extern ext_custom_robot_data_t custom_robot_data_t;
extern ext_map_command_t map_command_t;
extern ext_robot_key_command_t robot_key_command_t;
extern ext_map_robot_data_t map_robot_data_t;
extern ext_custom_client_data_t custom_client_data_t;
extern ext_map_data_t map_data_t;
extern ext_custom_info_t custom_info_t;

extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);

extern void get_chassis_power_and_buffer(fp32 *power_real, fp32 *buffer);

extern uint8_t get_robot_id(void);

extern void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0);
extern void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1);
extern uint16_t get_shooter_id1_17mm_speed_limit(void);
extern uint16_t get_chassis_power_limit(void);
extern void get_chassis_power_state(uint16_t *level, float *power, uint16_t *power_buffer);
extern void get_chassis_power_limit_state(uint16_t *chassis_power_limit);
#endif
