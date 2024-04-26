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
ext_referee_warning_t referee_warning_t;
ext_game_robot_status_t robot_state;
ext_power_heat_data_t power_heat_data_t;
ext_robot_pos_t robot_pos_t;
ext_buff_t buff_t;
ext_air_support_data_t air_support_data_t;
ext_hurt_data_t robot_hurt_t;
ext_shoot_data_t shoot_data_t;
ext_projectile_allowance_t projectile_allowance_t;
ext_rfid_status_t rfid_status_t;
ext_dart_client_cmd_t dart_client_cmd_t;
ext_ground_robot_position_t ground_robot_position_t;
ext_radar_mark_data_t radar_mark_data_t;
ext_sentry_info_t sentry_info_t;
ext_radar_info_t radar_info_t;
ext_robot_interaction_data_t robot_interaction_data_t;
ext_custom_robot_data_t custom_robot_data_t;
ext_map_command_t map_command_t;
ext_robot_key_command_t robot_key_command_t;
ext_map_robot_data_t map_robot_data_t;
ext_custom_client_data_t custom_client_data_t;
ext_map_data_t map_data_t;
ext_custom_info_t custom_info_t;

void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));
    memset(&game_state, 0, sizeof(ext_game_state_t));
    memset(&game_result, 0, sizeof(ext_game_result_t));
    memset(&game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));
    memset(&field_event, 0, sizeof(ext_event_data_t));
    memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
    memset(&referee_warning_t, 0, sizeof(ext_referee_warning_t));
    memset(&robot_state, 0, sizeof(ext_game_robot_status_t));
    memset(&power_heat_data_t, 0, sizeof(ext_power_heat_data_t));
    memset(&robot_pos_t, 0, sizeof(ext_robot_pos_t));
    memset(&buff_t, 0, sizeof(ext_buff_t));
    memset(&air_support_data_t, 0, sizeof(ext_air_support_data_t));
    memset(&robot_hurt_t, 0, sizeof(ext_hurt_data_t));
    memset(&shoot_data_t, 0, sizeof(ext_shoot_data_t));
    memset(&projectile_allowance_t, 0, sizeof(ext_projectile_allowance_t));
    memset(&rfid_status_t, 0, sizeof(ext_rfid_status_t));
    memset(&dart_client_cmd_t, 0, sizeof(ext_dart_client_cmd_t));
    memset(&ground_robot_position_t, 0, sizeof(ext_ground_robot_position_t));
    memset(&radar_mark_data_t, 0, sizeof(ext_radar_mark_data_t));
    memset(&sentry_info_t, 0, sizeof(ext_sentry_info_t));
    memset(&radar_info_t, 0, sizeof(ext_radar_info_t));
    memset(&robot_interaction_data_t, 0, sizeof(ext_robot_interaction_data_t));
    memset(&custom_robot_data_t, 0, sizeof(ext_custom_robot_data_t));
    memset(&map_command_t, 0, sizeof(ext_map_command_t));
    memset(&robot_key_command_t, 0, sizeof(ext_robot_key_command_t));
    memset(&map_robot_data_t, 0, sizeof(ext_map_robot_data_t));
    memset(&custom_client_data_t, 0, sizeof(ext_custom_client_data_t));
    memset(&map_data_t, 0, sizeof(ext_map_data_t));
    memset(&custom_info_t, 0, sizeof(ext_custom_info_t));
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
        memcpy(&game_result, frame + index, sizeof(ext_game_result_t));
    }
    break;
    case GAME_ROBOT_HP_CMD_ID:
    {
        memcpy(&game_robot_HP_t, frame + index, sizeof(ext_game_robot_HP_t));
    }
    break;

    case FIELD_EVENTS_CMD_ID:
    {
        memcpy(&field_event, frame + index, sizeof(ext_event_data_t));
    }
    break;
    case SUPPLY_PROJECTILE_ACTION_CMD_ID:
    {
        memcpy(&supply_projectile_action_t, frame + index, sizeof(ext_supply_projectile_action_t));
    }
    break;
    case REFEREE_WARNING_CMD_ID:
    {
        memcpy(&referee_warning_t, frame + index, sizeof(ext_referee_warning_t));
    }
    break;

    case ROBOT_STATE_CMD_ID:
    {
        memcpy(&robot_state, frame + index, sizeof(ext_game_robot_status_t));
    }
    break;
    case POWER_HEAT_DATA_CMD_ID:
    {
        memcpy(&power_heat_data_t, frame + index, sizeof(ext_power_heat_data_t));
    }
    break;
    case ROBOT_POS_CMD_ID:
    {
        memcpy(&robot_pos_t, frame + index, sizeof(ext_robot_pos_t));
    }
    break;
    case BUFF_MUSK_CMD_ID:
    {
        memcpy(&buff_t, frame + index, sizeof(ext_buff_t));
    }
    break;
    case AERIAL_ROBOT_ENERGY_CMD_ID:
    {
        memcpy(&air_support_data_t, frame + index, sizeof(ext_air_support_data_t));
    }
    break;
    case ROBOT_HURT_CMD_ID:
    {
        memcpy(&robot_hurt_t, frame + index, sizeof(ext_hurt_data_t));
    }
    break;
    case SHOOT_DATA_CMD_ID:
    {
        memcpy(&shoot_data_t, frame + index, sizeof(ext_shoot_data_t));
    }
    break;
    case BULLET_REMAINING_CMD_ID:
    {
        memcpy(&projectile_allowance_t, frame + index, sizeof(ext_projectile_allowance_t));
    }
    break;
    case RFID_STATUS_CMD_ID:
    {
        memcpy(&rfid_status_t, frame + index, sizeof(ext_rfid_status_t));
    }
    break;
    case DART_CLIENT_CMD_ID:
    {
        memcpy(&dart_client_cmd_t, frame + index, sizeof(ext_dart_client_cmd_t));
    }
    break;
    case GROUND_ROBOT_POSITION_CMD_ID:
    {
        memcpy(&ground_robot_position_t, frame + index, sizeof(ext_ground_robot_position_t));
    }
    break;
    case RADAR_MARK_DATA_CMD_ID:
    {
        memcpy(&radar_mark_data_t, frame + index, sizeof(ext_radar_mark_data_t));
    }
    break;
    case SENTRY_INFO_CMD_ID:
    {
        memcpy(&sentry_info_t, frame + index, sizeof(ext_sentry_info_t));
    }
    break;
    case RADAR_INFO_CMD_ID:
    {
        memcpy(&radar_info_t, frame + index, sizeof(ext_radar_info_t));
    }
    break;
    case ROBOT_INTERACTION_DATA_CMD_ID:
    {
        memcpy(&robot_interaction_data_t, frame + index, sizeof(ext_robot_interaction_data_t));
    }
    break;
    case CUSTOM_ROBOT_DATA_CMD_ID:
    {
        memcpy(&custom_robot_data_t, frame + index, sizeof(ext_custom_robot_data_t));
    }
    break;
    case MAP_COMMAND_CMD_ID:
    {
        memcpy(&map_command_t, frame + index, sizeof(ext_map_command_t));
    }
    break;
    case ROBOT_KEY_COMMAND_CMD_ID:
    {
        memcpy(&robot_key_command_t, frame + index, sizeof(ext_robot_key_command_t));
    }
    break;
    case MAP_ROBOT_DATA_CMD_ID:
    {
        memcpy(&map_robot_data_t, frame + index, sizeof(ext_map_robot_data_t));
    }
    break;
    case CUSTOM_CLIENT_DATA_CMD_ID:
    {
        memcpy(&custom_client_data_t, frame + index, sizeof(ext_game_state_t));
    }
    break;
    case MAP_DATA_CMD_ID:
    {
        memcpy(&map_data_t, frame + index, sizeof(ext_map_data_t));
    }
    break;
    case CUSTOM_INFO_CMD_ID:
    {
        memcpy(&custom_info_t, frame + index, sizeof(ext_custom_info_t));
    }
    break;
    default:
    {
        break;
    }
    }
}

void get_chassis_power_and_buffer(fp32 *power_real, fp32 *buffer)
{
    *power_real = power_heat_data_t.chassis_power;
    *buffer = power_heat_data_t.buffer_energy;
}

uint8_t get_robot_id(void)
{
    return robot_state.robot_id;
}

void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0)
{
    *heat0_limit = robot_state.shooter_barrel_heat_limit;
    *heat0 = power_heat_data_t.shooter_17mm_1_barrel_heat;
}

void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1)
{
    *heat1_limit = robot_state.shooter_barrel_heat_limit;
    *heat1 = power_heat_data_t.shooter_17mm_2_barrel_heat;
}
// uint16_t get_shooter_id1_17mm_speed_limit(void)
// {
//     return robot_state.shooter_id1_17mm_speed_limit;
// }

uint16_t get_chassis_power_limit(void)
{
    return robot_state.chassis_power_limit;
}
// 返回当前兵种的等级和底盘输出功率,用于超级电容的功率控制
void get_chassis_power_state(uint16_t *level, float *power, uint16_t *power_buffer)
{
    *level = robot_state.robot_level;
    *power = power_heat_data_t.chassis_power;
    *power_buffer = power_heat_data_t.buffer_energy;
}
void get_chassis_power_limit_state(uint16_t *chassis_power_limit)
{
    *chassis_power_limit = robot_state.chassis_power_limit;
}