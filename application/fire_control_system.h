#ifndef __FIRE_CONTROL_SYSTEM_H__
#define __FIRE_CONTROL_SYSTEM_H__

#include "struct_typedef.h"
#include "pid.h"
#include "CAN_receive.h"
#include "user_lib.h"

#define FIRE_CONTROL_SYSTEM_INIT_TIME 3000
#define FIRE_CONTROL_SYSTEM_WAIT_TIME 10

#define RED  2 //红方
#define BLUE 3 //蓝方

#define START_GAME     0x05 //开始比赛
#define NOT_START_GAME 0x06 //未开始比赛

#define SENTINEL_OFFLINE_WAIT_TIME 5 //停止比赛，等待5s

#define NEED_ENRICH_BLOOD 0x00 //需要补血
#define NOT_ENRICH_BLOOD 0x01 //不需要补血

#define COOLING_SHOOT_FREQ 5.0f

#define GENERAL_LIMIT    0.8f //常规模式冷却热量限制
#define RAGE_LIMIT       0.8f //狂暴模式冷却热量限制

#define SENTINEL_RAGE_HP_PERCENT        0.3f  //低于30%血量开启狂暴
#define SENTINEL_ENEMY_BASE_UP_PERCENT  0.5f  //敌人基地血量低于50%开启狂暴（补刀）
#define SENTINEL_BASE_UP_PERCENT        0.5f  //己方基地低于50%开启狂暴（换家）

#define GUN_SPEED_NUM 0.03f

#define COOL_SYSTEM_WAIT_TIME 30

//GUN ANGLE PID
#define GUN_ANGLE_PID_KP        4.0f
#define GUN_ANGLE_PID_KI        0.0f
#define GUN_ANGLE_PID_KD        0.0f
#define GUN_ANGLE_PID_MAX_OUT   20.0f
#define GUN_ANGLE_PID_MAX_IOUT  10.0f

//GUN SPEED PID
#define GUN_SPEED_PID_KP        800.0f
#define GUN_SPEED_PID_KI        0.0f
#define GUN_SPEED_PID_KD        5.0f
#define GUN_SPEED_PID_MAX_OUT   20000.0f
#define GUN_SPEED_PID_MAX_IOUT  2800.0f 

//枪管角度误差允许范围
#define GUN_NORMAL_SPEED 5.0f  //换枪管时的速度
#define GUN_INIT_SPEED   1.0f
#define GUN_BLOCK_TIME   500
#define GUN_BLOCK_SPEED  0.5f
#define GUN_CONTROL_TIME 1

typedef struct
{
	pid_type_def gun_angle_pid;
	pid_type_def gun_speed_pid;
	first_order_filter_type_t gun_speed_slow_set;
	const motor_measure_t *gun_motor_measure;
	uint8_t gun_lock_flag;
	uint8_t gun_init_flag;
	uint8_t gun_done;
	int16_t ecd_count;
	int16_t ecd_offset;
	int16_t gun_block_time;
	fp32 gun_angle;
	fp32 gun_last_angle;
	fp32 gun_speed;
	fp32 gun_angle_set;
	fp32 gun_speed_set;
	uint8_t gun_behaviour;
	uint8_t last_gun_change_flag;
	int16_t give_current;
}gun_control_t;

//typedef __packed struct
//{
//	uint8_t side;   //红方or蓝方
//	uint8_t SENTINEL_ID; //哨兵ID号
//	uint16_t SENTINEL_MAX_UP; //哨兵生命值上限
//	uint16_t SENTINEL_UP; //哨兵当前生命值
//	uint16_t SENTINEL_LAST_UP;//哨兵上一状态生命值
//	uint16_t BASE_UP;     //基地生命值
//	uint16_t ENEMY_BASE_UP; //敌方基地生命值
//	uint16_t shooter_id1_17mm_cooling_rate; //1号17mm枪口每秒冷却值
//	uint16_t shooter_id1_17mm_cooling_limit;//1号17mm枪口热量上限
//	uint16_t shooter_id1_17mm_speed_limit;  //1号17mm枪口上限速度
//	uint16_t shooter_id1_17mm_cooling_heat; //1号17mm枪口当前热量
//	uint16_t shooter_id1_17mm_cooling_last_heat; //1号17mm枪口上一状态热量
//	uint8_t bullet_freq;                    //子弹射频
////	uint16_t bullet_remaining_num_17mm;     //剩余子弹发射数
//	float SENTINEL_UP_PERCENT; //哨兵剩余血量(百分比)
//	float BASE_UP_PERCENT;  //基地剩余血量(百分比)
//	float ENEMY_BASE_UP_PERCENT; //敌方基地剩余血量(百分比)
//	float shooter_id1_17mm_cooling_heat_PERCENT; //热量百分比
////	float BULLET_REMAINING_NUM_17MMPERCENT; //剩余子弹发射数(百分比)
//	float x;//哨兵目前位置x，单位m->由裁判系统提供
//	float y;//哨兵目前位置y，单位m->由裁判系统提供
//}Typedef_SENTINEL;

typedef enum
{
  general_setting = 0x10, //常规模式
	rage_setting    //狂暴模式  
}COOLING_RANGE_e;

extern COOLING_RANGE_e cooling_range;

extern uint16_t sentinel_SENTINEL_UP;

extern unsigned char lose_up_flag; //损血标志位

extern float shoot_freq; //射频->（这个射频与CAN发送的值有关）;

extern gun_control_t gun_cmd;

//extern Typedef_SENTINEL sentinel;

extern uint8_t sentinel_game_state;

extern uint8_t sentinel_side; 

extern float sentinel_path_x; //云台手指令x
extern float sentinel_path_y; //云台手指令y
extern unsigned char sentinel_path_state_flag; //云台手命令状态，为1代表给过指令

extern uint8_t detect_enrich_flag(void);

extern void fire_control_system(void const * argument);

extern void gun_change_init(gun_control_t * gun_control_init);

extern void gun_control(gun_control_t * gun_control_cmd);

#endif
