#ifndef __FIRE_CONTROL_SYSTEM_H__
#define __FIRE_CONTROL_SYSTEM_H__

#include "struct_typedef.h"
#include "pid.h"
#include "CAN_receive.h"
#include "user_lib.h"

#define FIRE_CONTROL_SYSTEM_INIT_TIME 3000
#define FIRE_CONTROL_SYSTEM_WAIT_TIME 10

#define RED  2 //�췽
#define BLUE 3 //����

#define START_GAME     0x05 //��ʼ����
#define NOT_START_GAME 0x06 //δ��ʼ����

#define SENTINEL_OFFLINE_WAIT_TIME 5 //ֹͣ�������ȴ�5s

#define NEED_ENRICH_BLOOD 0x00 //��Ҫ��Ѫ
#define NOT_ENRICH_BLOOD 0x01 //����Ҫ��Ѫ

#define COOLING_SHOOT_FREQ 5.0f

#define GENERAL_LIMIT    0.8f //����ģʽ��ȴ��������
#define RAGE_LIMIT       0.8f //��ģʽ��ȴ��������

#define SENTINEL_RAGE_HP_PERCENT        0.3f  //����30%Ѫ��������
#define SENTINEL_ENEMY_BASE_UP_PERCENT  0.5f  //���˻���Ѫ������50%�����񱩣�������
#define SENTINEL_BASE_UP_PERCENT        0.5f  //�������ص���50%�����񱩣����ң�

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

//ǹ�ܽǶ��������Χ
#define GUN_NORMAL_SPEED 5.0f  //��ǹ��ʱ���ٶ�
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
//	uint8_t side;   //�췽or����
//	uint8_t SENTINEL_ID; //�ڱ�ID��
//	uint16_t SENTINEL_MAX_UP; //�ڱ�����ֵ����
//	uint16_t SENTINEL_UP; //�ڱ���ǰ����ֵ
//	uint16_t SENTINEL_LAST_UP;//�ڱ���һ״̬����ֵ
//	uint16_t BASE_UP;     //��������ֵ
//	uint16_t ENEMY_BASE_UP; //�з���������ֵ
//	uint16_t shooter_id1_17mm_cooling_rate; //1��17mmǹ��ÿ����ȴֵ
//	uint16_t shooter_id1_17mm_cooling_limit;//1��17mmǹ����������
//	uint16_t shooter_id1_17mm_speed_limit;  //1��17mmǹ�������ٶ�
//	uint16_t shooter_id1_17mm_cooling_heat; //1��17mmǹ�ڵ�ǰ����
//	uint16_t shooter_id1_17mm_cooling_last_heat; //1��17mmǹ����һ״̬����
//	uint8_t bullet_freq;                    //�ӵ���Ƶ
////	uint16_t bullet_remaining_num_17mm;     //ʣ���ӵ�������
//	float SENTINEL_UP_PERCENT; //�ڱ�ʣ��Ѫ��(�ٷֱ�)
//	float BASE_UP_PERCENT;  //����ʣ��Ѫ��(�ٷֱ�)
//	float ENEMY_BASE_UP_PERCENT; //�з�����ʣ��Ѫ��(�ٷֱ�)
//	float shooter_id1_17mm_cooling_heat_PERCENT; //�����ٷֱ�
////	float BULLET_REMAINING_NUM_17MMPERCENT; //ʣ���ӵ�������(�ٷֱ�)
//	float x;//�ڱ�Ŀǰλ��x����λm->�ɲ���ϵͳ�ṩ
//	float y;//�ڱ�Ŀǰλ��y����λm->�ɲ���ϵͳ�ṩ
//}Typedef_SENTINEL;

typedef enum
{
  general_setting = 0x10, //����ģʽ
	rage_setting    //��ģʽ  
}COOLING_RANGE_e;

extern COOLING_RANGE_e cooling_range;

extern uint16_t sentinel_SENTINEL_UP;

extern unsigned char lose_up_flag; //��Ѫ��־λ

extern float shoot_freq; //��Ƶ->�������Ƶ��CAN���͵�ֵ�йأ�;

extern gun_control_t gun_cmd;

//extern Typedef_SENTINEL sentinel;

extern uint8_t sentinel_game_state;

extern uint8_t sentinel_side; 

extern float sentinel_path_x; //��̨��ָ��x
extern float sentinel_path_y; //��̨��ָ��y
extern unsigned char sentinel_path_state_flag; //��̨������״̬��Ϊ1�������ָ��

extern uint8_t detect_enrich_flag(void);

extern void fire_control_system(void const * argument);

extern void gun_change_init(gun_control_t * gun_control_init);

extern void gun_control(gun_control_t * gun_control_cmd);

#endif
