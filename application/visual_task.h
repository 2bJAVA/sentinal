#ifndef __VISUAL_TASK_H__
#define __VISUAL_TASK_H__

#include "freertos.h"
#include "task.h"
#include "struct_typedef.h"
#include "remote_control.h"
#include "stdio.h"
#include "stdbool.h"
#include "math.h"
#include "arm_math.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define VISUAL_RECEPTION_INIT_TIME 516
#define VISUAL_RECEPTION_WAIT_TIME 2

#define CV_SOF 0x5A
#define CV_EOF 0xFF
#define CV_SEND 0
#define CV_RECEIVE 1
#define PC_ABS(x) ((x > 0) ? (x) : (-x))

#define CV_PITCH_ERROR	0.025f
#define CV_YAW_ERROR 	0.04f
#define CV_YAW_CONPEN   0.01f
#define CV_PITCH_CONPEN 0.001f

#define LargeArmor_HalfWidth 	0.1175f
#define LittleArmor_HalfWidth 0.07f // 0.07f
#define THERSHOLD_V_YAW       0.8f
#define VISION_CONTROL_TIME 	120  //��Χ100~200 
	
#define MINIPC_SENDLENGTH 28U

#define MINIPC_REVCLENGTH 48U

#define CV_YAW_LARGE_ERROR 0.02f
#define CV_YAW_LITTLE_ERROR 0.02f

#define DAMP_RANGE_CLOCK_WISE_MIN 0.0f
#define DAMP_RANGE_CLOCK_WISE_MAX 0.0f
#define DAMP_RANGE_CLOCK_ANTI_WISE_MIN 0.0f
#define DAMP_RANGE_CLOCK_ANTI_WISE_MAX 0.0f

#pragma pack(1)
typedef struct
{
	uint8_t header;
	uint8_t tracking : 1;
	uint8_t id : 3;         // 0-outpost 6-guard 7-base
	uint8_t armors_num : 3; // 2-balance 3-outpost 4-normal
	uint8_t reserved : 1;
	float x;
	float y;
	float z;
	float yaw;
	float vx;
	float vy;
	float vz;
	float v_yaw;
	float r1;
	float r2;
	float dz;
	uint16_t checksum;
} CV_R_t;

typedef struct
{
	uint8_t header;
	uint8_t detect_color : 1; // 0-red 1-blue
	bool reset_tracker : 1;
	uint8_t reserved : 6;
	float roll;
	float pitch;
	float yaw;
	float aim_x;
	float aim_y;
	float aim_z;
	uint16_t checksum;
} CV_T_t;
#pragma pack()

typedef __packed struct
{
	uint8_t state; // ���״̬
	// /*
	// LOST				0x00
	// FIND_NO_SHOOT	0x01
	// FIND_SHOOT		0x02
	// FINDING			0x03
	// */
	float pitch; // pitch����Ƕ�
	float yaw;   // yaw����Ƕ�
	// 0��������1������
	uint8_t relive_flag;      // ��������ָ��
	uint8_t pill_flag;        // ����һ�����ָ��
	uint16_t pill_17mm_count; // ��Ҫ�һ���17mm�ӵ���
	uint8_t valid;
	float last_pitch;
	float last_yaw;
	float per_pitch;
	float per_yaw;
	uint8_t now_state;
	uint8_t last_state;
	uint8_t lost_flag;
	uint8_t check;

} ext_gimbal_CV_ctrl_t;

#ifndef PI
#define PI 3.1415926535f
#endif
#define GRAVITY 9.78
typedef unsigned char uint8_t;
enum ARMOR_ID
{
	ARMOR_OUTPOST = 0,
	ARMOR_HERO = 1,
	ARMOR_ENGINEER = 2,
	ARMOR_INFANTRY3 = 3,
	ARMOR_INFANTRY4 = 4,
	ARMOR_INFANTRY5 = 5,
	ARMOR_GUARD = 6,
	ARMOR_BASE = 7
};

enum ARMOR_NUM
{
	ARMOR_NUM_BALANCE = 2,
	ARMOR_NUM_OUTPOST = 3,
	ARMOR_NUM_NORMAL = 4
};

enum BULLET_TYPE
{
	BULLET_17 = 0,
	BULLET_42 = 1
};

// ���ò���
struct SolveTrajectoryParams
{
	float k; // ����ϵ��

	// �������
	enum BULLET_TYPE bullet_type; // ������������� 0-���� 1-Ӣ��
	float current_v;              // ��ǰ����
	float current_pitch;          // ��ǰpitch
	float current_yaw;            // ��ǰyaw

	// Ŀ�����
	float xw;                 // ROS����ϵ�µ�x
	float yw;                 // ROS����ϵ�µ�y
	float zw;                 // ROS����ϵ�µ�z
	float vxw;                // ROS����ϵ�µ�vx
	float vyw;                // ROS����ϵ�µ�vy
	float vzw;                // ROS����ϵ�µ�vz
	float tar_yaw;            // Ŀ��yaw
	float v_yaw;              // Ŀ��yaw�ٶ�
	float r1;                 // Ŀ�����ĵ�ǰ��װ�װ�ľ���
	float r2;                 // Ŀ�����ĵ�����װ�װ�ľ���
	float dz;                 // ��һ��װ�װ������ڱ����ٵ�װ�װ�ĸ߶Ȳ�
	int bias_time;            // ƫ��ʱ��
	float s_bias;             // ǹ��ǰ�Ƶľ���
	float z_bias;             // yaw������ǹ��ˮƽ�Ĵ�ֱ����
	enum ARMOR_ID armor_id;   // װ�װ�����  0-outpost 6-guard 7-base
														// 1-Ӣ�� 2-���� 3-4-5-����
	enum ARMOR_NUM armor_num; // װ�װ�����  2-balance 3-outpost 4-normal

	float aim_x;
	float aim_y;
	float aim_z;
};

// ���ڴ洢Ŀ��װ�װ����Ϣ
struct tar_pos
{
	float x;   // װ�װ�����������ϵ�µ�x
	float y;   // װ�װ�����������ϵ�µ�y
	float z;   // װ�װ�����������ϵ�µ�z
	float yaw; // װ�װ�����ϵ�������������ϵ��yaw��
};

extern ext_gimbal_CV_ctrl_t CV_EV;

extern uint8_t visual_rx_buffer1[sizeof(CV_R_t)]; // �������ݻ�������

extern float yaw_angle_get, pitch_angle_get; // ��λ����

extern void visual_reception(void const *argument);

extern void NucToMcu(uint8_t *Buf, const uint32_t *Len);
#endif
