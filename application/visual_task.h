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
#define VISION_CONTROL_TIME 	120  //范围100~200 
	
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
	uint8_t state; // 设计状态
	// /*
	// LOST				0x00
	// FIND_NO_SHOOT	0x01
	// FIND_SHOOT		0x02
	// FINDING			0x03
	// */
	float pitch; // pitch电机角度
	float yaw;   // yaw电机角度
	// 0：不允许，1：允许
	uint8_t relive_flag;      // 复活允许指令
	uint8_t pill_flag;        // 允许兑换弹丸指令
	uint16_t pill_17mm_count; // 需要兑换的17mm子弹数
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

// 设置参数
struct SolveTrajectoryParams
{
	float k; // 弹道系数

	// 自身参数
	enum BULLET_TYPE bullet_type; // 自身机器人类型 0-步兵 1-英雄
	float current_v;              // 当前弹速
	float current_pitch;          // 当前pitch
	float current_yaw;            // 当前yaw

	// 目标参数
	float xw;                 // ROS坐标系下的x
	float yw;                 // ROS坐标系下的y
	float zw;                 // ROS坐标系下的z
	float vxw;                // ROS坐标系下的vx
	float vyw;                // ROS坐标系下的vy
	float vzw;                // ROS坐标系下的vz
	float tar_yaw;            // 目标yaw
	float v_yaw;              // 目标yaw速度
	float r1;                 // 目标中心到前后装甲板的距离
	float r2;                 // 目标中心到左右装甲版的距离
	float dz;                 // 另一对装甲板的相对于被跟踪的装甲板的高度差
	int bias_time;            // 偏差时间
	float s_bias;             // 枪口前推的距离
	float z_bias;             // yaw轴电机到枪口水平的垂直距离
	enum ARMOR_ID armor_id;   // 装甲板类型  0-outpost 6-guard 7-base
														// 1-英雄 2-工程 3-4-5-步兵
	enum ARMOR_NUM armor_num; // 装甲板数字  2-balance 3-outpost 4-normal

	float aim_x;
	float aim_y;
	float aim_z;
};

// 用于存储目标装甲板的信息
struct tar_pos
{
	float x;   // 装甲板在世界坐标系下的x
	float y;   // 装甲板在世界坐标系下的y
	float z;   // 装甲板在世界坐标系下的z
	float yaw; // 装甲板坐标系相对于世界坐标系的yaw角
};

extern ext_gimbal_CV_ctrl_t CV_EV;

extern uint8_t visual_rx_buffer1[sizeof(CV_R_t)]; // 接收数据缓存数组

extern float yaw_angle_get, pitch_angle_get; // 单位：度

extern void visual_reception(void const *argument);

extern void NucToMcu(uint8_t *Buf, const uint32_t *Len);
#endif
