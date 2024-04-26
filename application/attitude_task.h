#ifndef __ATTITUDE_TASK_H__
#define __ATTITUDE_TASK_H__

#include "freertos.h"
#include "task.h"
#include "pid.h"
#include "chassis_behaviour.h"

#define SENTINEL_DEBUG_MOD_CONSTANT 0.4f // 步兵模式下的比例系数
#define SENTINEL_CHASSIS_GY_SEN 0.5f

#define SENTINEL_CHASSIS_SPEED 0.5f	 // 位于0到4之间
#define SENTINEL_MIN_ROTATION_SPEED 4.0f // 哨兵旋转速度7.5
#define SENTINEL_FILTER_ROTATION_SPERD 2.0f //额外速度4.0
#define SENTINEL_FILTER_CORRECTION  0.0f //0.7504915783575617180771870304501f

#define INS_ADVACED_INIT_TIMES 500 // 预收集500次陀螺仪数据
#define INS_COLLECT_DATA_TIMES 50  // 收集50次有效陀螺仪数据
#define COLLEDCT_INS_WAIT_TIME 25  // 采集数据时间间隔

typedef enum
{
	STRAIGHT_MODE_UP = 1,			 // 直线模式->前进
	STRAIGHT_MODE_BACK,				 // 直线模式->后退
	TRANSLATION_MODE_UP,			 // 平移模式->前进
	TRANSLATION_MODE_BACK,			 // 平移模式->后退
	GUARD_MODE,						 // 警戒模式
	STOP_GUARD,						 // 停止警戒
	RELOCATE_MOD,					 // 修正警戒
	STRAIGHT_MODE_UP_WITHOUT_SPINING // 前进且不使用小陀螺
} Typedef_DISTANCE_MODE;

extern void chassis_angle_control_init(Typedef_DISTANCE *DIS);
extern void yaw_init_set(Typedef_DISTANCE *DIS);
extern void sentinel_get_start_yaw(void);
extern void filter_attitude_dispose(void);

#endif
