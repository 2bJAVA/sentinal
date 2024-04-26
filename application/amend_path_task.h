#ifndef __AMEND_PATH_TASK_H__
#define __AMEND_PATH_TASK_H__

#include "struct_typedef.h"
#include "freertos.h"
#include "task.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define CORRECTION_INIT_TIME 5
#define CORRECTION_WAIT_TIME 500		// 修正误差后的等待时间
#define LASER_RANGE_ANGLE 0.3f			// 激光修正允许范围
#define LASER_ERROR_RANGE 250			// 修正误差范围
#define DISTANCE_TO_HEAD_ENHANCING 1500 // 设定离墙距离
#define CORRECT_CONSTANT 10				// 修正正交系统常数

	typedef struct
	{
		float x;
		float y;
		unsigned char laser_choose;
	} Typedef_CORRECTION;

	extern Typedef_CORRECTION yaw_correction;

	extern void correction(void const *argument);
	extern TaskHandle_t correction_local_handler; // 任务句柄
	extern unsigned char correction_start_flag;

#endif
