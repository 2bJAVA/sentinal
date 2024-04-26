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
#define CORRECTION_WAIT_TIME 500		// ��������ĵȴ�ʱ��
#define LASER_RANGE_ANGLE 0.3f			// ������������Χ
#define LASER_ERROR_RANGE 250			// ������Χ
#define DISTANCE_TO_HEAD_ENHANCING 1500 // �趨��ǽ����
#define CORRECT_CONSTANT 10				// ��������ϵͳ����

	typedef struct
	{
		float x;
		float y;
		unsigned char laser_choose;
	} Typedef_CORRECTION;

	extern Typedef_CORRECTION yaw_correction;

	extern void correction(void const *argument);
	extern TaskHandle_t correction_local_handler; // ������
	extern unsigned char correction_start_flag;

#endif
