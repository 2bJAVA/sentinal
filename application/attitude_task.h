#ifndef __ATTITUDE_TASK_H__
#define __ATTITUDE_TASK_H__

#include "freertos.h"
#include "task.h"
#include "pid.h"
#include "chassis_behaviour.h"

#define SENTINEL_DEBUG_MOD_CONSTANT 0.4f // ����ģʽ�µı���ϵ��
#define SENTINEL_CHASSIS_GY_SEN 0.5f

#define SENTINEL_CHASSIS_SPEED 0.5f	 // λ��0��4֮��
#define SENTINEL_MIN_ROTATION_SPEED 4.0f // �ڱ���ת�ٶ�7.5
#define SENTINEL_FILTER_ROTATION_SPERD 2.0f //�����ٶ�4.0
#define SENTINEL_FILTER_CORRECTION  0.0f //0.7504915783575617180771870304501f

#define INS_ADVACED_INIT_TIMES 500 // Ԥ�ռ�500������������
#define INS_COLLECT_DATA_TIMES 50  // �ռ�50����Ч����������
#define COLLEDCT_INS_WAIT_TIME 25  // �ɼ�����ʱ����

typedef enum
{
	STRAIGHT_MODE_UP = 1,			 // ֱ��ģʽ->ǰ��
	STRAIGHT_MODE_BACK,				 // ֱ��ģʽ->����
	TRANSLATION_MODE_UP,			 // ƽ��ģʽ->ǰ��
	TRANSLATION_MODE_BACK,			 // ƽ��ģʽ->����
	GUARD_MODE,						 // ����ģʽ
	STOP_GUARD,						 // ֹͣ����
	RELOCATE_MOD,					 // ��������
	STRAIGHT_MODE_UP_WITHOUT_SPINING // ǰ���Ҳ�ʹ��С����
} Typedef_DISTANCE_MODE;

extern void chassis_angle_control_init(Typedef_DISTANCE *DIS);
extern void yaw_init_set(Typedef_DISTANCE *DIS);
extern void sentinel_get_start_yaw(void);
extern void filter_attitude_dispose(void);

#endif
