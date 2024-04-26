/**
  ****************************(C) COPYRIGHT 2023 ULTRA****************************
  * @file       attitude.c/h
  * @brief      FreeRTOS���񣬵�����̬��������->�ϲ���chassis_task.c,����������
  *
  * @note
  * @history
  *  Version    Date            Author          Modification
	*  V1.0.0     2023-2-24       ����             1.done.
	*  V1.0.1     2023-5-2        ����             2.combine to chassis_task.C.
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 ULTRA****************************
  */
#include "attitude_task.h"
#include "chassis_task.h"
#include "freertos.h"
#include "task.h"
#include "chassis_behaviour.h"
#include "usart.h"
#include "cmsis_os.h"
#include "calibrate_task.h"
#include "bsp_rng.h"
#include "bsp_spining.h"
#include <stdio.h>
#include <math.h>
#include "fire_control_system.h"

unsigned short int attitude_yaw_init_flag = 0;
unsigned short int array_counter = 0;

unsigned char attitude_change_flag = 0;

float INS_COLLECT_ARR[INS_COLLECT_DATA_TIMES];
float SPEED_CONSTANT = (float)(180000.0f / SENTINEL_CHASSIS_SPEED);


/**
 * @brief ���̽ǶȻ����Ƴ�ʼ��
 * @param distance �ṹ��
 * @retval None.
 */
void chassis_angle_control_init(Typedef_DISTANCE *DIS)
{
	DIS->x = 0; // ֱ������
	DIS->y = 0; // ��������
	DIS->z = 0; // ת������
	DIS->spining_angle = 0;
	DIS->start_yaw_angle = 0;	   // ��ʼyaw��
	DIS->rotating_yaw_angle = 0;   // ��ת��yaw��
	DIS->correction_yaw_angle = 0; // ����yaw��
	DIS->remember_yaw_angle = 0;
	DIS->spining_flag = 0;
	DIS->control_priority = 0;
}

/**
 * @brief yaw��ʼ�ǻ�ȡ
 * @param distance �ṹ��
 * @retval None.
 */
void yaw_init_set(Typedef_DISTANCE *DIS)
{
	DIS->start_yaw_angle = chassis_move.chassis_yaw;
}

/**
 * @brief ��ʼYAW�ǶȻ�ȡ
 * @param None.
 * @retval None.
 */
void sentinel_get_start_yaw()
{
	int i, j;
	float temp;
	while (attitude_yaw_init_flag < INS_ADVACED_INIT_TIMES) // �ȴ����ݽ���
	{
		if ((distance.start_yaw_angle == 0) || (attitude_yaw_init_flag <= INS_ADVACED_INIT_TIMES))
		{
			yaw_init_set(&distance);

		}
		if ((distance.start_yaw_angle != 0) && (attitude_yaw_init_flag <= INS_ADVACED_INIT_TIMES))
		{
			attitude_yaw_init_flag++;
		}
	}
	attitude_yaw_init_flag = 0;
	while (attitude_yaw_init_flag < INS_COLLECT_DATA_TIMES) // �ռ���Ч����
	{
		if (distance.start_yaw_angle != 0)
		{
			INS_COLLECT_ARR[array_counter] = distance.start_yaw_angle;
			array_counter++;
			attitude_yaw_init_flag++;
			yaw_init_set(&distance);

		}
		else if (distance.start_yaw_angle == 0)
		{
			yaw_init_set(&distance);
		}
	}
	for (i = 1; i <= INS_COLLECT_DATA_TIMES; i++) // ð����������
	{
		for (j = 0; j <= INS_COLLECT_DATA_TIMES - i; j++)
		{
			if (INS_COLLECT_ARR[j] > INS_COLLECT_ARR[j + 1])
			{
				temp = INS_COLLECT_ARR[j];
				INS_COLLECT_ARR[j] = INS_COLLECT_ARR[j + 1];
				INS_COLLECT_ARR[j + 1] = temp;
			}
		}

	}
	if (INS_COLLECT_DATA_TIMES % 2 == 0) // �����������Ϊż��
	{
		distance.start_yaw_angle = (INS_COLLECT_ARR[INS_COLLECT_DATA_TIMES / 2] + INS_COLLECT_ARR[INS_COLLECT_DATA_TIMES / 2 - 1]) / 2.0f;
	}
	else if (INS_COLLECT_DATA_TIMES % 2 != 0) // �����������Ϊ����
	{
		distance.start_yaw_angle = INS_COLLECT_ARR[(INS_COLLECT_DATA_TIMES + 1) / 2];
	}
}

float filter_distance_z = 0.7504915783575617180771870304501f;
float filter_correct_z = 0.005f;
unsigned char filter_distance_z_flag = 0;

/**
   * @brief ����С����
   * @param None.
   * @retval None.
   */
void filter_attitude_dispose()
{
	if(!filter_distance_z_flag)
	{
	  distance.z = SENTINEL_FILTER_ROTATION_SPERD * sin(filter_distance_z) + SENTINEL_MIN_ROTATION_SPEED;
		filter_distance_z = filter_distance_z + filter_correct_z;
	}
	else
	{
	  distance.z = SENTINEL_FILTER_ROTATION_SPERD * sin(filter_distance_z) + SENTINEL_MIN_ROTATION_SPEED;
		filter_distance_z = filter_distance_z - filter_correct_z;
	}
	if(filter_distance_z > BSP_PI / 2.0f)
	{
		filter_distance_z_flag = 1;
	}
	else if(filter_distance_z < SENTINEL_FILTER_CORRECTION)
	{
		filter_distance_z_flag = 0;
	}

}
