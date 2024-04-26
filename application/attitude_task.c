/**
  ****************************(C) COPYRIGHT 2023 ULTRA****************************
  * @file       attitude.c/h
  * @brief      FreeRTOS任务，底盘姿态处理任务->合并入chassis_task.c,并不是任务
  *
  * @note
  * @history
  *  Version    Date            Author          Modification
	*  V1.0.0     2023-2-24       天衍             1.done.
	*  V1.0.1     2023-5-2        天衍             2.combine to chassis_task.C.
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
 * @brief 底盘角度环控制初始化
 * @param distance 结构体
 * @retval None.
 */
void chassis_angle_control_init(Typedef_DISTANCE *DIS)
{
	DIS->x = 0; // 直线修正
	DIS->y = 0; // 左右修正
	DIS->z = 0; // 转弯修正
	DIS->spining_angle = 0;
	DIS->start_yaw_angle = 0;	   // 初始yaw角
	DIS->rotating_yaw_angle = 0;   // 旋转后yaw角
	DIS->correction_yaw_angle = 0; // 修正yaw角
	DIS->remember_yaw_angle = 0;
	DIS->spining_flag = 0;
	DIS->control_priority = 0;
}

/**
 * @brief yaw初始角获取
 * @param distance 结构体
 * @retval None.
 */
void yaw_init_set(Typedef_DISTANCE *DIS)
{
	DIS->start_yaw_angle = chassis_move.chassis_yaw;
}

/**
 * @brief 初始YAW角度获取
 * @param None.
 * @retval None.
 */
void sentinel_get_start_yaw()
{
	int i, j;
	float temp;
	while (attitude_yaw_init_flag < INS_ADVACED_INIT_TIMES) // 等待数据结束
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
	while (attitude_yaw_init_flag < INS_COLLECT_DATA_TIMES) // 收集有效数据
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
	for (i = 1; i <= INS_COLLECT_DATA_TIMES; i++) // 冒泡排序法排序
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
	if (INS_COLLECT_DATA_TIMES % 2 == 0) // 如果数据总量为偶数
	{
		distance.start_yaw_angle = (INS_COLLECT_ARR[INS_COLLECT_DATA_TIMES / 2] + INS_COLLECT_ARR[INS_COLLECT_DATA_TIMES / 2 - 1]) / 2.0f;
	}
	else if (INS_COLLECT_DATA_TIMES % 2 != 0) // 如果数据重量为奇数
	{
		distance.start_yaw_angle = INS_COLLECT_ARR[(INS_COLLECT_DATA_TIMES + 1) / 2];
	}
}

float filter_distance_z = 0.7504915783575617180771870304501f;
float filter_correct_z = 0.005f;
unsigned char filter_distance_z_flag = 0;

/**
   * @brief 正弦小陀螺
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
