/**
  ****************************(C) COPYRIGHT 2023 ULTRA****************************
  * @file       amend_path_task.c/h
  * @brief      FreeRTOS任务，哨兵路径姿态修正任务
  *
  * @note
  * @history
  *  Version    Date            Author          Modification
	*  V1.0.0     2023-1-15       天衍             1.done.
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 ULTRA****************************
  */

#include "path_planning_task.h"
#include "amend_path_task.h"
#include "pid.h"
#include "chassis_task.h"
#include "freertos.h"
#include "task.h"
#include "chassis_behaviour.h"
#include "usart.h"
#include "cmsis_os.h"
#include "bsp_spi_master.h"
#include "path_planning_task.h"
#include "stdio.h"
#include "math.h"
#include "attitude_task.h"
#include <stdio.h>
#include <math.h>

TaskHandle_t correction_local_handler; // 任务句柄

unsigned char correction_start_flag = 0;

static void correction_set(float x, float y, unsigned char laser_number);
static void correction_spining_set(unsigned char target);

Typedef_CORRECTION yaw_correction;

/**
 * @brief 姿态修正任务
 * @param None.
 * @retval None.
 */
void correction(void const *argument)
{
	vTaskDelay(CORRECTION_INIT_TIME);
	// get task handle, must enable 'xTaskGetHandle' in cubeMX
	// 获取任务句柄，必须在CubeMX使能'xtaskGetHand'
	correction_local_handler = xTaskGetHandle(pcTaskGetName(NULL));
	correction_start_flag = 1; // 当一切就绪后，将标志位置1，表示可以开始进行姿态控制
	while (1)
	{
		// wait for task waked up
		// 等待任务被唤醒
		while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
		{
		}
		if (!correction_flag)
		{
			correction_flag++;
			correction_set(yaw_correction.x, yaw_correction.y, yaw_correction.laser_choose);
		}
		else
		{
			correction_spining_set(distance.target_enhancing);
		}
	}
}

/**
 * @brief 路径修正函数
 * @param 修正区间(x,0)或者(0,y),使用的修正激光
 * @retval None.
 */
static void correction_set(float x, float y, unsigned char laser_number)
{
	unsigned char finish_correction_flag = 0;
	unsigned short int last_laser_distance;
	unsigned short int laser_distance;
	float last_coordinate;
	float coordinate;
	double laser_distance_err;
	double coordinate_err;
	while (!finish_correction_flag)
	{
		while ((distance.mod == GUARD_MODE) || (distance.mod == RELOCATE_MOD)) // 正在旋转，停止修正
		{
			vTaskDelay(1);
		}
		if (laser.state == LASER_OFF) // 若激光测距出错，直接返回
		{
			return;
		}
		last_laser_distance = laser_distance;
		last_coordinate = coordinate;
		switch (laser_number)
		{
		case (laser_head):
			if (x != 0)
			{
				laser_distance = laser.head;
				laser_distance_err = laser_distance - last_laser_distance;
				coordinate = coordinates.coordinates_x;
				coordinate_err = coordinate - last_coordinate;
				distance.correction_yaw_angle = -atan2(laser_distance_err, coordinate_err);
			}
			else if (y != 0)
			{
				laser_distance = laser.head;
				laser_distance_err = laser_distance - last_laser_distance;
				coordinate = coordinates.coordinates_y;
				coordinate_err = coordinate - last_coordinate;
				distance.correction_yaw_angle = -atan2(laser_distance_err, coordinate_err);
			}
			break;
		case (laser_back):
			if (x != 0)
			{
				laser_distance = laser.back;
				laser_distance_err = laser_distance - last_laser_distance;
				coordinate = coordinates.coordinates_x;
				coordinate_err = coordinate - last_coordinate;
				distance.correction_yaw_angle = -atan2(laser_distance_err, coordinate_err);
			}
			else if (y != 0)
			{
				laser_distance = laser.back;
				laser_distance_err = laser_distance - last_laser_distance;
				coordinate = coordinates.coordinates_y;
				coordinate_err = coordinate - last_coordinate;
				distance.correction_yaw_angle = -atan2(laser_distance_err, coordinate_err);
			}
			break;
		case (laser_left):
			if (x != 0)
			{
				laser_distance = laser.left;
				laser_distance_err = laser_distance - last_laser_distance;
				coordinate = coordinates.coordinates_x;
				coordinate_err = coordinate - last_coordinate;
				distance.correction_yaw_angle = -atan2(laser_distance_err, coordinate_err);
			}
			else if (y != 0)
			{
				laser_distance = laser.left;
				laser_distance_err = laser_distance - last_laser_distance;
				coordinate = coordinates.coordinates_y;
				coordinate_err = coordinate - last_coordinate;
				distance.correction_yaw_angle = -atan2(laser_distance_err, coordinate_err);
			}
			break;
		case (laser_right):
			if (x != 0)
			{
				laser_distance = laser.right;
				laser_distance_err = laser_distance - last_laser_distance;
				coordinate = coordinates.coordinates_x;
				coordinate_err = coordinate - last_coordinate;
				distance.correction_yaw_angle = -atan2(laser_distance_err, coordinate_err);
			}
			else if (y != 0)
			{
				laser_distance = laser.right;
				laser_distance_err = laser_distance - last_laser_distance;
				coordinate = coordinates.coordinates_y;
				coordinate_err = coordinate - last_coordinate;
				distance.correction_yaw_angle = -atan2(laser_distance_err, coordinate_err);
			}
		}
		if (((x > 0) && (coordinate > x)) || ((x < 0) && (coordinate < x)))
		{
			finish_correction_flag = 1;
		}
		else if (((y > 0) && (coordinate > y)) || ((y < 0) && (coordinate < y)))
		{
			finish_correction_flag = 1;
		}
		vTaskDelay(25);
	}
}

/**
 * @brief 小陀螺路径修正
 * @param  目标墙
 * @retval None.
 */
static void correction_spining_set(unsigned char target)
{
	float mesure_angle;
	unsigned short int actual_distance;
	if (target == enhancing_head)
	{
		if ((chassis_move.chassis_yaw - distance.start_yaw_angle <= LASER_RANGE_ANGLE) && (chassis_move.chassis_yaw - distance.start_yaw_angle >= (-LASER_RANGE_ANGLE)))
		{
			mesure_angle = chassis_move.chassis_yaw - distance.start_yaw_angle;
			actual_distance = (laser.head + OFFSET_HEAD_DISTANCE * tan((double)(mesure_angle)) + OFFSET_X_HEAD_DISTANCE) * cos((double)(mesure_angle));
		}
		if (actual_distance >= (DISTANCE_TO_HEAD_ENHANCING + laser_head))
		{
			coordinates.coordinates_error_x = coordinates.coordinates_error_x - CORRECT_CONSTANT;
			vTaskDelay(CORRECTION_WAIT_TIME);
		}
		else if (actual_distance <= (DISTANCE_TO_HEAD_ENHANCING - laser_head))
		{
			coordinates.coordinates_error_x = coordinates.coordinates_error_x + CORRECT_CONSTANT;
			vTaskDelay(CORRECTION_WAIT_TIME);
		}
	}
}
