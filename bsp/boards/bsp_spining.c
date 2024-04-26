/* ****************************(C) COPYRIGHT 2023 ULTRA****************************
  * @file       bsp_spining.c/h
  * @brief      小陀螺算法。在5.2中改为视觉接管，也许会有问题？
  *             
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023.3.10       天衍            1.done.      
	*  V1.0.1     2023.5.2        天衍            2.add.
  *
  @verbatim 
  ==============================================================================
   偷个懒，相信你们能看懂
  ==============================================================================
  @endverbatim 将spining_arithmetic函数，扔进底盘chassis_behaviour.c中的该任务处理
	之后即可，例如chassis_open_set_control()之后，注：一定要放在代码执行最后一行。
  ****************************(C) COPYRIGHT 2023 ULTRA****************************
  */
#include "bsp_spining.h"
#include "attitude_task.h"
#include <stdio.h>
#include <math.h>
#include "user_lib.h"
#include "chassis_behaviour.h"
#include "pid.h"
#include "path_task.h"

//float debug_angle;

/**
   * @brief 小陀螺算法
   * @param  Vx，Vy，Wz
   * @retval None.
   */
void spining_arithmetic(fp32 * Vx,fp32 * Vy,fp32 *Wz)
{
	fp32 sin_yaw = 0.0f, cos_yaw = 0.0f; 
	fp32 spining_t[3];	
//	set_speed = invSqrt(((*Vx)*(*Vx)+(*Vy)*(*Vy)));
//	 
//	 if(set_speed >= (*Wz/SPINING_CONSTANT))
//	 {
//		 set_speed = *Wz/SPINING_CONSTANT;
//	 }
	
   sin_yaw = sin(-(-(chassis_move.chassis_yaw + path_control.yaw_offset)- distance.spining_angle));
   cos_yaw = cos(-(-(chassis_move.chassis_yaw + path_control.yaw_offset)- distance.spining_angle));

	 spining_t[0]=*Vx;
	 spining_t[1]=sin_yaw;
	 spining_t[2]=-(spining_t[0]*spining_t[1]);
	
   *Vx = (cos_yaw * (*Vx) + sin_yaw * (*Vy)) * SENTINEL_CHASSIS_GY_SEN;
   *Vy = (spining_t[2] + cos_yaw * (*Vy)) * SENTINEL_CHASSIS_GY_SEN;
	 
//	 distance.spining_angle = set_angle;
}

