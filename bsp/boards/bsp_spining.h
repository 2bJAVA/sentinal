/* ****************************(C) COPYRIGHT 2023 ULTRA****************************
  * @file       bsp_spining.c/h
  * @brief      小陀螺算法，在5.2中改为视觉接管
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
	
#ifndef __BSP_SPINING_H__
#define __BSP_SPINING_H__

#ifdef __cplusplus
extern "C" {
#endif
	
#include "struct_typedef.h"
#include "arm_math.h"
	
#define Vx_MAX_LIMIT 5.0f //Vx速度限幅
#define Vy_MAX_LIMIT 5.0f //Vy速度限幅
#define SPINING_CONSTANT 3.5f
	
#define BSP_PI 3.1415926535897932384626433832795f

extern void spining_arithmetic(fp32 * Vx,fp32 * Vy,fp32 *Wz);
	
//extern void spining_correct_arithmetic(fp32 *Vx,fp32 *Vy);
	
#endif
