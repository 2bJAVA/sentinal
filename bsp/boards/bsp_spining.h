/* ****************************(C) COPYRIGHT 2023 ULTRA****************************
  * @file       bsp_spining.c/h
  * @brief      С�����㷨����5.2�и�Ϊ�Ӿ��ӹ�
  *             
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023.3.10       ����            1.done.      
	*  V1.0.1     2023.5.2        ����            2.add.
  *
  @verbatim 
  ==============================================================================
   ͵���������������ܿ���
  ==============================================================================
  @endverbatim ��spining_arithmetic�������ӽ�����chassis_behaviour.c�еĸ�������
	֮�󼴿ɣ�����chassis_open_set_control()֮��ע��һ��Ҫ���ڴ���ִ�����һ�С�
  ****************************(C) COPYRIGHT 2023 ULTRA****************************
  */
	
#ifndef __BSP_SPINING_H__
#define __BSP_SPINING_H__

#ifdef __cplusplus
extern "C" {
#endif
	
#include "struct_typedef.h"
#include "arm_math.h"
	
#define Vx_MAX_LIMIT 5.0f //Vx�ٶ��޷�
#define Vy_MAX_LIMIT 5.0f //Vy�ٶ��޷�
#define SPINING_CONSTANT 3.5f
	
#define BSP_PI 3.1415926535897932384626433832795f

extern void spining_arithmetic(fp32 * Vx,fp32 * Vy,fp32 *Wz);
	
//extern void spining_correct_arithmetic(fp32 *Vx,fp32 *Vy);
	
#endif
