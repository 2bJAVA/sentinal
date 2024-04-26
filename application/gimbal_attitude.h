/**
  ****************************(C) COPYRIGHT 2023 ULTRA****************************
  * @file       gimbal_attitude.c/h
  * @brief      FreeRTOS任务，哨兵云台处理任务
  *             
  * @note       
  * @history
  *  Version    Date            Author          Modification
	*  V1.0.0     2022-1-6        天衍             1.done
	*  V1.0.1     2023-2-21       天衍             2.add_hit_feed_back
	*  V1.0.2     2023-5-2        天衍             3.standard
	*  V2.0.0     2023-9-18       天衍             4.完善，修改，整理  
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 ULTRA****************************
  */
#ifndef __GIMBAL_ATTITUDE_H__
#define __GIMBAL_ATTITUDE_H__

#include "struct_typedef.h"
#include "freertos.h"
#include "task.h"
#include "pid.h"

#define GIMBAL_ATTITUDE_INIT_TIME 17

#define GIMBAL_STATE_CHANGE_TIME 1000 //云台模式更改等待时间

#define GIMBAL_YAW_RANGE      0.05f //角度允许误差范围，单位rad   

#define SCAN_PITCH_ANGLE   0.0f  

#define START_YAW 0.0f  //扫描模式中心点(-pi,+pi)
#define STATE_YAW 4.0f  //扫描模式范围（+-中心点）确保中心点+-扫描角度位于(-pi,0)(0,pi)范围内

#define FEED_BACK_START_YAW 0.0f
#define FEED_BACK_STATE_YAW 4.0f //受击后扫描

#define SENTINEL_GIMBAL_YAW_NORMAL_SPEED   -0.005f//扫描模式下云台转速(新哨兵采用传送带，故这里选择add_angle，以防止疯转)
#define SENTINEL_GIMBAL_PITCH_NORMAL_SPEED 0.0015f //扫描模式下的PITCH转速

#define SENTINEL_PITCH_MAX_ANGLE 0.20f //最大扫描角
#define SENTINEL_PITCH_MIN_ANGLE -0.1f //最小扫描角

#define GIMBAL_ACCEL_YAW_NUM      0.03f
#define GIMBAL_ACCEL_PITCH_NUM     0.10f

#define FOUND_ENEMY 1
#define NOT_FOUND   0
#define CV_EV_ERROR 3
#define FRIC_OPEN   0x36
#define FRIC_CLOSE  0X39

#define CV_EV_WAIT_TIME 50

#define GIMBAL_INIT_WAIT_TIME 20 //等待云台初始化时间(无单位)

#define GIMBAL_TURN_YAW_WAIT_TIME 2000 //云台翻转等待时间(单位ms)

typedef enum
{
	GIMBAL_SCAN_MOD=0x98,       //扫描模式
	GIMBAL_TRACK_ENEMY_MOD=0xAA, //锁敌模式
	GIMBAL_NONE //空模式，用于原代码初始化
}Typedef_SENTINEL_MOD;

extern Typedef_SENTINEL_MOD sentinel_gimbal_behaviour;

extern void gimbal_attitude_control(void const * argument);
extern TaskHandle_t gimbal_attitude_local_handler;    //任务句柄
extern unsigned char gimbal_attitude_start_flag;
extern float yaw_speed_control(fp32 yaw_speed);
extern float pitch_speed_control(fp32 pitch_speed);
extern unsigned char correlate_data(void);
extern unsigned char SCAN_FEED_BACK(void);
//extern float yaw_speed_control_x(float angle);

#endif
