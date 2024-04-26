/**
  ****************************(C) COPYRIGHT 2023 ULTRA****************************
  * @file       gimbal_attitude.c/h
  * @brief      FreeRTOS�����ڱ���̨��������
  *             
  * @note       
  * @history
  *  Version    Date            Author          Modification
	*  V1.0.0     2022-1-6        ����             1.done
	*  V1.0.1     2023-2-21       ����             2.add_hit_feed_back
	*  V1.0.2     2023-5-2        ����             3.standard
	*  V2.0.0     2023-9-18       ����             4.���ƣ��޸ģ�����  
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

#define GIMBAL_STATE_CHANGE_TIME 1000 //��̨ģʽ���ĵȴ�ʱ��

#define GIMBAL_YAW_RANGE      0.05f //�Ƕ�������Χ����λrad   

#define SCAN_PITCH_ANGLE   0.0f  

#define START_YAW 0.0f  //ɨ��ģʽ���ĵ�(-pi,+pi)
#define STATE_YAW 4.0f  //ɨ��ģʽ��Χ��+-���ĵ㣩ȷ�����ĵ�+-ɨ��Ƕ�λ��(-pi,0)(0,pi)��Χ��

#define FEED_BACK_START_YAW 0.0f
#define FEED_BACK_STATE_YAW 4.0f //�ܻ���ɨ��

#define SENTINEL_GIMBAL_YAW_NORMAL_SPEED   -0.005f//ɨ��ģʽ����̨ת��(���ڱ����ô��ʹ���������ѡ��add_angle���Է�ֹ��ת)
#define SENTINEL_GIMBAL_PITCH_NORMAL_SPEED 0.0015f //ɨ��ģʽ�µ�PITCHת��

#define SENTINEL_PITCH_MAX_ANGLE 0.20f //���ɨ���
#define SENTINEL_PITCH_MIN_ANGLE -0.1f //��Сɨ���

#define GIMBAL_ACCEL_YAW_NUM      0.03f
#define GIMBAL_ACCEL_PITCH_NUM     0.10f

#define FOUND_ENEMY 1
#define NOT_FOUND   0
#define CV_EV_ERROR 3
#define FRIC_OPEN   0x36
#define FRIC_CLOSE  0X39

#define CV_EV_WAIT_TIME 50

#define GIMBAL_INIT_WAIT_TIME 20 //�ȴ���̨��ʼ��ʱ��(�޵�λ)

#define GIMBAL_TURN_YAW_WAIT_TIME 2000 //��̨��ת�ȴ�ʱ��(��λms)

typedef enum
{
	GIMBAL_SCAN_MOD=0x98,       //ɨ��ģʽ
	GIMBAL_TRACK_ENEMY_MOD=0xAA, //����ģʽ
	GIMBAL_NONE //��ģʽ������ԭ�����ʼ��
}Typedef_SENTINEL_MOD;

extern Typedef_SENTINEL_MOD sentinel_gimbal_behaviour;

extern void gimbal_attitude_control(void const * argument);
extern TaskHandle_t gimbal_attitude_local_handler;    //������
extern unsigned char gimbal_attitude_start_flag;
extern float yaw_speed_control(fp32 yaw_speed);
extern float pitch_speed_control(fp32 pitch_speed);
extern unsigned char correlate_data(void);
extern unsigned char SCAN_FEED_BACK(void);
//extern float yaw_speed_control_x(float angle);

#endif
