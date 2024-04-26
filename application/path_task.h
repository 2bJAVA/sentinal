/**
  ****************************(C) COPYRIGHT 2023 ULTRA****************************
  * @file       path.c/h
  * @brief      path control task,
  *             ·����������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-11-28      ����              1. done
	*  V2.0.0     2024-3-4        ����              2.�ںϽ�����
  *
  @verbatim
  ==============================================================================
	ʹ��˵��������path_init�м������õ�·�������õ�ģʽ�����Ժ�λ�û�PID�󼴿�ʹ�ã�
	path_init���г�ʼ·���������ӣ�

	path_not_event_set(path_control_init,0,50000.0f,0.0f,LINE,5000);
	path_not_event_set(path_control_init,1,-50000.0f,0.0f,LINE,5000);
	
	//ѭ������Ϊ1
	path_cir_set(path_control_init,0,0,1,1,0);
	
	path_not_event_set(path_control_init,100,20000.0f,0.0f,AROUND,5000);
	path_not_event_set(path_control_init,101,-20000.0f,0.0f,AROUND,5000);
	
	//·����д
	path_cir_set(path_control_init,1,100,101,UNLIMIT_TIME,0);
	
	�������󣺻���ǰ��50000.0f��������λ��Ȼ������50000.0f�ı�������λ������
	��������ƽ������ƽ��20000.0f��������λ��ѭ��
  ==============================================================================
  @endverbatim
	ע�����ʹ�ñ�����ʱ��Ĭ��ʧ���Ӿ�mid360�״����ʹ��ʱ��ע�͵���freertos����
	����
  ****************************(C) COPYRIGHT 2023 ULTRA****************************
  */
	
#ifndef __PATH_TASK_H__
#define __PATH_TASK_H__

#include "struct_typedef.h"
#include "freertos.h"
#include "task.h"
#include "pid.h"
#include "user_lib.h"

#define PATH_TASK_INIT_TIME 600//650
#define PATH_TASK_WAIT_TIME 1

#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191

#define LINE	 0 //ֱ��ģʽ
#define AROUND 1 //����ģʽ

#define EVENT     0 //�¼�����ģʽ
#define NOT_EVENT 1 //��ʱģʽ

#define ANGLE_CHANGE_CONSTANT    0.1f
#define ANGLE_CHANGE_WAIT_TIME   200
#define ANGLE_CHANGE_WAIT_TIME_X 500

#define ECD_TRANSFER 0.01f
#define ECD_ERROR    115.0f //�������Χ120.0f

//#define PATH_SCAN_MAX_COUNT 5000

#define UNLIMIT_TIME 0xffff

//λ�û�PID
#define PATH_SPEED_PID_KP        0.0002f
#define PATH_SPEED_PID_KI        0.001f
#define PATH_SPEED_PID_KD        0.005f
#define PATH_SPEED_PID_MAX_OUT   0.4f//0.5
#define PATH_SPEED_PID_MAX_IOUT  0.0f

//����PID
#define LASER_SPEED_PID_KP        0.8f
#define LASER_SPEED_PID_KI        0.02f
#define LASER_SPEED_PID_KD        0.0f
#define LASER_SPEED_PID_MAX_OUT   0.25f
#define LASER_SPEED_PID_MAX_IOUT  0.05f

//�����Ƿ�ʹ��
#define LASER_ENABLE  1
#define LASER_DISABLE 0

//�������ȼ�
#define NOT_SET_PRIORITY   0
#define AMEND_X_PRIORITY   1
#define AMEND_Y_PRIORITY   2

//������������λm
#define LASER_ERROR_X   0.15f
#define LASER_ERROR_Y   0.15f

//����ǽ����������x1 + x2 + MACHINE_X - laser_x1_set - laser_x2_set����С���������Ϊû���ϰ���
#define LOCATE_BARRIER_ADMIT 0.5f

//��е��������������֮��ļ��
#define MACHINE_X 0.2f
#define MACHINE_Y 0.2f

//����Ҫôȫ�ã�Ҫôʧ������ĳЩ���Ӷ��ﵽ���һ���ϰ�����ǣ�X1��X2����ͬʱʧ�ܣ�Y1��Y2����ͬʱʧ��
#define USE_ALL_LASER 				0x00
#define DISABLE_X1_LASER  		0x11
#define DISABLE_X2_LASER  		0x12
#define DISABLE_Y1_LASER  		0x13
#define DISABLE_Y2_LASER  		0x14
#define DISABLE_X1_Y1_LASER 	0x21
#define DISABLE_X1_Y2_LASER   0x22
#define DISABLE_X2_Y1_LASER   0x23
#define DISABLE_X2_Y2_LASER   0x24

#define EVENT_NORMAL_DELAY_TIME 60

#define SET_COUNT 5

#define CHASSIS_SEND_DATA 7
#define ACCEPT_LASER_DATA 19

#define SEND_EOF 0x5A
#define SEND_NOF 0xFF

//����
#define SCHEM_ATTACK_ANGLE_MIN 	 	-PI / 4.0f
#define SCHEM_ATTACK_ANGLE_MAX  	 PI / 4.0f
//����&ǿռ���������&ǿ��
#define SCHEM_ATTACK_CAPTURE_MIN   PI / 4.0f
#define SCHEM_ATTACK_CAPTURE_MAX   PI / 2.0f
//����&ǿռ���������&����
#define SCHEM_DEFENCE_CAPTURE_MIN -PI / 4.0f
#define SCHEM_DEFENCE_CAPTURE_MAX -PI / 2.0f

/*ֻ����������£�������Ҫ���������
#define SCHEM_DEFENCE_MIN
#define SCHEM_DEFENCE_MAX
*/
#define DISABLE_SCAN 55 
#define ENABLE_SCAN  1

#define GATHER_LASER_COUNT 6

//������Ѫ
#define ATTACK_MIN_HP 	300
//������Ѫ
#define DEFENCE_MIN_HP 400

typedef struct
{
	fp32 ecd_set[256];
	fp32 angle_set[256];
	fp32 scan_angle_set[256];
	uint8_t mod_set[256];
	uint8_t  delay_mod[256];
	uint32_t delay_time[256];
	uint8_t  delay_flag[256];
	uint8_t laser_flag[256]; //�Ƿ�������
	uint8_t special_flag[256];
	uint8_t disable_yaw_scan[256]; //0:�ر�yawɨ�裬1������yawɨ��
	uint8_t laser_amend_priority[256]; //�������ȼ�
	uint8_t laser_disable[256];
	uint8_t behaviour_done_flag[256];
	//��ǵ�ȷ��
	fp32 laser_x1_set[256];
	fp32 laser_x2_set[256];
	fp32 laser_y1_set[256];
	fp32 laser_y2_set[256];
}path_ecd_set_t;

typedef struct
{
	uint16_t cir_time[32];
	uint8_t  cir_min[32];
	uint8_t  cir_max[32];
	uint8_t  mark_end_flag[32];
	uint8_t  restore_cir_min;
	uint8_t  restore_cir_max;
	uint8_t  restore_flag;
}cir_t;

typedef struct
{
	uint8_t valid_flag;
	fp32 x1;
	fp32 x2;
	fp32 y1;
	fp32 y2;
	/*
			1
		4	  2
			3
	1:y1;
	2:x1;
	3:y2;
	4:x2;
	*/
	uint8_t laser_amend_x_done_flag;
	uint8_t laser_amend_y_done_flag;
}laser_t;

typedef struct
{
  pid_type_def path_speed_pid;
	pid_type_def laser_speed_pid[2]; 
	path_ecd_set_t path_ecd_set;
	cir_t cir;
	laser_t laser;
	laser_t laser_last;
	fp32 ecd_now;
	fp32 yaw_offset;
	int32_t ecd_count1;
	int32_t ecd_count2;
	int32_t ecd_count3;
	int32_t ecd_count4;
	uint16_t ecd_offset1;
	uint16_t ecd_offset2;
	uint16_t ecd_offset3;
	uint16_t ecd_offset4;
	uint8_t  behaviour_count;
	uint8_t path_count;
	uint8_t path_change_flag;
	uint8_t path_pause_flag;
}path_control_t;

extern uint8_t laser_buff[ACCEPT_LASER_DATA];

extern path_control_t path_control;

/**
   * @brief ·��������ͣ
   * @param None.
   * @retval None.
   */
extern void path_pause(void);

/**
   * @brief ·�������������
   * @param None.
   * @retval None.
   */
extern void path_continue(void);
	
#endif
