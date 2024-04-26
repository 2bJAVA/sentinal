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
#include "gimbal_attitude.h"
#include "pid.h"
#include "gimbal_task.h"
#include "freertos.h"
#include "task.h"
#include "gimbal_behaviour.h"
#include "usart.h"
#include "cmsis_os.h"
#include "gimbal_behaviour.h"
#include "remote_control.h"
#include "visual_task.h"
#include "chassis_behaviour.h"
#include "fire_control_system.h"
#include "attitude_task.h"
#include "INS_task.h"
#include "chassis_behaviour.h"
#include "bsp_spining.h"

TaskHandle_t gimbal_attitude_local_handler;    //������

static void gimbal_angle_control_init(Typedef_gimbal_control * GIM);
static void gimbal_attitude_dispose(Typedef_gimbal_control * GIM);
static void gimbal_scan_mod_set(void);
	
unsigned char gimbal_attitude_start_flag=0;
unsigned char gimbal_attitude_flag=0;
unsigned char gimbal_turn_yaw_flag=0;
unsigned int gimbal_scan_counter=0;
unsigned char under_attack_feed_back_flag=0;

Typedef_SENTINEL_MOD sentinel_gimbal_behaviour = GIMBAL_NONE;  //��ʼģʽΪ��ģʽ�����������̨��ʼ��

/**
  * @brief          ��̨��̬��������
  * @param[in]      argument: NULL
  * @retval         none
  */
void gimbal_attitude_control(void const * argument)
{
	  //�������һ��ʱ��
    vTaskDelay(GIMBAL_ATTITUDE_INIT_TIME);
	  
	  gimbal_angle_control_init(&gimbal_distance);
   //get task handle, must enable 'xTaskGetHandle' in cubeMX
    //��ȡ��������������CubeMXʹ��'xtaskGetHand'
    gimbal_attitude_local_handler = xTaskGetHandle(pcTaskGetName(NULL));
    gimbal_attitude_start_flag = 1;  //��һ�о����󣬽���־λ��1����ʾ���Կ�ʼ������̬����
    while(1)
    {
        //wait for task waked up
        //�ȴ����񱻻���
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
				}
				gimbal_scan_mod_set(); //ֻ����һ��
				//�ܻ����
				//SCAN_FEED_BACK();
				//��̬����
				gimbal_attitude_dispose(&gimbal_distance);
    }
}

/**
   * @brief ��̨���ƽṹ���ʼ��
   * @param ��̨���ƽṹ��
   * @retval None.
   */
static void gimbal_angle_control_init(Typedef_gimbal_control * GIM)
{
  const static fp32 gimbal_yaw_speed_order_filter[1] = {GIMBAL_ACCEL_YAW_NUM};
	const static fp32 gimbal_pitch_speed_order_filter[1] = {GIMBAL_ACCEL_PITCH_NUM};
	
	GIM->accumulated_pitch=0;
	GIM->accumulated_yaw=0;
	GIM->set_start_yaw_angle=0;
	GIM->set_start_pitch_angle=0;
	GIM->yaw_speed=0.0f;
	GIM->pitch_speed=0.0f;

  //��һ���˲�����б����������
  first_order_filter_init(&GIM->gimbal_cmd_slow_set_yaw_speed, GIMBAL_CONTROL_TIME, gimbal_yaw_speed_order_filter);
  first_order_filter_init(&GIM->gimbal_cmd_slow_set_pitch_speed, GIMBAL_CONTROL_TIME, gimbal_pitch_speed_order_filter);
}

/**
   * @brief ��̨��̬�����������
   * @param ��̨���ƽṹ��
   * @retval None.
   */
static void gimbal_attitude_dispose(Typedef_gimbal_control * GIM)
{
	  float sentinel_scan_angle_1 = 0.0f;
		float sentinel_scan_angle_2 = 0.0f;
		if(correlate_data() == FOUND_ENEMY) //������ֵ���
		{
			sentinel_gimbal_behaviour = GIMBAL_TRACK_ENEMY_MOD; //��������ģʽ
		}
		else if((correlate_data() == NOT_FOUND) && (sentinel_gimbal_behaviour == GIMBAL_TRACK_ENEMY_MOD)) //������ģʽ�л���ɨ��ģʽ
		{
				sentinel_gimbal_behaviour = GIMBAL_SCAN_MOD;
		}
		if(sentinel_gimbal_behaviour == GIMBAL_SCAN_MOD) //ɨ��ģʽ
		{
        GIM->yaw_speed = SENTINEL_GIMBAL_YAW_NORMAL_SPEED;//SENTINEL_GIMBAL_YAW_NORMAL_SPEED
			  //yaw_speed���б�־λgimbal_turn_yaw_flag����PITCH��ʹ�ñ�־λ��ֻ�ڳ�ʼ��pitch_speedΪ0ʱ���и�ֵ
				if(GIM->pitch_speed == 0.0f)
				{
					GIM->pitch_speed = SENTINEL_GIMBAL_PITCH_NORMAL_SPEED;
		    }
			  //������Ը�Ϊ�Ӿ��趨ɨ�跶Χ�Լ�ɨ������
				GIM->accumulated_yaw = STATE_YAW;
				GIM->set_start_yaw_angle = START_YAW;
        sentinel_scan_angle_1 = GIM->set_start_yaw_angle + GIM->accumulated_yaw;
				sentinel_scan_angle_2 = GIM->set_start_yaw_angle - GIM->accumulated_yaw;
			  //���Ƕ�������-PI��PI֮��,angle_1,angle_2:�߽��
//			  if(sentinel_scan_angle_1 > BSP_PI)
//				{
//					sentinel_scan_angle_1 = -2*BSP_PI + sentinel_scan_angle_1;
//				}
//				else if(sentinel_scan_angle_1 < -BSP_PI)
//				{
//					sentinel_scan_angle_1 = 2*BSP_PI + sentinel_scan_angle_1;
//				}				
//				if(sentinel_scan_angle_2 > BSP_PI)
//				{
//					sentinel_scan_angle_2 = -2*BSP_PI + sentinel_scan_angle_2;
//				}
//				else if(sentinel_scan_angle_2 < -BSP_PI)
//				{
//					sentinel_scan_angle_2 = 2*BSP_PI + sentinel_scan_angle_2;
//				}
			//���ĵ�ȷ���������ã�����ȥ��������û��Ҫ~
			if(((gimbal_control.gimbal_yaw_motor.absolute_angle - GIM->set_start_yaw_angle <= GIMBAL_YAW_RANGE) && (gimbal_control.gimbal_yaw_motor.absolute_angle-GIM->set_start_yaw_angle >= (-GIMBAL_YAW_RANGE))) && (gimbal_turn_yaw_flag == 0) )
			{
				gimbal_turn_yaw_flag++;
			}
			//��ʽ��״̬��
			if(((gimbal_control.gimbal_yaw_motor.absolute_angle - sentinel_scan_angle_1 <= GIMBAL_YAW_RANGE) && (gimbal_control.gimbal_yaw_motor.absolute_angle - sentinel_scan_angle_1 >= (-GIMBAL_YAW_RANGE))) && (gimbal_turn_yaw_flag == 1))
			{
				GIM->yaw_speed=-SENTINEL_GIMBAL_YAW_NORMAL_SPEED;     //��ת
				gimbal_turn_yaw_flag++;
			}
			if(gimbal_turn_yaw_flag == 2)
			{
				GIM->yaw_speed=-SENTINEL_GIMBAL_YAW_NORMAL_SPEED;     //��ת
			}
			if(((gimbal_control.gimbal_yaw_motor.absolute_angle - sentinel_scan_angle_2 <= GIMBAL_YAW_RANGE) && (gimbal_control.gimbal_yaw_motor.absolute_angle - sentinel_scan_angle_2 >= (-GIMBAL_YAW_RANGE))) && (gimbal_turn_yaw_flag == 2))
			{
				GIM->yaw_speed=SENTINEL_GIMBAL_YAW_NORMAL_SPEED;     //��ת����Ȼһ��ʼ�͸�ֵ�ˣ�
        gimbal_turn_yaw_flag=0;    //������ԵĲ���,Ҳ��״̬��
			}
			//Ҳ��ȷ���������ĵ�SENTINEL_PITCH_MAX_ANGLE�Լ�SENTINEL_PITCH_MIN_ANGLE���������pitch_speed < 0�������MAX_ANGLE�����ƶ��������ٴ���MAX_ANGLE�󣬾Ϳ��Խ��з���
//			if(gimbal_control.gimbal_pitch_motor.absolute_angle <= SENTINEL_PITCH_MAX_ANGLE && GIM->pitch_speed < 0)
//			{
//				GIM->pitch_speed = -GIM->pitch_speed;
//			}
//			else if(gimbal_control.gimbal_pitch_motor.absolute_angle >= SENTINEL_PITCH_MIN_ANGLE && GIM->pitch_speed > 0)
//			{
//				GIM->pitch_speed = -GIM->pitch_speed;
//			}
	  }

		else if(sentinel_gimbal_behaviour == GIMBAL_TRACK_ENEMY_MOD) //����ģʽ
		{
				GIM->yaw_speed = 0.0f;			
			  GIM->pitch_speed = 0.0f;
		} 
//sentinel_gimbal_behaviour = GIMBAL_TRACK_ENEMY_MOD;
}


/**
   * @brief ɨ��ģʽ����
   * @param None.
   * @retval None.
   */
static void gimbal_scan_mod_set()
{
	if(gimbal_scan_counter <= GIMBAL_INIT_WAIT_TIME)
	{
		gimbal_scan_counter++;
	}
	else if(gimbal_scan_counter == (GIMBAL_INIT_WAIT_TIME+1))
	{
		sentinel_gimbal_behaviour = GIMBAL_SCAN_MOD;
		gimbal_scan_counter++;
	}
}

/**
   * @brief �����Ƿ��ֵ���
   * @param  None.
   * @retval ���ֵ���->FOUND_ENEMY,δ���ֵ���->NOT_FOUND,�Ӿ��쳣->CV_EV_ERROR
   */
unsigned char correlate_data()
{
	if((CV_EV.state == 0x01) || (CV_EV.state == 0x02) || (CV_EV.state == 0x03))
  {
		return FOUND_ENEMY;
	}
	else 
	{
		return NOT_FOUND;
	}
}

/**
   * @brief �ܻ�����
   * @param None.
   * @retval None.
   */
unsigned char SCAN_FEED_BACK()
{
	if((lose_up_flag == 1) && (correlate_data() == NOT_FOUND)) //��Ѫ��δ���ֵ���
	{
		under_attack_feed_back_flag = 1; //�����ܻ�ɨ��
		lose_up_flag = 0;
	}
	else if((under_attack_feed_back_flag == 1) && (correlate_data() == FOUND_ENEMY)) //��Ѫ�����ֵ���
	{
		under_attack_feed_back_flag = 0; //��ձ�־λ
	}
	return under_attack_feed_back_flag;
}
