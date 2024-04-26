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

TaskHandle_t gimbal_attitude_local_handler;    //任务句柄

static void gimbal_angle_control_init(Typedef_gimbal_control * GIM);
static void gimbal_attitude_dispose(Typedef_gimbal_control * GIM);
static void gimbal_scan_mod_set(void);
	
unsigned char gimbal_attitude_start_flag=0;
unsigned char gimbal_attitude_flag=0;
unsigned char gimbal_turn_yaw_flag=0;
unsigned int gimbal_scan_counter=0;
unsigned char under_attack_feed_back_flag=0;

Typedef_SENTINEL_MOD sentinel_gimbal_behaviour = GIMBAL_NONE;  //初始模式为空模式，方便进行云台初始化

/**
  * @brief          云台姿态控制任务
  * @param[in]      argument: NULL
  * @retval         none
  */
void gimbal_attitude_control(void const * argument)
{
	  //任务空闲一段时间
    vTaskDelay(GIMBAL_ATTITUDE_INIT_TIME);
	  
	  gimbal_angle_control_init(&gimbal_distance);
   //get task handle, must enable 'xTaskGetHandle' in cubeMX
    //获取任务句柄，必须在CubeMX使能'xtaskGetHand'
    gimbal_attitude_local_handler = xTaskGetHandle(pcTaskGetName(NULL));
    gimbal_attitude_start_flag = 1;  //当一切就绪后，将标志位置1，表示可以开始进行姿态控制
    while(1)
    {
        //wait for task waked up
        //等待任务被唤醒
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
				}
				gimbal_scan_mod_set(); //只进行一次
				//受击检测
				//SCAN_FEED_BACK();
				//姿态解算
				gimbal_attitude_dispose(&gimbal_distance);
    }
}

/**
   * @brief 云台控制结构体初始化
   * @param 云台控制结构体
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

  //用一阶滤波代替斜波函数生成
  first_order_filter_init(&GIM->gimbal_cmd_slow_set_yaw_speed, GIMBAL_CONTROL_TIME, gimbal_yaw_speed_order_filter);
  first_order_filter_init(&GIM->gimbal_cmd_slow_set_pitch_speed, GIMBAL_CONTROL_TIME, gimbal_pitch_speed_order_filter);
}

/**
   * @brief 云台姿态结算控制任务
   * @param 云台控制结构体
   * @retval None.
   */
static void gimbal_attitude_dispose(Typedef_gimbal_control * GIM)
{
	  float sentinel_scan_angle_1 = 0.0f;
		float sentinel_scan_angle_2 = 0.0f;
		if(correlate_data() == FOUND_ENEMY) //如果发现敌人
		{
			sentinel_gimbal_behaviour = GIMBAL_TRACK_ENEMY_MOD; //进入锁敌模式
		}
		else if((correlate_data() == NOT_FOUND) && (sentinel_gimbal_behaviour == GIMBAL_TRACK_ENEMY_MOD)) //从锁敌模式切换到扫描模式
		{
				sentinel_gimbal_behaviour = GIMBAL_SCAN_MOD;
		}
		if(sentinel_gimbal_behaviour == GIMBAL_SCAN_MOD) //扫描模式
		{
        GIM->yaw_speed = SENTINEL_GIMBAL_YAW_NORMAL_SPEED;//SENTINEL_GIMBAL_YAW_NORMAL_SPEED
			  //yaw_speed是有标志位gimbal_turn_yaw_flag，而PITCH不使用标志位，只在初始化pitch_speed为0时进行赋值
				if(GIM->pitch_speed == 0.0f)
				{
					GIM->pitch_speed = SENTINEL_GIMBAL_PITCH_NORMAL_SPEED;
		    }
			  //这里可以改为视觉设定扫描范围以及扫描中心
				GIM->accumulated_yaw = STATE_YAW;
				GIM->set_start_yaw_angle = START_YAW;
        sentinel_scan_angle_1 = GIM->set_start_yaw_angle + GIM->accumulated_yaw;
				sentinel_scan_angle_2 = GIM->set_start_yaw_angle - GIM->accumulated_yaw;
			  //将角度限制在-PI到PI之间,angle_1,angle_2:边界点
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
			//中心点确立：调试用，可以去掉，不过没必要~
			if(((gimbal_control.gimbal_yaw_motor.absolute_angle - GIM->set_start_yaw_angle <= GIMBAL_YAW_RANGE) && (gimbal_control.gimbal_yaw_motor.absolute_angle-GIM->set_start_yaw_angle >= (-GIMBAL_YAW_RANGE))) && (gimbal_turn_yaw_flag == 0) )
			{
				gimbal_turn_yaw_flag++;
			}
			//正式的状态机
			if(((gimbal_control.gimbal_yaw_motor.absolute_angle - sentinel_scan_angle_1 <= GIMBAL_YAW_RANGE) && (gimbal_control.gimbal_yaw_motor.absolute_angle - sentinel_scan_angle_1 >= (-GIMBAL_YAW_RANGE))) && (gimbal_turn_yaw_flag == 1))
			{
				GIM->yaw_speed=-SENTINEL_GIMBAL_YAW_NORMAL_SPEED;     //反转
				gimbal_turn_yaw_flag++;
			}
			if(gimbal_turn_yaw_flag == 2)
			{
				GIM->yaw_speed=-SENTINEL_GIMBAL_YAW_NORMAL_SPEED;     //反转
			}
			if(((gimbal_control.gimbal_yaw_motor.absolute_angle - sentinel_scan_angle_2 <= GIMBAL_YAW_RANGE) && (gimbal_control.gimbal_yaw_motor.absolute_angle - sentinel_scan_angle_2 >= (-GIMBAL_YAW_RANGE))) && (gimbal_turn_yaw_flag == 2))
			{
				GIM->yaw_speed=SENTINEL_GIMBAL_YAW_NORMAL_SPEED;     //正转（虽然一开始就赋值了）
        gimbal_turn_yaw_flag=0;    //方便调试的参数,也是状态机
			}
			//也是确立两个中心点SENTINEL_PITCH_MAX_ANGLE以及SENTINEL_PITCH_MIN_ANGLE，这里假设pitch_speed < 0。电机向MAX_ANGLE方向移动，这样再大于MAX_ANGLE后，就可以进行反向
//			if(gimbal_control.gimbal_pitch_motor.absolute_angle <= SENTINEL_PITCH_MAX_ANGLE && GIM->pitch_speed < 0)
//			{
//				GIM->pitch_speed = -GIM->pitch_speed;
//			}
//			else if(gimbal_control.gimbal_pitch_motor.absolute_angle >= SENTINEL_PITCH_MIN_ANGLE && GIM->pitch_speed > 0)
//			{
//				GIM->pitch_speed = -GIM->pitch_speed;
//			}
	  }

		else if(sentinel_gimbal_behaviour == GIMBAL_TRACK_ENEMY_MOD) //锁敌模式
		{
				GIM->yaw_speed = 0.0f;			
			  GIM->pitch_speed = 0.0f;
		} 
//sentinel_gimbal_behaviour = GIMBAL_TRACK_ENEMY_MOD;
}


/**
   * @brief 扫描模式设置
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
   * @brief 检验是否发现敌人
   * @param  None.
   * @retval 发现敌人->FOUND_ENEMY,未发现敌人->NOT_FOUND,视觉异常->CV_EV_ERROR
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
   * @brief 受击反馈
   * @param None.
   * @retval None.
   */
unsigned char SCAN_FEED_BACK()
{
	if((lose_up_flag == 1) && (correlate_data() == NOT_FOUND)) //损血但未发现敌人
	{
		under_attack_feed_back_flag = 1; //开启受击扫描
		lose_up_flag = 0;
	}
	else if((under_attack_feed_back_flag == 1) && (correlate_data() == FOUND_ENEMY)) //损血但发现敌人
	{
		under_attack_feed_back_flag = 0; //清空标志位
	}
	return under_attack_feed_back_flag;
}
