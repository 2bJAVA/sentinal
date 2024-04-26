/**
  ****************************(C) COPYRIGHT 2023 ULTRA****************************
  * @file       path.c/h
  * @brief      path control task,
  *             路径控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-11-28      天衍              1. done
	*  V2.0.0     2024-3-4        天衍              2.融合进激光
  *
  @verbatim
  ==============================================================================
	使用说明，见在path_init中加入设置的路径，设置的模式，调试好位置环PID后即可使用，
	path_init中有初始路径设置例子：

	path_not_event_set(path_control_init,0,50000.0f,0.0f,LINE,5000);
	path_not_event_set(path_control_init,1,-50000.0f,0.0f,LINE,5000);
	
	//循环次数为1
	path_cir_set(path_control_init,0,0,1,1,0);
	
	path_not_event_set(path_control_init,100,20000.0f,0.0f,AROUND,5000);
	path_not_event_set(path_control_init,101,-20000.0f,0.0f,AROUND,5000);
	
	//路径编写
	path_cir_set(path_control_init,1,100,101,UNLIMIT_TIME,0);
	
	代码现象：会先前进50000.0f编码器单位，然后会后退50000.0f的编码器单位，最后会
	进入先左平移再右平移20000.0f编码器单位的循环
  ==============================================================================
  @endverbatim
	注意事项：使用编码器时会默认失能视觉mid360雷达，当不使用时，注释掉其freertos任务
	即可
  ****************************(C) COPYRIGHT 2023 ULTRA****************************
  */
	
#include "path_task.h"
#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "gimbal_attitude.h"
#include "fire_control_system.h"
#include "usart.h"
#include "user_lib.h"
#include "visual_task.h"

uint32_t path_counter = 0;
//uint32_t path_scan_counter = 0;
uint32_t divide_count = 0;
uint8_t chassis_buff[CHASSIS_SEND_DATA];
uint8_t laser_buff[ACCEPT_LASER_DATA];
uint32_t game_counter = 0; //比赛计时时间
uint8_t schem_choose = 0;

path_control_t path_control;

chassis_behaviour_e last_chassis_behaviour_mode = CHASSIS_ZERO_FORCE;

static void path_task_init(path_control_t * path_control_init);

static void path_mode_change_control_transit(path_control_t * path_transit);

static void path_feedback_update(path_control_t * path_coordinates);

static void path_control_clear_data(path_control_t * path_clear);

static void path_control_loop(path_control_t * path_control_loop);

static fp32 path_abs(fp32 input);

static void path_task_state_control(path_control_t * path_state);

static void PATH_EVENT1(path_control_t * path_event1);

static void laser_locate(path_control_t * laser_locate);

static void path_not_event_set(path_control_t * not_event_set, uint16_t behaviour_num ,fp32 ecd_set, fp32 angle_set, uint8_t mod_set ,uint32_t delay_time,uint8_t disable_yaw_scan,	fp32 scan_angle_set);
	
static void path_event_set(path_control_t * event_set, uint16_t behaviour_num ,fp32 ecd_set, fp32 angle_set, uint8_t mod_set, uint8_t laser_flag,uint8_t amend_priority, uint8_t disable, fp32 x1_set, fp32 x2_set, fp32 y1_set, fp32 y2_set);

static void path_cir_set(path_control_t * path_cir, uint8_t cir_num, uint8_t cir_min, uint8_t cir_max, uint16_t cir_time, uint8_t end_flag);

static void path_special_set(path_control_t * path_special, uint16_t behaviour_num, uint32_t delay_time,fp32 scan_angle_set);

static void restore_cir_set(path_control_t * path_restore, uint8_t cir_min, uint8_t cir_max);

static void restore_detect(path_control_t * restore_detect);

static void send_laser_data(void);

uint8_t crc8(uint8_t *data, int size);

static void FloatToByte(float floatNum, uint8_t *byteArry);
	
static float Hex_To_Decimal(uint8_t *Byte, int num);

/**
  * @brief         路径任务，间隔 PATH_TASK_WAIT_TIME 1ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void path_task(void const * argument)
{
	vTaskDelay(PATH_TASK_INIT_TIME);
	
	//路径任务初始化
	path_task_init(&path_control);

  while (1)
  {
		//模式切换检测
		path_mode_change_control_transit(&path_control);
	
		//数据更新
		path_feedback_update(&path_control);
			
		//路径任务控制
		path_control_loop(&path_control);
		
		//路径任务状态控制
		path_task_state_control(&path_control);
		
		//任务延时，防止任务卡死
    vTaskDelay(PATH_TASK_WAIT_TIME);
  }
}

fp32 delta_angle;
/**
   * @brief 路径任务初始化
   * @param 路径任务结构体
   * @retval None.
   */
static void path_task_init(path_control_t * path_control_init)
{
	static const fp32 path_speed_pid[3] = {PATH_SPEED_PID_KP,PATH_SPEED_PID_KI, PATH_SPEED_PID_KD};
	
	static const fp32 laser_speed_pid[3] = {LASER_SPEED_PID_KP,LASER_SPEED_PID_KI, LASER_SPEED_PID_KD};
	
	//PID初始化
	PID_init(&path_control_init->path_speed_pid, PID_POSITION, path_speed_pid, PATH_SPEED_PID_MAX_OUT, PATH_SPEED_PID_MAX_IOUT);

	PID_init(&path_control_init->laser_speed_pid[0], PID_POSITION, laser_speed_pid, LASER_SPEED_PID_MAX_OUT, LASER_SPEED_PID_MAX_IOUT);

	PID_init(&path_control_init->laser_speed_pid[1], PID_POSITION, laser_speed_pid, LASER_SPEED_PID_MAX_OUT, LASER_SPEED_PID_MAX_IOUT);

	//清空回血/补血标志位
	path_control_init->cir.restore_flag = 0;

	//清除yaw_offset
	path_control_init->yaw_offset = 0.0f;
	
	//比赛开启begin
	while(sentinel_game_state != START_GAME)
	{
		vTaskDelay(1);
	}
	//end
	
	//编码器数据清除
	path_control_clear_data(path_control_init);
	
	//路径设置begin 
	//使用path_not_event_set()，以及path_event_set()两个函数，快速编写

  //跑5000编码器单位跑至墙角周围
//	path_not_event_set(path_control_init,101,0.1f,0.0f,AROUND,200,DISABLE_SCAN);
	
	//激光编写，我也不知道跑到这里的时候在哪，随意编写了一个x1 = 1.0f，x2 = 1.0f，y1 = 1.0f，y2 = 1.0f，可以根据实际改这个demo
//	path_event_set(path_control_init,102,0.1f,0.0f,LINE,LASER_ENABLE,AMEND_Y_PRIORITY,DISABLE_X2_Y2_LASER,1.0f,1.0f,1.0f,1.0f);
	
	//路径编写
//	path_cir_set(path_control_init,0,101,102,1,0);

	
	//方案选择：前面都为例子
	delta_angle = rad_format(chassis_move.chassis_yaw - chassis_move.chassis_INS_angle[0]);
	//判断下降沿

	//在判断中编写路径
	//底盘角度 - 云台角度 < 设定角度：方案一：attack
	if(delta_angle < SCHEM_ATTACK_ANGLE_MAX && delta_angle > SCHEM_ATTACK_ANGLE_MIN)
	{
		schem_choose = 0;
		
		//先45度前压中场，再前开
		path_not_event_set(path_control_init,0,15000.0f,PI / 4.0f,LINE,500,DISABLE_SCAN,0.0f);
		path_not_event_set(path_control_init,1,15000.0f,0.0f,LINE,500,DISABLE_SCAN,0.0f);
		
		//使用x1,Y1,Y2小陀螺前压
		path_event_set(path_control_init,2,0.0001f,0.0f,LINE,LASER_ENABLE,AMEND_X_PRIORITY,DISABLE_X2_LASER,1.0f,1.0f,1.0f,1.0f);
		
		//损血300使用激光撤退！
		path_event_set(path_control_init,3,0.0001f,0.0f,LINE,LASER_ENABLE,AMEND_X_PRIORITY,DISABLE_X2_LASER,1.0f,1.0f,1.0f,1.0f);
		//退回去
		path_not_event_set(path_control_init,4,-15000.0f,PI / 4.0f,LINE,500,DISABLE_SCAN,0.0f);
		//回退防御点
		path_not_event_set(path_control_init,5,5000.0f,0.0f,200,AROUND,DISABLE_SCAN,0.0f);
		path_event_set(path_control_init,6,0.1f,0.0f,LINE,LASER_ENABLE,AMEND_Y_PRIORITY,DISABLE_X1_LASER,1.0f,1.0f,1.0f,1.0f);
		
		//路径汇总
		path_cir_set(path_control_init,0,6,1,1,1);
		
		//在此处编写回血路径,停留5s，也许能回满也说不定，此处，无论是方向/朝向，我都没调，需要看你调
		path_not_event_set(path_control_init,100,0.1f,0.0f,LINE,500,DISABLE_SCAN,0.0f);
		path_special_set(path_control_init,101,5000,0.0f);
		path_not_event_set(path_control_init,102,0.1f,0.0f,LINE,100,DISABLE_SCAN,0.0f);
		restore_cir_set(path_control_init,100,102);
	}
	//强攻中心增益后强攻
	else if(delta_angle < SCHEM_ATTACK_CAPTURE_MAX && delta_angle > SCHEM_ATTACK_CAPTURE_MIN)
	{
		schem_choose = 2;
		//先45度前压入场，再小陀螺架射地方
		path_not_event_set(path_control_init,0,15000.0f,PI / 4.0f,LINE,500,DISABLE_SCAN,0.0f);
		path_event_set(path_control_init,1,0.0001f,0.0f,LINE,LASER_ENABLE,AMEND_X_PRIORITY,DISABLE_X2_LASER,1.0f,1.0f,1.0f,1.0f);
		
		//若一分钟了，小陀螺强攻增益！先前进，再左移
		path_event_set(path_control_init,2,0.0001f,0.0f,LINE,LASER_ENABLE,AMEND_X_PRIORITY,DISABLE_X2_LASER,1.0f,1.0f,1.0f,1.0f);
		path_event_set(path_control_init,3,0.0001f,0.0f,LINE,LASER_ENABLE,AMEND_X_PRIORITY,USE_ALL_LASER,1.0f,1.0f,1.0f,1.0f);
		
		//占领完毕，小陀螺右移 + 强攻
		path_event_set(path_control_init,4,0.0001f,0.0f,LINE,LASER_ENABLE,AMEND_X_PRIORITY,DISABLE_X2_LASER,1.0f,1.0f,1.0f,1.0f);
		
		//损血300使用激光撤退！
		path_event_set(path_control_init,5,0.0001f,0.0f,LINE,LASER_ENABLE,AMEND_X_PRIORITY,DISABLE_X2_LASER,1.0f,1.0f,1.0f,1.0f);
		//退回去
		path_not_event_set(path_control_init,6,-15000.0f,PI / 4.0f,LINE,500,DISABLE_SCAN,0.0f);
		//回退防御点
		path_not_event_set(path_control_init,7,5000.0f,0.0f,200,AROUND,DISABLE_SCAN,0.0f);
		path_event_set(path_control_init,8,0.1f,0.0f,LINE,LASER_ENABLE,AMEND_Y_PRIORITY,DISABLE_X1_LASER,1.0f,1.0f,1.0f,1.0f);

		//路径汇总
		path_cir_set(path_control_init,0,8,1,1,1);
		
		//在此处编写回血路径,停留5s，也许能回满也说不定，此处，无论是方向/朝向，我都没调，需要看你调
		path_not_event_set(path_control_init,100,0.1f,0.0f,LINE,500,DISABLE_SCAN,0.0f);
		path_special_set(path_control_init,101,5000,0.0f);
		path_not_event_set(path_control_init,102,0.1f,0.0f,LINE,100,DISABLE_SCAN,0.0f);
		restore_cir_set(path_control_init,100,102);
		
	}
	//强占中心增益后退守
	else if(delta_angle < SCHEM_DEFENCE_CAPTURE_MAX && delta_angle > SCHEM_DEFENCE_CAPTURE_MIN)
	{
		schem_choose = 3;
		//先45度前压入场，再小陀螺架射地方
		path_not_event_set(path_control_init,0,15000.0f,PI / 4.0f,LINE,500,DISABLE_SCAN,0.0f);
		path_event_set(path_control_init,1,0.0001f,0.0f,LINE,LASER_ENABLE,AMEND_X_PRIORITY,DISABLE_X2_LASER,1.0f,1.0f,1.0f,1.0f);
		
		//若一分钟了，小陀螺强攻增益！先前进，再左移
		path_event_set(path_control_init,2,0.0001f,0.0f,LINE,LASER_ENABLE,AMEND_X_PRIORITY,DISABLE_X2_LASER,1.0f,1.0f,1.0f,1.0f);
		path_event_set(path_control_init,3,0.0001f,0.0f,LINE,LASER_ENABLE,AMEND_X_PRIORITY,USE_ALL_LASER,1.0f,1.0f,1.0f,1.0f);
		
		//占领完毕，小陀螺回退架设点
		path_event_set(path_control_init,4,0.0001f,0.0f,LINE,LASER_ENABLE,AMEND_X_PRIORITY,DISABLE_X2_LASER,1.0f,1.0f,1.0f,1.0f);
		
		//损血超过200退回去
		path_not_event_set(path_control_init,5,-15000.0f,PI / 4.0f,LINE,500,DISABLE_SCAN,0.0f);
		//回退防御点
		path_not_event_set(path_control_init,6,5000.0f,0.0f,200,AROUND,DISABLE_SCAN,0.0f);
		path_event_set(path_control_init,7,0.1f,0.0f,LINE,LASER_ENABLE,AMEND_Y_PRIORITY,DISABLE_X1_LASER,1.0f,1.0f,1.0f,1.0f);

		//路径汇总
		path_cir_set(path_control_init,0,7,1,1,1);
		
		//在此处编写回血路径,停留5s，也许能回满也说不定，此处，无论是方向/朝向，我都没调，需要看你调
		path_not_event_set(path_control_init,100,0.1f,0.0f,LINE,500,DISABLE_SCAN,0.0f);
		path_special_set(path_control_init,101,5000,0.0f);
		path_not_event_set(path_control_init,102,0.1f,0.0f,LINE,100,DISABLE_SCAN,0.0f);
		restore_cir_set(path_control_init,100,102);
	}
	//否则：defence
	else
	{
		schem_choose = 1;
		
		//在防御点启动(函数的使用看注释)
		path_not_event_set(path_control_init,0,5000.0f,0.0f,AROUND,200,DISABLE_SCAN,0.0f);
		path_event_set(path_control_init,1,0.1f,0.0f,LINE,LASER_ENABLE,AMEND_Y_PRIORITY,DISABLE_X1_LASER,1.0f,1.0f,1.0f,1.0f);
		path_cir_set(path_control_init,0,0,1,1,1);
	
		//在此处编写回血路径,停留5s，也许能回满也说不定，此处，无论是方向/朝向，我都没调，需要看你调
		path_not_event_set(path_control_init,100,0.1f,0.0f,LINE,500,DISABLE_SCAN,0.0f);
		path_special_set(path_control_init,101,5000,0.0f);
		path_not_event_set(path_control_init,102,0.1f,0.0f,LINE,100,DISABLE_SCAN,0.0f);
		restore_cir_set(path_control_init,100,102);
	}
	
	
}

/**
   * @brief 遥控器模式变更，数据更新
   * @param 路径任务结构体
   * @retval None.
   */
static void path_mode_change_control_transit(path_control_t * path_transit)
{
	if(path_transit->path_pause_flag == 1 || sentinel_game_state == NOT_START_GAME)
	{
		return;
	}
	
	if(chassis_behaviour_mode == CHASSIS_AUTO_SENTINEL && last_chassis_behaviour_mode != CHASSIS_AUTO_SENTINEL)
	{
		path_control_clear_data(path_transit);
	}
	
  last_chassis_behaviour_mode = chassis_behaviour_mode;
}

/**
   * @brief 获取编码器值
   * @param 路径任务结构体
   * @retval None.
   */
static void path_feedback_update(path_control_t * path_coordinates)
{
	if((path_coordinates->path_pause_flag == 1 || chassis_behaviour_mode != CHASSIS_AUTO_SENTINEL) || (sentinel_game_state == NOT_START_GAME))
	{
		return;
	}
		// 电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
  if (chassis_move.motor_chassis[0].chassis_motor_measure->ecd - chassis_move.motor_chassis[0].chassis_motor_measure->last_ecd > HALF_ECD_RANGE)
  {
    path_coordinates->ecd_count1--;
  }
  else if (chassis_move.motor_chassis[0].chassis_motor_measure->ecd - chassis_move.motor_chassis[0].chassis_motor_measure->last_ecd < -HALF_ECD_RANGE)
  {
    path_coordinates->ecd_count1++;
  }
  if (chassis_move.motor_chassis[1].chassis_motor_measure->ecd - chassis_move.motor_chassis[1].chassis_motor_measure->last_ecd > HALF_ECD_RANGE)
  {
    path_coordinates->ecd_count2--;
  }
  else if (chassis_move.motor_chassis[1].chassis_motor_measure->ecd - chassis_move.motor_chassis[1].chassis_motor_measure->last_ecd < -HALF_ECD_RANGE)
  {
    path_coordinates->ecd_count2++;
  }
  if (chassis_move.motor_chassis[2].chassis_motor_measure->ecd - chassis_move.motor_chassis[2].chassis_motor_measure->last_ecd > HALF_ECD_RANGE)
  {
    path_coordinates->ecd_count3--;
  }
  else if (chassis_move.motor_chassis[2].chassis_motor_measure->ecd - chassis_move.motor_chassis[2].chassis_motor_measure->last_ecd < -HALF_ECD_RANGE)
  {
    path_coordinates->ecd_count3++;
  }
  if (chassis_move.motor_chassis[3].chassis_motor_measure->ecd - chassis_move.motor_chassis[3].chassis_motor_measure->last_ecd > HALF_ECD_RANGE)
  {
    path_coordinates->ecd_count4--;
  }
  else if (chassis_move.motor_chassis[3].chassis_motor_measure->ecd - chassis_move.motor_chassis[3].chassis_motor_measure->last_ecd < -HALF_ECD_RANGE)
  {
    path_coordinates->ecd_count4++;
  }
	//若为直线模式
	if(path_coordinates->path_ecd_set.mod_set[path_coordinates->behaviour_count] == LINE)
	{
		path_coordinates->ecd_now = -((-path_coordinates->ecd_count1 + path_coordinates->ecd_count2 + path_coordinates->ecd_count3 - path_coordinates->ecd_count4) * ECD_RANGE \
																+ (-(chassis_move.motor_chassis[0].chassis_motor_measure->ecd - path_coordinates->ecd_offset1) + \
																	(chassis_move.motor_chassis[1].chassis_motor_measure->ecd - path_coordinates->ecd_offset2) + \
																	(chassis_move.motor_chassis[2].chassis_motor_measure->ecd - path_coordinates->ecd_offset3) - \
																	(chassis_move.motor_chassis[3].chassis_motor_measure->ecd - path_coordinates->ecd_offset4))) * ECD_TRANSFER;
	}
	//若为左右模式
	else if(path_coordinates->path_ecd_set.mod_set[path_coordinates->behaviour_count] == AROUND)
	{
		path_coordinates->ecd_now = -((path_coordinates->ecd_count1 + path_coordinates->ecd_count2 - path_coordinates->ecd_count3 - path_coordinates->ecd_count4) * ECD_RANGE \
																+ ((chassis_move.motor_chassis[0].chassis_motor_measure->ecd - path_coordinates->ecd_offset1) + \
																	(chassis_move.motor_chassis[1].chassis_motor_measure->ecd - path_coordinates->ecd_offset2) - \
																	(chassis_move.motor_chassis[2].chassis_motor_measure->ecd - path_coordinates->ecd_offset3) - \
																	(chassis_move.motor_chassis[3].chassis_motor_measure->ecd - path_coordinates->ecd_offset4))) * ECD_TRANSFER;		
	}
}

uint8_t laser_x_flag = 0;
/**
   * @brief 路径任务控制
   * @param 路径任务结构体
   * @retval None.
   */
static void path_control_loop(path_control_t * path_control_loop)
{
	if((path_control_loop->path_pause_flag == 1 || chassis_behaviour_mode != CHASSIS_AUTO_SENTINEL) || (sentinel_game_state == NOT_START_GAME))
	{
		return;
	}
	
	//若当前路径为末尾路径，且达到最后一个动作，且激光修正无误
	if(path_control_loop->cir.mark_end_flag[path_control_loop->path_count] && path_control_loop->laser.laser_amend_x_done_flag == 1 && path_control_loop->laser.laser_amend_y_done_flag == 1 && path_control_loop->behaviour_count == path_control_loop->cir.cir_max[path_control_loop->path_count])
	{
		//设置初始动作
		if(path_control_loop->path_change_flag == 0)
		{
			path_control_loop->behaviour_count = path_control_loop->cir.restore_cir_min;
			path_control_loop->path_change_flag = 1;
		}		
	}
	//若还有循环次数
	else if(path_control_loop->cir.cir_time[path_control_loop->path_count] > 0)
	{
		//设置初始动作
		if(path_control_loop->path_change_flag == 0)
		{
			path_control_loop->behaviour_count = path_control_loop->cir.cir_min[path_control_loop->path_count];
			path_control_loop->path_change_flag = 1;
		}
	}
	//切换至下一条路径
	 else 
	 {
		 distance.x = 0.0f;
		 distance.y = 0.0f;
		 if(path_control_loop->path_count < 31)
		 {
			 path_control_loop->path_count++;
		 }
		}
	 
		//若路径不为0
		if(path_control_loop->path_ecd_set.ecd_set[path_control_loop->behaviour_count] != 0)
		{
			static uint8_t laser_run_flag_now = 0;
			static uint8_t laser_run_flag_last = 0;
			float speed_set = 0.0f;
			
			if(path_control_loop->path_ecd_set.special_flag[path_control_loop->behaviour_count] != 1)
			{
				if(!(path_control_loop->path_ecd_set.delay_mod[path_control_loop->behaviour_count] == EVENT && path_control_loop->path_ecd_set.behaviour_done_flag[path_control_loop->behaviour_count] == 1))
				{
					laser_run_flag_now = 0;
				}
				else
				{
					laser_run_flag_now = 1;
				}
				
				laser_x_flag = laser_run_flag_now;
		
				//开启激光后，对速度数据进行清空
				if(laser_run_flag_now == 1 && laser_run_flag_last == 0)
				{
					distance.x = 0.0f;
					distance.y = 0.0f;
				}
				laser_run_flag_last = laser_run_flag_now;
				
				if(laser_run_flag_now == 0)
				{
					speed_set = PID_calc(&path_control_loop->path_speed_pid,path_control_loop->ecd_now,path_control_loop->path_ecd_set.ecd_set[path_control_loop->behaviour_count]);
					
					distance.spining_angle = rad_format(path_control_loop->path_ecd_set.angle_set[path_control_loop->behaviour_count] + path_control_loop->yaw_offset);
				}
				
				if(laser_run_flag_now == 0)
				{
					distance.spining_flag = 0;
				}
				
				//如果角度变化较大，进行延迟
				if(path_abs(rad_format(path_control_loop->path_ecd_set.angle_set[path_control_loop->behaviour_count] - chassis_move.chassis_yaw)) > ANGLE_CHANGE_CONSTANT && laser_run_flag_now == 0)
				{
					distance.x = 0.0f;
					distance.y = 0.0f;
					vTaskDelay((uint32_t)(ANGLE_CHANGE_WAIT_TIME + path_abs(ANGLE_CHANGE_WAIT_TIME_X * (path_control_loop->path_ecd_set.angle_set[path_control_loop->behaviour_count] - chassis_move.chassis_yaw))));
					path_control_loop->ecd_offset1 = chassis_move.motor_chassis[0].chassis_motor_measure->ecd;
					path_control_loop->ecd_offset2 = chassis_move.motor_chassis[1].chassis_motor_measure->ecd;
					path_control_loop->ecd_offset3 = chassis_move.motor_chassis[2].chassis_motor_measure->ecd;
					path_control_loop->ecd_offset4 = chassis_move.motor_chassis[3].chassis_motor_measure->ecd;				
				}
				if(path_control_loop->path_ecd_set.mod_set[path_control_loop->behaviour_count] == LINE && laser_run_flag_now == 0)
				{
					distance.x = speed_set;
					distance.y = 0.0f;
				}
				else if(path_control_loop->path_ecd_set.mod_set[path_control_loop->behaviour_count] == AROUND && laser_run_flag_now == 0)
				{
					distance.x = 0.0f;
					distance.y = speed_set;
				}
			}
			else
			{
				//特殊动作，开启正弦小陀螺
				distance.x = 0.0f;
				distance.y = 0.0f;
				distance.spining_flag = 1;
			}
			
			//判断是否到达位置
			if(path_abs(path_control_loop->path_ecd_set.ecd_set[path_control_loop->behaviour_count] - path_control_loop->ecd_now) < ECD_ERROR || path_control_loop->path_ecd_set.special_flag[path_control_loop->behaviour_count] == 1)
			{
				//如果选择的是时间延迟模式
				if(path_control_loop->path_ecd_set.delay_mod[path_control_loop->behaviour_count] == NOT_EVENT)
				{
					path_counter++;
					//延迟时间达到
					if(path_counter >= path_control_loop->path_ecd_set.delay_time[path_control_loop->behaviour_count])
					{
						path_counter = 0;
						path_control_loop->ecd_now = 0;
						path_control_loop->ecd_offset1 = chassis_move.motor_chassis[0].chassis_motor_measure->ecd;
						path_control_loop->ecd_offset2 = chassis_move.motor_chassis[1].chassis_motor_measure->ecd;
						path_control_loop->ecd_offset3 = chassis_move.motor_chassis[2].chassis_motor_measure->ecd;
						path_control_loop->ecd_offset4 = chassis_move.motor_chassis[3].chassis_motor_measure->ecd;
						path_control_loop->ecd_count1 = 0;
						path_control_loop->ecd_count2 = 0;
						path_control_loop->ecd_count3 = 0;
						path_control_loop->ecd_count4 = 0;
						path_control_loop->behaviour_count++;					
					}
				}
				//若为事件延迟
				else if(path_control_loop->path_ecd_set.delay_mod[path_control_loop->behaviour_count] == EVENT)
				{
					path_counter++;
					path_control_loop->path_ecd_set.behaviour_done_flag[path_control_loop->behaviour_count] = 1;
					if(path_counter >= EVENT_NORMAL_DELAY_TIME)
					{
						path_counter = EVENT_NORMAL_DELAY_TIME;
					}
					if(path_control_loop->path_ecd_set.delay_flag[path_control_loop->behaviour_count] == 1 && path_counter >= EVENT_NORMAL_DELAY_TIME)
					{
						path_counter = 0;
						path_control_loop->ecd_now = 0;
						path_control_loop->ecd_offset1 = chassis_move.motor_chassis[0].chassis_motor_measure->ecd;
						path_control_loop->ecd_offset2 = chassis_move.motor_chassis[1].chassis_motor_measure->ecd;
						path_control_loop->ecd_offset3 = chassis_move.motor_chassis[2].chassis_motor_measure->ecd;
						path_control_loop->ecd_offset4 = chassis_move.motor_chassis[3].chassis_motor_measure->ecd;
						path_control_loop->ecd_count1 = 0;
						path_control_loop->ecd_count2 = 0;
						path_control_loop->ecd_count3 = 0;
						path_control_loop->ecd_count4 = 0;				
						path_control_loop->behaviour_count++;										
					}
				}
			}
			//如果到达末尾，且为UNLIMIT_TIME
			if(path_control_loop->behaviour_count > path_control_loop->cir.cir_max[path_control_loop->path_count] && path_control_loop->cir.cir_time[path_control_loop->path_count] == UNLIMIT_TIME)
			{
					path_control_loop->path_change_flag = 0;
			}
			//如果达到末尾，且循环次数不为UNLIMIT_TIME，并且不为回复/复活路径
		  else if(path_control_loop->behaviour_count > path_control_loop->cir.cir_max[path_control_loop->path_count] && path_control_loop->cir.cir_time[path_control_loop->path_count] != UNLIMIT_TIME && path_control_loop->cir.restore_flag == 0)
			{
				//循环次数递减
				path_control_loop->path_change_flag = 0;
				path_control_loop->cir.cir_time[path_control_loop->path_count]--;
				//若没有循环次数
				if(path_control_loop->cir.cir_time[path_control_loop->path_count] <= 0)
				{
					path_control_loop->path_count++;
				}
			}
			//回复路径达到末尾，清空标志位，返回end激光修正
			else if(path_control_loop->behaviour_count > path_control_loop->cir.restore_cir_max && path_control_loop->cir.restore_flag == 1)
			{
				path_control_loop->cir.restore_flag = 0;
			}
		}
		//路径设置ECD出现0,设置错误,全部清空
		else
		{
			path_control_clear_data(path_control_loop);
		}
}

/**
   * @brief 数据清除
   * @param 路径任务结构体
   * @retval None.
   */
static void path_control_clear_data(path_control_t * path_clear)
{
	path_clear->ecd_now = 0;
	path_clear->ecd_offset1 = chassis_move.motor_chassis[0].chassis_motor_measure->ecd;
	path_clear->ecd_offset2 = chassis_move.motor_chassis[1].chassis_motor_measure->ecd;
	path_clear->ecd_offset3 = chassis_move.motor_chassis[2].chassis_motor_measure->ecd;
	path_clear->ecd_offset4 = chassis_move.motor_chassis[3].chassis_motor_measure->ecd;	
	path_clear->ecd_count1 = 0;
	path_clear->ecd_count2 = 0;
	path_clear->ecd_count3 = 0;
	path_clear->ecd_count4 = 0;
	path_clear->path_count = 0;
	path_clear->behaviour_count = 0;
	path_clear->path_change_flag = 0;
	path_clear->path_pause_flag = 0;
}

/**
   * @brief 求绝对值
   * @param 输入值
   * @retval 绝对值
   */
static fp32 path_abs(fp32 input)
{
	if(input < 0)
	{
		return (-input);
	}
	else
	{
		return input;
	}
}

/**
   * @brief 路径任务暂停
   * @param None.
   * @retval None.
   */
void path_pause()
{
	distance.x = 0;
	distance.y = 0;
	path_control.path_pause_flag = 1;
}

/**
   * @brief 路径任务继续运行
   * @param None.
   * @retval None.
   */
void path_continue()
{
	path_control.path_pause_flag = 0;
	path_control.ecd_offset1 = chassis_move.motor_chassis[0].chassis_motor_measure->ecd;
	path_control.ecd_offset2 = chassis_move.motor_chassis[1].chassis_motor_measure->ecd;
	path_control.ecd_offset3 = chassis_move.motor_chassis[2].chassis_motor_measure->ecd;
	path_control.ecd_offset4 = chassis_move.motor_chassis[3].chassis_motor_measure->ecd;	
}

/**
   * @brief 路径任务状态控制
   * @param 路径任务结构体
   * @retval None.
   */
static void path_task_state_control(path_control_t * path_state)
{
	if(chassis_behaviour_mode != CHASSIS_AUTO_SENTINEL && sentinel_game_state != START_GAME)
	{
		return;
	}
	//PATH_ENVENT1 begin
	PATH_EVENT1(path_state);
	//end
	
	/*
	//其他判断条件 begin
	//比如这里例子：发现敌人，原地自旋并开火(由于陀螺仪零飘限制，这些代码并不能使用)
	if(correlate_data() == FOUND_ENEMY)
	{
		path_control.path_pause_flag = 1;
		distance.spining_flag = 1;
	}
	else
	{
		path_control.path_pause_flag = 0;
	}
	
	//损血原地自旋，自旋时间PATH_SCAN_COUNTms
	if(SCAN_FEED_BACK() == 1)
	{
		path_scan_counter = PATH_SCAN_MAX_COUNT;
		path_control.path_pause_flag = 1;
		distance.spining_flag = 1;
	}
	else
	{
		if(path_scan_counter <= 0)
		{
			path_control.path_pause_flag = 0;
		}
		else
		{
			path_control.path_pause_flag = 1;
			distance.spining_flag = 1;			
			path_scan_counter--;
		}
	}
	*/
	
	//向激光部分发送底盘数据
	send_laser_data();
	
	//回血/复活检测
	restore_detect(path_state);
	
	//end
	//使能mid360视觉雷达(使用编码器了，肯定要失能的啦)
//	distance.control_priority = 0;
	//byd视觉屁用没有，使能编码器
		distance.control_priority = 1;
		
	game_counter++;
}
	

//编写事件延迟(RM比赛中，事件延迟只适合激光)
static void PATH_EVENT1(path_control_t * path_event1)
{
	static uint16_t capture_count = 0;
	//进攻情况
	if(schem_choose == 0 && path_event1->path_ecd_set.laser_flag[path_event1->behaviour_count] == LASER_ENABLE)
	{
	//提供一个使用激光的例子，但我并未设置参数
		//事件延迟且编码器动作完成
		if(path_event1->path_ecd_set.delay_mod[path_event1->behaviour_count] == EVENT && path_event1->path_ecd_set.behaviour_done_flag[path_event1->behaviour_count] == 1 && (path_event1->behaviour_count == 2 || path_event1->behaviour_count == 3))
		{
			//开启小陀螺以及激光定位
			laser_locate(path_event1);
			//定位完成，开启下一个动作，进攻状态，损血达到设定值
			if(path_event1->laser.laser_amend_x_done_flag == 1 && path_event1->laser.laser_amend_y_done_flag == 1 && (sentinel_SENTINEL_UP < ATTACK_MIN_HP) && path_event1->behaviour_count == 2 && path_event1->path_ecd_set.behaviour_done_flag[path_event1->behaviour_count] == 1)
			{
				path_event1->path_ecd_set.delay_flag[path_event1->behaviour_count] = 1;
			}
		}
		
		//撤退时的情况
		if(path_event1->path_ecd_set.delay_mod[path_event1->behaviour_count] == EVENT && path_event1->path_ecd_set.behaviour_done_flag[path_event1->behaviour_count] == 1 && path_event1->behaviour_count == 3)
		{
			laser_locate(path_event1);
			if(path_event1->laser.laser_amend_x_done_flag == 1 && path_event1->laser.laser_amend_y_done_flag == 1)
			{
				path_event1->path_ecd_set.delay_flag[path_event1->behaviour_count] = 1;
			}
		}
	}
	else if(schem_choose >= 2 && path_event1->path_ecd_set.laser_flag[path_event1->behaviour_count] == LASER_ENABLE && path_event1->behaviour_count == 1)
	{
		laser_locate(path_event1);
		//架设损血直接退
		if(sentinel_SENTINEL_UP < ATTACK_MIN_HP && schem_choose == 2)
		{
			path_event1->behaviour_count = 6;
		}
		else if(sentinel_SENTINEL_UP < DEFENCE_MIN_HP && schem_choose == 1)
		{
			path_event1->behaviour_count = 5;
		}
		//开始前压站点
		if(path_event1->laser.laser_amend_x_done_flag == 1 && path_event1->laser.laser_amend_y_done_flag == 1 && game_counter > 60000 && path_event1->path_ecd_set.behaviour_done_flag[path_event1->behaviour_count] == 1)
		{
			path_event1->path_ecd_set.delay_flag[path_event1->behaviour_count] = 1;
		}
	}
	else if(schem_choose >= 2 && path_event1->path_ecd_set.laser_flag[path_event1->behaviour_count] == LASER_ENABLE && (path_event1->behaviour_count == 2 || path_event1->behaviour_count == 3))
	{
		laser_locate(path_event1);
		if(path_event1->laser.laser_amend_x_done_flag == 1 && path_event1->laser.laser_amend_y_done_flag == 1 && path_event1->behaviour_count == 3 && path_event1->path_ecd_set.behaviour_done_flag[path_event1->behaviour_count] == 1)
		{
			capture_count++;
		}
		if(path_event1->laser.laser_amend_x_done_flag == 1 && path_event1->laser.laser_amend_y_done_flag == 1 && (path_event1->behaviour_count == 2 || (path_event1->behaviour_count == 3 && capture_count > 60000)) && path_event1->path_ecd_set.behaviour_done_flag[path_event1->behaviour_count] == 1)
		{
			capture_count = 0;
			path_event1->path_ecd_set.delay_flag[path_event1->behaviour_count] = 1;
		}		
	}
	else if(schem_choose >= 2 && path_event1->path_ecd_set.laser_flag[path_event1->behaviour_count] == LASER_ENABLE && path_event1->behaviour_count == 4)
	{
		laser_locate(path_event1);
		//定位完成，开启下一个动作，进攻状态，损血达到设定值
		if(path_event1->laser.laser_amend_x_done_flag == 1 && path_event1->laser.laser_amend_y_done_flag == 1 && ((schem_choose == 2 && sentinel_SENTINEL_UP < ATTACK_MIN_HP) || (schem_choose == 3 && sentinel_SENTINEL_UP < DEFENCE_MIN_HP)) && path_event1->path_ecd_set.behaviour_done_flag[path_event1->behaviour_count] == 1)
		{
			path_event1->path_ecd_set.delay_flag[path_event1->behaviour_count] = 1;
		}
	}
	else if(schem_choose == 2 && path_event1->path_ecd_set.laser_flag[path_event1->behaviour_count] == LASER_ENABLE && path_event1->behaviour_count == 5)
	{
		laser_locate(path_event1);
		//定位完成，开启下一个动作，进攻状态，损血达到设定值
		if(path_event1->laser.laser_amend_x_done_flag == 1 && path_event1->laser.laser_amend_y_done_flag == 1 && path_event1->path_ecd_set.behaviour_done_flag[path_event1->behaviour_count] == 1)
		{
			path_event1->path_ecd_set.delay_flag[path_event1->behaviour_count] = 1;
		}
	}
		//防御情况
	else if(path_event1->path_ecd_set.laser_flag[path_event1->behaviour_count] == LASER_ENABLE)
	{
		laser_locate(path_event1);
	}
}

fp32 debug_buff[10];

/**
   * @brief 激光定位，附在事件延迟之后
   * @param None.
   * @retval None.
   */
static void laser_locate(path_control_t * laser_locate)
{
	//判断模式，防止勿使用，事件延迟且开启激光
	if(laser_locate->path_ecd_set.delay_mod[laser_locate->behaviour_count] == EVENT && laser_locate->path_ecd_set.laser_flag[laser_locate->behaviour_count] == LASER_ENABLE)
	{
		//开启小陀螺
		distance.spining_flag = 1;
		distance.spining_angle = 0.0f;
		
		//如果收到了来自从机的激光信号
		if(laser_locate->laser.valid_flag == 1)
		{
			//障碍物标志位
			uint8_t barrier_flag_x = 0;
			uint8_t barrier_flag_y = 0;
			fp32 laser_x_error,laser_y_error = 0.0f;
			
			laser_locate->laser.valid_flag = 0;
			
			if(path_abs(laser_locate->path_ecd_set.laser_x1_set[laser_locate->behaviour_count] + laser_locate->path_ecd_set.laser_x2_set[laser_locate->behaviour_count] - MACHINE_X - laser_locate->laser.x1 - laser_locate->laser.x2) > LOCATE_BARRIER_ADMIT)
			{
				barrier_flag_x = 1;
			}
			
			if(path_abs(laser_locate->path_ecd_set.laser_y1_set[laser_locate->behaviour_count] + laser_locate->path_ecd_set.laser_y1_set[laser_locate->behaviour_count] - MACHINE_Y - laser_locate->laser.y1 - laser_locate->laser.y1) > LOCATE_BARRIER_ADMIT)
			{
				barrier_flag_y = 1;
			}
			
			//若有X激光失能
			if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X1_LASER || laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X2_LASER || (laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] >= DISABLE_X1_Y1_LASER))
			{
				barrier_flag_x = 0;
			}

			//若有Y激光失能
			if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_Y1_LASER || laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_Y2_LASER || (laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] >= DISABLE_X1_Y1_LASER))
			{
				barrier_flag_y = 0;
			}

			if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == USE_ALL_LASER)
			{
				//测算x偏差
				laser_x_error = (-(laser_locate->laser.x1 + MACHINE_X / 2.0f - laser_locate->path_ecd_set.laser_x1_set[laser_locate->behaviour_count]) + (laser_locate->laser.x2 + MACHINE_X / 2.0f - laser_locate->path_ecd_set.laser_x2_set[laser_locate->behaviour_count])) / 2.0f;					
				//测算y偏差
				laser_y_error = ((laser_locate->laser.y1 + MACHINE_Y / 2.0f - laser_locate->path_ecd_set.laser_y1_set[laser_locate->behaviour_count]) - (laser_locate->laser.y2 + MACHINE_Y / 2.0f - laser_locate->path_ecd_set.laser_y2_set[laser_locate->behaviour_count])) / 2.0f;				
			}
			else if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X1_LASER)
			{
				//测算x偏差
				laser_x_error = (laser_locate->laser.x2 + MACHINE_X / 2.0f - laser_locate->path_ecd_set.laser_x2_set[laser_locate->behaviour_count]);					
				//测算y偏差
				laser_y_error = ((laser_locate->laser.y1 + MACHINE_Y / 2.0f - laser_locate->path_ecd_set.laser_y1_set[laser_locate->behaviour_count]) - (laser_locate->laser.y2 + MACHINE_Y / 2.0f - laser_locate->path_ecd_set.laser_y2_set[laser_locate->behaviour_count])) / 2.0f;					
			}
			else if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X2_LASER)
			{
				//测算x偏差
				laser_x_error = -(laser_locate->laser.x1 + MACHINE_X / 2.0f - laser_locate->path_ecd_set.laser_x1_set[laser_locate->behaviour_count]);					
				//测算y偏差
				laser_y_error = ((laser_locate->laser.y1 + MACHINE_Y / 2.0f - laser_locate->path_ecd_set.laser_y1_set[laser_locate->behaviour_count]) - (laser_locate->laser.y2 + MACHINE_Y / 2.0f - laser_locate->path_ecd_set.laser_y2_set[laser_locate->behaviour_count])) / 2.0f;								
			}
			else if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_Y1_LASER)
			{
				//测算x偏差
				laser_x_error = (-(laser_locate->laser.x1 + MACHINE_X / 2.0f - laser_locate->path_ecd_set.laser_x1_set[laser_locate->behaviour_count]) + (laser_locate->laser.x2 + MACHINE_X / 2.0f - laser_locate->path_ecd_set.laser_x2_set[laser_locate->behaviour_count])) / 2.0f;					
				//测算y偏差
				laser_y_error =  -(laser_locate->laser.y2 + MACHINE_Y / 2.0f - laser_locate->path_ecd_set.laser_y2_set[laser_locate->behaviour_count]);								
			}
			else if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_Y2_LASER)
			{
				//测算x偏差
				laser_x_error = (-(laser_locate->laser.x1 + MACHINE_X / 2.0f - laser_locate->path_ecd_set.laser_x1_set[laser_locate->behaviour_count]) + (laser_locate->laser.x2 + MACHINE_X / 2.0f - laser_locate->path_ecd_set.laser_x2_set[laser_locate->behaviour_count])) / 2.0f;					
				//测算y偏差
				laser_y_error = (laser_locate->laser.y1 + MACHINE_Y / 2.0f - laser_locate->path_ecd_set.laser_y1_set[laser_locate->behaviour_count]);								
			}	
			else if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X1_Y1_LASER)
			{
				//测算x偏差
				laser_x_error = (laser_locate->laser.x2 + MACHINE_X / 2.0f - laser_locate->path_ecd_set.laser_x2_set[laser_locate->behaviour_count]);					
				//测算y偏差
				laser_y_error = -(laser_locate->laser.y2 + MACHINE_Y / 2.0f - laser_locate->path_ecd_set.laser_y2_set[laser_locate->behaviour_count]);							
			}
			else if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X1_Y2_LASER)
			{
				//测算x偏差
				laser_x_error = (laser_locate->laser.x2 + MACHINE_X / 2.0f - laser_locate->path_ecd_set.laser_x2_set[laser_locate->behaviour_count]);					
				//测算y偏差
				laser_y_error = (laser_locate->laser.y1 + MACHINE_Y / 2.0f - laser_locate->path_ecd_set.laser_y1_set[laser_locate->behaviour_count]);						
			}
			else if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X2_Y1_LASER)
			{
				//测算x偏差
				laser_x_error = -(laser_locate->laser.x1 + MACHINE_X / 2.0f - laser_locate->path_ecd_set.laser_x1_set[laser_locate->behaviour_count]);
				//测算y偏差
				laser_y_error =  -(laser_locate->laser.y2 + MACHINE_Y / 2.0f - laser_locate->path_ecd_set.laser_y2_set[laser_locate->behaviour_count]);							
			}
			else if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X2_Y2_LASER)
			{
				//测算x偏差
				laser_x_error = -(laser_locate->laser.x1 + MACHINE_X / 2.0f - laser_locate->path_ecd_set.laser_x1_set[laser_locate->behaviour_count]);
				//测算y偏差
				laser_y_error = (laser_locate->laser.y1 + MACHINE_Y / 2.0f - laser_locate->path_ecd_set.laser_y1_set[laser_locate->behaviour_count]);								
			}
			else
			{
				distance.x = 0.0f;
				distance.y = 0.0f;
				return;
			}
			
			//没有设定优先级，就是直接进行斜线修正
			if(laser_locate->path_ecd_set.laser_amend_priority[laser_locate->behaviour_count] == NOT_SET_PRIORITY)
			{
				//若十字扫描中有障碍物
				if(barrier_flag_x == 1 || barrier_flag_y == 1)
				{
					distance.x = 0.0f;
					distance.y = 0.0f;
					return;
				}

				//没有障碍物，开启PID计算
				distance.y = PID_calc(&laser_locate->laser_speed_pid[0],0.0f,laser_x_error);
				distance.x = PID_calc(&laser_locate->laser_speed_pid[1],0.0f,laser_y_error);
				
				//修正是否完成标志位
				if(path_abs(laser_x_error) < LASER_ERROR_X)
				{
					laser_locate->laser.laser_amend_x_done_flag = 1;
				}
				else
				{
					laser_locate->laser.laser_amend_x_done_flag = 0;
				}
				if(path_abs(laser_y_error) < LASER_ERROR_Y)
				{
					laser_locate->laser.laser_amend_y_done_flag = 1;
				}
				else
				{
					laser_locate->laser.laser_amend_y_done_flag = 0;
				}
			}
			//优先进行X修正
			else if(laser_locate->path_ecd_set.laser_amend_priority[laser_locate->behaviour_count] == AMEND_X_PRIORITY)
			{
				//若x方向有障碍物
				if(barrier_flag_x == 1)
				{
					distance.x = 0.0f;
					distance.y = 0.0f;
					return;
				}		
				
				//没有障碍物，开启PID计算
				distance.y = PID_calc(&laser_locate->laser_speed_pid[0],0.0f,laser_x_error);
				
				//修正是否完成标志位
				if(path_abs(laser_x_error) < LASER_ERROR_X)
				{
					laser_locate->laser.laser_amend_x_done_flag = 1;
				}
				else
				{
					laser_locate->laser.laser_amend_x_done_flag = 0;
				}
				
				//若x修正完成，进行y修正
				if(laser_locate->laser.laser_amend_x_done_flag == 1 && barrier_flag_y == 0)
				{
					distance.x = PID_calc(&laser_locate->laser_speed_pid[1],0.0f,laser_y_error);						
				}
				else
				{
					distance.x = 0.0f;
				}
				
				if(path_abs(laser_y_error) < LASER_ERROR_Y)
				{
					laser_locate->laser.laser_amend_y_done_flag = 1;
				}
				else
				{
					laser_locate->laser.laser_amend_y_done_flag = 0;
				}
			}
			//优先进行Y修正
			else if(laser_locate->path_ecd_set.laser_amend_priority[laser_locate->behaviour_count] == AMEND_Y_PRIORITY)
			{
				//若y方向有障碍物
				if(barrier_flag_y == 1)
				{
					distance.x = 0.0f;
					distance.y = 0.0f;
					return;
				}
				
				//没有障碍物，开启PID计算
				distance.x = PID_calc(&laser_locate->laser_speed_pid[1],0.0f,laser_y_error);
				
				//修正是否完成标志位
				if(path_abs(laser_y_error) < LASER_ERROR_Y)
				{
					laser_locate->laser.laser_amend_y_done_flag = 1;
				}
				else
				{
					laser_locate->laser.laser_amend_y_done_flag = 0;
				}
				
				//若y修正完成，进行x修正
				if(laser_locate->laser.laser_amend_y_done_flag == 1 && barrier_flag_x == 0)
				{
					distance.y = PID_calc(&laser_locate->laser_speed_pid[0],0.0f,laser_x_error);						
				}
				else
				{
					distance.y = 0.0f;
				}
				
				if(path_abs(laser_x_error) < LASER_ERROR_X)
				{
					laser_locate->laser.laser_amend_x_done_flag = 1;
				}
				else
				{
					laser_locate->laser.laser_amend_x_done_flag = 0;
				}
			}
			
			//定义一些前一时刻的变量
			static uint8_t laser_amend_x_done_last_flag = 0;
			static uint8_t laser_amend_y_done_last_flag = 0;
			static uint8_t laser_amend_start_flag = 0;
			static uint8_t laser_x_flag = 0;
			static uint8_t laser_y_flag = 0;
			static uint8_t laser_last_behaviour = 0;
			
			//定义一些数据
			static fp32 laser_x_temp_buff[2][GATHER_LASER_COUNT] = {0};
			static fp32 laser_y_temp_buff[2][GATHER_LASER_COUNT] = {0};
			static uint8_t laser_x_count = 0;
			static uint8_t laser_y_count = 0;
			static uint8_t record_done = 0;
			
			if(laser_locate->behaviour_count != laser_last_behaviour)
			{
				laser_x_count = 0;
				laser_y_count = 0;
			}
			laser_last_behaviour = laser_locate->behaviour_count;
			//完成了X激光修正
			if(laser_amend_x_done_last_flag != laser_locate->laser.laser_amend_x_done_flag && laser_locate->laser.laser_amend_x_done_flag == 1)
			{
				laser_x_flag = 1;
			}
			else if(laser_locate->laser.laser_amend_x_done_flag == 0)
			{
				laser_x_flag = 0;
			}
				debug_buff[0] = laser_amend_x_done_last_flag;
			//完成了Y激光修正
			if(laser_amend_y_done_last_flag != laser_locate->laser.laser_amend_y_done_flag && laser_locate->laser.laser_amend_y_done_flag == 1)
			{
				laser_y_flag = 1;
			}
			else if(laser_locate->laser.laser_amend_y_done_flag == 0)
			{
				laser_y_flag = 0;
			}
			
			//根据激光完成程度不同，对start标志位进行赋值
			if(laser_x_flag == 1 && laser_y_flag == 1)
			{
				laser_amend_start_flag = 3;
			}
			else if(laser_x_flag == 1)
			{
				laser_amend_start_flag = 1;
			}
			else if(laser_y_flag == 1)		
			{
				laser_amend_start_flag = 2;
			}				
			else
			{
				laser_amend_start_flag = 0;
			}
			
			laser_amend_x_done_last_flag = laser_locate->laser.laser_amend_x_done_flag;
			laser_amend_y_done_last_flag = laser_locate->laser.laser_amend_x_done_flag;
			
			//记录一下初始数值
			if((laser_amend_start_flag == 1 || laser_amend_start_flag == 3)&& laser_x_count < GATHER_LASER_COUNT - 1)
			{
				laser_x_temp_buff[0][laser_x_count] = laser_locate->laser.x1;
				laser_x_temp_buff[1][laser_x_count] = laser_locate->laser.x2;
				laser_x_count++;
			}
			if((laser_amend_start_flag == 2 || laser_amend_start_flag == 3) && laser_y_count < GATHER_LASER_COUNT - 1)
			{
				laser_y_temp_buff[0][laser_y_count] = laser_locate->laser.y1;
				laser_y_temp_buff[1][laser_y_count] = laser_locate->laser.y2;
				laser_y_count++;
			}
			
			uint8_t i;
			fp32 temp = 0.0f;
			debug_buff[1] = laser_amend_start_flag;
			debug_buff[2] = laser_x_count;
			debug_buff[3] = laser_y_count;
			
			//求记录数据的平均值，记录在laser_last结构体中
			if((laser_x_count >= GATHER_LASER_COUNT - 1 && laser_y_count >= GATHER_LASER_COUNT - 1) && record_done < 3)
			{
				record_done = 3;
				for(i = 0; i < GATHER_LASER_COUNT - 1; i++)
				{
					temp += laser_x_temp_buff[0][i];
				}
				laser_locate->laser_last.x1 = (temp / (float)GATHER_LASER_COUNT); 
				temp = 0.0f;
				for(i = 0; i < GATHER_LASER_COUNT - 1; i++)
				{
					temp += laser_x_temp_buff[1][i];
				}
				laser_locate->laser_last.x2 = (temp / (float)GATHER_LASER_COUNT);
				temp = 0.0f;
				for(i = 0; i < GATHER_LASER_COUNT - 1; i++)
				{
					temp += laser_y_temp_buff[0][i];
				}
				laser_locate->laser_last.y1 = (temp / (float)GATHER_LASER_COUNT); 
				temp = 0.0f;
				for(i = 0; i < GATHER_LASER_COUNT - 1; i++)
				{
					temp += laser_y_temp_buff[1][i];
				}
				laser_locate->laser_last.y2 = (temp / (float)GATHER_LASER_COUNT);				
			}
			else if(laser_x_count >= GATHER_LASER_COUNT - 1 && record_done == 0)
			{
				record_done = 1;
				for(i = 0; i < GATHER_LASER_COUNT - 1; i++)
				{
					temp += laser_x_temp_buff[0][i];
				}
				laser_locate->laser_last.x1 = (temp / (float)GATHER_LASER_COUNT); 
				temp = 0.0f;
				for(i = 0; i < GATHER_LASER_COUNT - 1; i++)
				{
					temp += laser_x_temp_buff[1][i];
				}
				laser_locate->laser_last.x2 = (temp / (float)GATHER_LASER_COUNT); 
			}
			else if(laser_y_count >= GATHER_LASER_COUNT - 1 && record_done == 0)
			{
				record_done = 2;
				for(i = 0; i < GATHER_LASER_COUNT - 1; i++)
				{
					temp += laser_y_temp_buff[0][i];
				}
				laser_locate->laser_last.y1 = (temp / (float)GATHER_LASER_COUNT); 
				temp = 0.0f;
				for(i = 0; i < GATHER_LASER_COUNT - 1; i++)
				{
					temp += laser_y_temp_buff[1][i];
				}
				laser_locate->laser_last.y2 = (temp / (float)GATHER_LASER_COUNT);			
			}
			else if(laser_y_count < GATHER_LASER_COUNT - 1 && laser_x_count < GATHER_LASER_COUNT - 1)
			{
				record_done = 0;
			}
			debug_buff[4] = record_done;
			fp32 yaw_offset_plus = 0.025f;			
		  fp32 amend_yaw = 0.0f;
			//现在，记录完了刚开始的数据，该处理yaw漂移了
			//x记录完成
			if((record_done == 1 || record_done == 3) && laser_locate->laser.laser_amend_x_done_flag == 1)
			{
				//失能了X1
				if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X1_LASER || laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X1_Y1_LASER || laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X1_Y2_LASER)
				{
					//差值近似为delta，默认90度直角，近似为：
//					amend_yaw += atan2(laser_locate->laser.x2 - laser_locate->laser_last.x2, laser_locate->laser_last.x2) * yaw_offset_plus;
					amend_yaw = (laser_locate->laser.x2 - laser_locate->laser_last.x2) * yaw_offset_plus;
				}
				//失能了X2
				else if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X2_LASER || laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X2_Y1_LASER || laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X2_Y2_LASER)
				{
//					amend_yaw += atan2(laser_locate->laser.x1 - laser_locate->laser_last.x1, laser_locate->laser_last.x1) * yaw_offset_plus;
					amend_yaw = (laser_locate->laser.x1 - laser_locate->laser_last.x1) * yaw_offset_plus;
				}
				//没有失能
				else
				{
//					amend_yaw += (atan2(laser_locate->laser.x2 - laser_locate->laser_last.x2, laser_locate->laser_last.x2) * yaw_offset_plus + atan2(laser_locate->laser.x1 - laser_locate->laser_last.x1, laser_locate->laser_last.x1) * yaw_offset_plus) / 2.0f;
						amend_yaw = (laser_locate->laser.x2 - laser_locate->laser_last.x2) * yaw_offset_plus + (laser_locate->laser.x1 - laser_locate->laser_last.x1) * yaw_offset_plus ;
				}				
			}
			if((record_done == 2 || record_done == 3) && laser_locate->laser.laser_amend_y_done_flag == 1)	
			{
				//失能了Y1
				if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_Y1_LASER || laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X1_Y1_LASER || laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X2_Y1_LASER)
				{
//						amend_yaw += atan2(laser_locate->laser.y2 - laser_locate->laser_last.y2, laser_locate->laser_last.y2) * yaw_offset_plus;
					amend_yaw += (laser_locate->laser.y2 - laser_locate->laser_last.y2) * yaw_offset_plus;
				}
				//失能了Y2
				else if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_Y2_LASER || laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X1_Y2_LASER || laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X2_Y2_LASER)
				{
//						amend_yaw += atan2(laser_locate->laser.y1 - laser_locate->laser_last.y1, laser_locate->laser_last.y1) * yaw_offset_plus;
					amend_yaw += (laser_locate->laser.y1 - laser_locate->laser_last.y1) * yaw_offset_plus;
				}
				//没有失能
				else
				{
//						amend_yaw += (atan2(laser_locate->laser.y2 - laser_locate->laser_last.y2, laser_locate->laser_last.y2) * yaw_offset_plus + atan2(laser_locate->laser.y1 - laser_locate->laser_last.y1, laser_locate->laser_last.y1) * yaw_offset_plus) / 2.0f;					
					amend_yaw += (laser_locate->laser.y2 - laser_locate->laser_last.y2) * yaw_offset_plus + (laser_locate->laser.y1 - laser_locate->laser_last.y1) * yaw_offset_plus;
				}				
			}
			
			if(record_done == 3)
			{
				laser_locate->yaw_offset -= (amend_yaw / 2.0f); 
			}
			else if(record_done == 1 || record_done == 2)
			{
				laser_locate->yaw_offset -= amend_yaw;
			} 
			
			laser_locate->yaw_offset = rad_format(laser_locate->yaw_offset);
		}
		
		//修正完成，原地自旋
		if(laser_locate->laser.laser_amend_x_done_flag == 1 && laser_locate->laser.laser_amend_y_done_flag == 1)
		{
			distance.x = 0.0f;
			distance.y = 0.0f;
		}
	}
}

/**
   * @brief 延时动作快捷设置
   * @param 结构体，动作编号，编码器值，角度值，模式设置(LINE OR AROUND)，延时时间，是否禁用扫描模式
   * @retval None.
   */
static void path_not_event_set(path_control_t * not_event_set, uint16_t behaviour_num ,fp32 ecd_set, fp32 angle_set, uint8_t mod_set ,uint32_t delay_time,uint8_t disable_yaw_scan,	fp32 scan_angle_set)
{
	//防止误操作而导致的数组越界
	if(behaviour_num > 255)
	{
		return;
	}
	not_event_set->path_ecd_set.ecd_set[behaviour_num] = ecd_set;
	not_event_set->path_ecd_set.angle_set[behaviour_num] = angle_set;
	not_event_set->path_ecd_set.mod_set[behaviour_num] = mod_set;
	not_event_set->path_ecd_set.delay_mod[behaviour_num] = NOT_EVENT;
	not_event_set->path_ecd_set.delay_time[behaviour_num] = delay_time;
	not_event_set->path_ecd_set.disable_yaw_scan[behaviour_num] = disable_yaw_scan;
	not_event_set->path_ecd_set.scan_angle_set[behaviour_num] = scan_angle_set;
	not_event_set->path_ecd_set.special_flag[behaviour_num] = 0;
}

/**
   * @brief 事件延迟快速设置
   * @param 结构体，动作编号，编码器值，角度值，模式设置(LINE OR AROUND)，是否开启激光，修正逻辑优先级，激光是否局部失能，四个定位数x1，x2，y1，y2
   * @retval None.
   */
static void path_event_set(path_control_t * event_set, uint16_t behaviour_num ,fp32 ecd_set, fp32 angle_set, uint8_t mod_set, uint8_t laser_flag,uint8_t amend_priority, uint8_t disable, fp32 x1_set, fp32 x2_set, fp32 y1_set, fp32 y2_set)
{
	//防止误操作而导致的数组越界
	if(behaviour_num > 255)
	{
		return;
	}
	event_set->path_ecd_set.ecd_set[behaviour_num] = ecd_set;
	event_set->path_ecd_set.angle_set[behaviour_num] = angle_set;
	event_set->path_ecd_set.mod_set[behaviour_num] = mod_set;
	event_set->path_ecd_set.delay_mod[behaviour_num] = EVENT;
	event_set->path_ecd_set.delay_flag[behaviour_num] = 0;
	event_set->path_ecd_set.behaviour_done_flag[behaviour_num] = 0;
	event_set->path_ecd_set.laser_flag[behaviour_num] = laser_flag;
	event_set->path_ecd_set.laser_amend_priority[behaviour_num] = amend_priority;
	event_set->path_ecd_set.laser_disable[behaviour_num] = disable;
	event_set->path_ecd_set.laser_x1_set[behaviour_num] = x1_set;	
	event_set->path_ecd_set.laser_x2_set[behaviour_num] = x2_set;	
	event_set->path_ecd_set.laser_y1_set[behaviour_num] = y1_set;	
	event_set->path_ecd_set.laser_y2_set[behaviour_num] = y2_set;	
	event_set->path_ecd_set.disable_yaw_scan[behaviour_num] = ENABLE_SCAN;
	event_set->path_ecd_set.special_flag[behaviour_num] = 0;
}

/**
   * @brief 路径设置
	 * @param 结构体，路径编号，起始动作，结束动作，循环次数，是否为结束路径(0，不是,1:是)
   * @retval None.
   */
//PS:只有在结束路径末尾动作完成后，哨兵才会进行补血/复活，末尾动作必须为激光，时间延迟，且delay_flag永不为1以及循环次数为1，即：不会切换到下一条路径，以及确保是最后一个动作
static void path_cir_set(path_control_t * path_cir, uint8_t cir_num, uint8_t cir_min, uint8_t cir_max, uint16_t cir_time, uint8_t end_flag)
{
	//防止误操作导致的数组越界
	if(cir_num > 31)
	{
		return;
	}
	path_cir->cir.cir_min[cir_num] = cir_min;
	path_cir->cir.cir_max[cir_num] = cir_max;
	path_cir->cir.cir_time[cir_num] = cir_time;
	path_cir->cir.mark_end_flag[cir_num] = end_flag;
}

/**
   * @brief 特殊动作，用于RM比赛中，位于回血点补血时，开启自旋，枪口朝一个方向
   * @param ...
   * @retval None.
   */
static void path_special_set(path_control_t * path_special, uint16_t behaviour_num, uint32_t delay_time,fp32 scan_angle_set)
{
	//防止误设置导致的数组越界
	if(behaviour_num > 256)
	{
		return;
	}
	path_special->path_ecd_set.delay_mod[behaviour_num] = NOT_EVENT;
	path_special->path_ecd_set.special_flag[behaviour_num] = 1;
	path_special->path_ecd_set.ecd_set[behaviour_num] = 0.0001f;
	path_special->path_ecd_set.delay_time[behaviour_num] = delay_time;
	path_special->path_ecd_set.disable_yaw_scan[behaviour_num] = DISABLE_SCAN;
	path_special->path_ecd_set.disable_yaw_scan[behaviour_num] = scan_angle_set;
	
}

/**
   * @brief 补血/回复路径设置
   * @param 结构体，起始动作，结束动作
   * @retval None.
   */
static void restore_cir_set(path_control_t * path_restore, uint8_t cir_min, uint8_t cir_max)
{
	path_restore->cir.restore_cir_min = cir_min;
	path_restore->cir.restore_cir_max = cir_max;
}

/**
   * @brief 回血/复活检测
   * @param 结构体
   * @retval None.
   */
static void restore_detect(path_control_t * restore_detect)
{
	static uint8_t restore_detect_flag = 0;
	//attack
	if(schem_choose == 0 || schem_choose == 2 || schem_choose == 3)
	{
		//为结束路径，结束动作，且激光修正完毕，进攻逻辑，回退肯定是小于300血了 / 200血
		if(game_counter >= 240000 && restore_detect_flag == 0 && restore_detect->laser.laser_amend_x_done_flag == 1 && restore_detect->laser.laser_amend_y_done_flag == 1 && restore_detect->cir.mark_end_flag[restore_detect->path_count] == 1 && restore_detect->behaviour_count == restore_detect->cir.cir_max[restore_detect->path_count])
		{
			restore_detect_flag = 1;
			restore_detect->cir.restore_flag = 1;
			restore_detect->path_change_flag = 0;
		}		
	}
	//defence
	else if(schem_choose == 1)
	{
		if(game_counter >= 240000 && restore_detect_flag == 0 && restore_detect->laser.laser_amend_x_done_flag == 1 && restore_detect->laser.laser_amend_y_done_flag == 1 && restore_detect->cir.mark_end_flag[restore_detect->path_count] == 1 && restore_detect->behaviour_count == restore_detect->cir.cir_max[restore_detect->path_count])
		{
			if(sentinel_SENTINEL_UP < DEFENCE_MIN_HP)
			{
				restore_detect_flag = 1;
				restore_detect->cir.restore_flag = 1;
				restore_detect->path_change_flag = 0;				
			}
		}
	}
}

/// @brief 浮点数转换为字节流 IEEE754
/// @param floatNum 需要转换的浮点数
/// @param byteArry 用于存储字节流的长度为4的字节数组
static void FloatToByte(float floatNum, uint8_t *byteArry)
{
  uint8_t *pchar = (uint8_t *)&floatNum;
  for (int i = 0; i < sizeof(float); i++)
  {
    byteArry[3 - i] = *pchar;
    pchar++;
  }
}

/// @brief 将字节流转换为浮点数 IEEE754
/// @param Byte 字节数组
/// @param num 字节数，因为转换为float，一般在这里使用sizeof(float)
/// @return 返回转换后的浮点数
static float Hex_To_Decimal(uint8_t *Byte, int num)
{
  uint8_t cByte[4];
  for (int i = 0; i < num; i++)
  {
    cByte[i] = Byte[3 - i];
  }
  float pfValue = *(float *)&cByte;
  return pfValue;
}

/// @brief crc8校验算法
uint8_t crc8(uint8_t *data, int size)
{
  uint8_t crc = 0x00;
  uint8_t poly = 0x07;
  int bit;
  while (size--)
  {
    crc ^= *data++;
    for (bit = 0; bit < 8; bit++)
    {
      if (crc & 0x80)
      {
        crc = (crc << 1) ^ poly;
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

/**
   * @brief 对激光所属：STM32F407ZG发送数据
   * @param None.
   * @retval None.
   */
static void send_laser_data()
{
	divide_count++;
	//当分频达到最大的时，这里设置SET_COUNT，控制在100hz，防止发送频率过快
	if(divide_count == SET_COUNT)
	{
		divide_count = 0;
		
		//头帧
		chassis_buff[0] = SEND_EOF;
		
		//底盘数据
		FloatToByte(rad_format(chassis_move.chassis_INS_angle[0] + path_control.yaw_offset),&chassis_buff[1]);
		
		//crc校验
		chassis_buff[5] = crc8(&chassis_buff[0],5);
		
		//尾帧
		chassis_buff[6] = SEND_NOF;
		
		HAL_UART_Transmit(&huart1, chassis_buff, CHASSIS_SEND_DATA,1000);
	}
}

/**
   * @brief 处理激光所属：STM32F407ZG数据
   * @param None.
   * @retval None.
   */
void accept_laser_data()
{
	//头帧，尾帧以及CRC校验
		if(laser_buff[0] == 0x5A && crc8(&laser_buff[0],17) == laser_buff[17] && laser_buff[18] == 0xFF)
		{
			//校验无误后对数据进行解算处理
			path_control.laser.x1 = Hex_To_Decimal(&laser_buff[1],4);
			path_control.laser.x2 = Hex_To_Decimal(&laser_buff[5],4);
			path_control.laser.y1 = Hex_To_Decimal(&laser_buff[9],4);
			path_control.laser.y2 = Hex_To_Decimal(&laser_buff[13],4);
			path_control.laser.valid_flag = 1;
		}
}

/**
   * @brief 串口1中断，接收来自激光所属：STM32F407ZG数据
   * @param None.
   * @retval None.
   */
void USART1_IRQHandler(void)
{
	int temp_flag = 0;
	int temp;
	temp_flag = __HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE);
	if((temp_flag != RESET))																
	{
			__HAL_UART_CLEAR_IDLEFLAG(&huart1);
			temp = huart1.Instance->SR;   	//清除标志位									
			temp = huart1.Instance->DR; 						
			HAL_UART_DMAStop(&huart1);   									
			temp = hdma_usart1_rx.Instance->NDTR; 	  		
			accept_laser_data();		
			   											
	}
	HAL_UART_Receive_DMA(&huart1,laser_buff,19); //使能DMA

  HAL_UART_IRQHandler(&huart1);

}

