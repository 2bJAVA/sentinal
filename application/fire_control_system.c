/**
  ****************************(C) COPYRIGHT 2023 ULTRA****************************
  * @file       fire_control_system.c/h
  * @brief      FreeRTOS任务，火控系统
  *
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-1-24       天衍             1. frame_done
	*  V1.0.1     2023-1-25       天衍             2. done
	*  V1.0.2     2023-2-19       天衍             3. add_hit_feedback
	*  V1.0.3     2023-5-2        天衍             4. make concise
  *
  @verbatim 为了尽量不影响官方的裁判系统代码，建立的单独线程。
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 ULTRA****************************
  */
#include "fire_control_system.h"
#include "freertos.h"
#include "task.h"
#include "cmsis_os.h"
#include "main.h"
#include "referee.h"
#include "pid.h"
#include "chassis_behaviour.h"
#include "gimbal_attitude.h"
#include "shoot.h"
#include "gimbal_task.h"
#include "gimbal_behaviour.h"

static void record_state_play(void);
static void get_side(void);
static void sentinel_state_update(void);
static void cooling_system(void);
static void record_update_freq(void);
static void check_up(void);
static void fire_control_system_mod_choose(void);
	
//以前是用结构体的，但结构体似乎接收不了正确的数据，直接摆在下面，都一样
uint8_t sentinel_side;   //红方or蓝方
uint8_t sentinel_SENTINEL_ID; //哨兵ID号
uint16_t sentinel_SENTINEL_MAX_UP; //哨兵生命值上限
uint16_t sentinel_SENTINEL_UP; //哨兵当前生命值
uint16_t sentinel_SENTINEL_LAST_UP;//哨兵上一状态生命值
uint16_t sentinel_BASE_UP;     //基地生命值
uint16_t sentinel_ENEMY_BASE_UP; //敌方基地生命值
uint16_t sentinel_shooter_id1_17mm_cooling_rate; //1号17mm枪口每秒冷却值
uint16_t sentinel_shooter_id1_17mm_cooling_limit;//1号17mm枪口热量上限
uint16_t sentinel_shooter_id1_17mm_speed_limit;  //1号17mm枪口上限速度
uint16_t sentinel_shooter_id1_17mm_cooling_heat; //1号17mm枪口当前热量
uint16_t sentinel_shooter_id2_17mm_cooling_limit;//2号17mm枪口热量上限
uint16_t sentinel_shooter_id2_17mm_cooling_heat; //2号17mm枪口当前热量
uint8_t sentinel_bullet_freq;                    //子弹射频
float sentinel_SENTINEL_UP_PERCENT = 1.0f; //哨兵剩余血量(百分比)
float sentinel_BASE_UP_PERCENT = 1.0f;  //基地剩余血量(百分比)
float sentinel_ENEMY_BASE_UP_PERCENT = 1.0f; //敌方基地剩余血量(百分比)
float sentinel_shooter_id1_17mm_cooling_heat_PERCENT = 0.0f; //热量百分比
float sentinel_shooter_id2_17mm_cooling_heat_PERCENT = 0.0f; //热量百分比
float sentinel_path_x = 0.0f; //云台手指令x
float sentinel_path_y = 0.0f; //云台手指令y
unsigned char sentinel_path_state_flag = 0; //云台手命令状态，为1代表给过指令

uint8_t sentinel_game_state = 0;

COOLING_RANGE_e cooling_range = general_setting; // 常规模式

float shoot_freq = 0;				// 射频->（这个射频与CAN发送的值有关）
unsigned char update_freq;		// 数据更新频率
float last_up;					// 上一状态生命值
unsigned char lose_up_flag = 0; // 损失血量标志位
unsigned char get_side_flag = 0;
unsigned char debug_game_progress = 0;

unsigned char sentinel_offline_wait_time = 0;

unsigned char gun_change_flag = 0;

unsigned int cool_system_wait_time = 0;

gun_control_t gun_cmd;

/**
 * @brief          火控系统任务
 * @param[in]      argument: NULL
 * @retval         none
 */
void fire_control_system(void const *argument)
{
	vTaskDelay(FIRE_CONTROL_SYSTEM_INIT_TIME);
	get_side();																				  // 获取对战信息
	record_state_play();																	  // 获取比赛信息
	while (1)
	{
		sentinel_state_update();				   // 更新比赛信息
		fire_control_system_mod_choose(); //火控系统模式选择
		cooling_system();						   // 冷却系统启动
		record_update_freq();					   // 记录更新次数
		check_up();								   // 检测当前血量
		vTaskDelay(FIRE_CONTROL_SYSTEM_WAIT_TIME); // 设置为10ms一刷新，方便配置各个数据系统刷新频率
	}
}

/**
 * @brief 获取对战信息
 * @param None.
 * @retval None.
 */
static void get_side()
{
	sentinel_SENTINEL_ID = robot_state.robot_id;
	if (sentinel_SENTINEL_ID == 7) // 判断哨兵ID号
	{
		sentinel_side = RED;
	}
	else if (sentinel_SENTINEL_ID == 107)
	{
		sentinel_side = BLUE;
	}
}

/**
 * @brief 记录比赛信息
 * @param None.
 * @retval None.
 */
static void record_state_play()
{
	sentinel_SENTINEL_UP_PERCENT = 1;
	sentinel_BASE_UP_PERCENT = 1;
	sentinel_ENEMY_BASE_UP_PERCENT = 1;

	if (sentinel_side == RED)
	{
		sentinel_BASE_UP = game_robot_HP_t.red_base_HP;
		sentinel_ENEMY_BASE_UP = game_robot_HP_t.blue_base_HP;
	}
	else if (sentinel_side == BLUE)
	{
		sentinel_BASE_UP = game_robot_HP_t.red_base_HP;
		sentinel_ENEMY_BASE_UP = game_robot_HP_t.blue_base_HP;
	}

	sentinel_shooter_id1_17mm_cooling_limit = robot_state.shooter_barrel_heat_limit;
	sentinel_shooter_id1_17mm_cooling_rate = robot_state.shooter_barrel_cooling_value;
	sentinel_shooter_id2_17mm_cooling_limit = robot_state.shooter_barrel_heat_limit;
	sentinel_SENTINEL_MAX_UP = robot_state.maximum_HP;
	sentinel_shooter_id1_17mm_cooling_heat = power_heat_data_t.shooter_17mm_1_barrel_heat;
	sentinel_SENTINEL_LAST_UP = sentinel_SENTINEL_UP;
}

/**
 * @brief 更新比赛信息(当前)
 * @param None.
 * @retval None.
 */
static void sentinel_state_update()
{
	if (update_freq % 2 == 0) // 保障每20ms启动
	{
		sentinel_shooter_id1_17mm_cooling_heat = power_heat_data_t.shooter_17mm_1_barrel_heat;										   // 热量
		sentinel_shooter_id2_17mm_cooling_heat = power_heat_data_t.shooter_17mm_2_barrel_heat;				
		sentinel_shooter_id1_17mm_cooling_heat_PERCENT = sentinel_shooter_id1_17mm_cooling_heat / sentinel_shooter_id1_17mm_cooling_limit; // 热量百分比
		sentinel_shooter_id2_17mm_cooling_heat_PERCENT = sentinel_shooter_id2_17mm_cooling_heat / sentinel_shooter_id2_17mm_cooling_limit; // 热量百分比
		sentinel_SENTINEL_UP = robot_state.current_HP;																					   // 血量
		sentinel_SENTINEL_UP_PERCENT = sentinel_SENTINEL_UP / sentinel_SENTINEL_MAX_UP;													   // 血量百分比
		last_up = sentinel_SENTINEL_UP_PERCENT;
	}
	if (update_freq % 100 == 0)						 // 保障1s启动一次
	{
	  if (sentinel_side == RED) // 红方
		{
			sentinel_BASE_UP_PERCENT = game_robot_HP_t.red_base_HP / sentinel_BASE_UP;
			sentinel_ENEMY_BASE_UP_PERCENT = game_robot_HP_t.blue_base_HP / sentinel_ENEMY_BASE_UP;
		}
		else if (sentinel_side == BLUE) // 蓝方
		{
			sentinel_BASE_UP_PERCENT = game_robot_HP_t.blue_base_HP / sentinel_BASE_UP;
			sentinel_ENEMY_BASE_UP_PERCENT = game_robot_HP_t.red_base_HP / sentinel_ENEMY_BASE_UP;
		}
		//比赛开始标志位:注释掉game_state.game_progress表示为开启电控模拟比赛开始，并开启下面这串代码，watch窗口打开即可
//		if(game_state.game_progress == 4)
//		{
//			sentinel_game_state = START_GAME;
//			sentinel_offline_wait_time = 0;
//		}
	    sentinel_game_state = NOT_START_GAME;
	    sentinel_offline_wait_time = 0;
//	    sentinel_game_state = START_GAME;
//	    sentinel_offline_wait_time = 0;
		
		if(debug_game_progress == 4)
		{
			sentinel_game_state = START_GAME;
			sentinel_offline_wait_time = 0;
		}
	  else
		{
			sentinel_offline_wait_time++;
		}
		if(sentinel_offline_wait_time >= SENTINEL_OFFLINE_WAIT_TIME)
		{
			sentinel_game_state = NOT_START_GAME;
			sentinel_offline_wait_time = SENTINEL_OFFLINE_WAIT_TIME;
		}
		get_side();
	}
}

/**
 * @brief 火控系统模式选择
 * @param None.
 * @retval None.
 */
static void fire_control_system_mod_choose()
{
	if(sentinel_SENTINEL_UP_PERCENT >= SENTINEL_RAGE_HP_PERCENT && sentinel_ENEMY_BASE_UP_PERCENT >= SENTINEL_ENEMY_BASE_UP_PERCENT && sentinel_BASE_UP_PERCENT >= SENTINEL_BASE_UP_PERCENT)
	{
		cooling_range = general_setting;
	}
	else 
	{
		cooling_range = rage_setting;
	}
}

/**
 * @brief 冷却系统(稳定热量在某一范围内)(50HZ->20ms启动)
 * @param None.
 * @retval None.
 */
static void cooling_system()
{
	if (update_freq % 2 == 0) // 保障每20ms启动
	{
			//枪管更换成功后冷却系统等待一段时间
			if(gun_change_flag == 1 && gun_cmd.gun_done == 1)
			{
				if(cool_system_wait_time < COOL_SYSTEM_WAIT_TIME)
				{
					cool_system_wait_time++;
					return;
				}
				else
				{
					cool_system_wait_time = 0;
					gun_change_flag = 0;
				}
			}
			if(sentinel_game_state == START_GAME)
			{
				//选中枪管1，枪管1超热量 或 选中枪管2，枪管2超热量
			  if(((sentinel_shooter_id1_17mm_cooling_heat_PERCENT > GENERAL_LIMIT && gun_cmd.gun_behaviour == 0)\
						|| (sentinel_shooter_id2_17mm_cooling_heat_PERCENT > GENERAL_LIMIT && gun_cmd.gun_behaviour == 1))\
						&& (cooling_range == general_setting))
					{
						//换枪管
						gun_change_flag = 1;
						//shoot_freq = -COOLING_SHOOT_FREQ;
					}
			}
//			//不开始比赛，只使用一个枪管
//			if(sentinel_shooter_id1_17mm_cooling_heat_PERCENT > RAGE_LIMIT && cooling_range == rage_setting)
//			{
//				shoot_freq = -COOLING_SHOOT_FREQ;
//			}
//			else
//			{
//				shoot_freq = 0; 
//			}
	}
}

/**
 * @brief 记录数据刷新次数
 * @param  None.
 * @retval None.
 */
static void record_update_freq()
{
	update_freq++;
	if (update_freq == 100)
	{
		update_freq = 0;
	}
}

/**
 * @brief 损血检测
 * @param None.
 * @retval None.
 */
static void check_up()
{
	if (last_up > sentinel_SENTINEL_UP_PERCENT)
	{
		last_up = sentinel_SENTINEL_UP_PERCENT;
		lose_up_flag = 1;
	}
}

/**
   * @brief 枪管转换初始化
   * @param 枪管控制结构体
   * @retval None.
   */
void gun_change_init(gun_control_t * gun_control_init)
{
	static const fp32 gun_angle_pid[3] = {GUN_ANGLE_PID_KP,GUN_ANGLE_PID_KI, GUN_ANGLE_PID_KD};
	static const fp32 gun_speed_pid[3] = {GUN_SPEED_PID_KP,GUN_SPEED_PID_KI, GUN_SPEED_PID_KD};	
	const static fp32 gun_order_filter[1] = {GUN_SPEED_NUM};
	
	//枪管编码器初始化
	PID_init(&gun_control_init->gun_angle_pid, PID_POSITION, gun_angle_pid, GUN_ANGLE_PID_MAX_OUT, GUN_ANGLE_PID_MAX_IOUT);
	
	//枪管速度初始化
	PID_init(&gun_control_init->gun_speed_pid, PID_POSITION, gun_speed_pid, GUN_SPEED_PID_MAX_OUT, GUN_SPEED_PID_MAX_IOUT);
	
  //用一阶滤波代替斜波函数生成
  first_order_filter_init(&gun_control_init->gun_speed_slow_set, GUN_CONTROL_TIME, gun_order_filter);
	
	//gun_done信号初始化
	gun_control_init->gun_done = 0;
	
	//gun_behaviour信号初始化
	gun_control_init->gun_behaviour = 0;
	gun_control_init->last_gun_change_flag = 0;

	//获取2006电机数据指针
	gun_control_init->gun_motor_measure = get_gun_motor_measure_point();
	
	//更新电机速度
	gun_control_init->gun_speed = gun_control_init->gun_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED;
	
	//
	gun_control_init->ecd_count = 0;
	
	//更新电机角度
	gun_control_init->gun_angle = gun_control_init->gun_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
	
	//初始化标志位
	gun_control_init->gun_init_flag = 0;
	
	//gun电机角度锁定
	gun_control_init->gun_lock_flag = 0;
	
}

/**
   * @brief 哨兵枪管控制
   * @param 枪管控制结构体
   * @retval None.
   */

void gun_control(gun_control_t * gun_control_cmd)
{
	//更新电机速度
	gun_control_cmd->gun_speed = gun_control_cmd->gun_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED;
	
	// 电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
  if (gun_control_cmd->gun_motor_measure->ecd - gun_control_cmd->gun_motor_measure->last_ecd > HALF_ECD_RANGE)
  {
    gun_control_cmd->ecd_count--;
  }
  else if (gun_control_cmd->gun_motor_measure->ecd - gun_control_cmd->gun_motor_measure->last_ecd < -HALF_ECD_RANGE)
  {
    gun_control_cmd->ecd_count++;
  }
		
	//更新电机角度
	gun_control_cmd->gun_angle =  (gun_control_cmd->ecd_count * ECD_RANGE + gun_control_cmd->gun_motor_measure->ecd - gun_control_cmd->ecd_offset) * MOTOR_ECD_TO_ANGLE;
	gun_control_cmd->gun_last_angle = gun_control_cmd->gun_angle;
	
	if(gun_change_flag != gun_control_cmd->last_gun_change_flag && gun_control_cmd->last_gun_change_flag == 0)
	{
		gun_control_cmd->gun_block_time = 0; 
		gun_control_cmd->gun_lock_flag = 0;		
		gun_control_cmd->gun_done = 0;
		if(gun_control_cmd->gun_behaviour == 0)
		{
			gun_control_cmd->gun_behaviour = 1;
		}
		else if(gun_control_cmd->gun_behaviour == 1)
		{
			gun_control_cmd->gun_behaviour = 0;
		}
	}
	
	
	//根据是否更换枪管确立枪管位置
	if(gun_control_cmd->gun_behaviour == 0)
	{
		gun_control_cmd->gun_speed_set = GUN_NORMAL_SPEED;
	}
	else if(gun_control_cmd->gun_behaviour == 1)
	{
		gun_control_cmd->gun_speed_set = -GUN_NORMAL_SPEED;
	}

	gun_control_cmd->last_gun_change_flag = gun_change_flag;
	
	gun_control_cmd->gun_angle_pid.mode = PID_POSITION;
	gun_control_cmd->gun_angle_pid.Kp = GUN_ANGLE_PID_KP;
	gun_control_cmd->gun_angle_pid.Ki = GUN_ANGLE_PID_KI;
	gun_control_cmd->gun_angle_pid.Kd = GUN_ANGLE_PID_KD;
	gun_control_cmd->gun_angle_pid.max_out = GUN_ANGLE_PID_MAX_OUT;
	gun_control_cmd->gun_angle_pid.max_iout = GUN_ANGLE_PID_MAX_IOUT;
	
	//开启角度锁定
	if(gun_control_cmd->gun_lock_flag == 1)
	{
		//角度环
		gun_control_cmd->gun_speed_set = PID_calc(&gun_control_cmd->gun_angle_pid, gun_control_cmd->gun_angle, gun_control_cmd->gun_angle_set);
		
		//滤波
		first_order_filter_cali(&gun_control_cmd->gun_speed_slow_set,gun_control_cmd->gun_speed_set);
		gun_control_cmd->gun_speed_set = gun_control_cmd->gun_speed_slow_set.out;
	}
	
	if(gun_control_cmd->gun_init_flag == 0)
	{
		gun_control_cmd->gun_speed_set = GUN_INIT_SPEED;
	}
	
	if(((sentinel_game_state == START_GAME) || (sentinel_game_state == NOT_START_GAME && gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)) && gun_control_cmd->gun_lock_flag == 0)
	{
		//堵转检测
		if(gun_control_cmd->gun_speed < GUN_BLOCK_SPEED && gun_control_cmd->gun_speed > -GUN_BLOCK_SPEED && gun_control_cmd->gun_block_time < GUN_BLOCK_TIME && gun_control_cmd->gun_done == 0)
		{
			gun_control_cmd->gun_block_time++;
		}
		else if(gun_control_cmd->gun_block_time >= GUN_BLOCK_TIME)
		{
			if(gun_control_cmd->gun_init_flag == 0)
			{
				gun_control_cmd->gun_init_flag = 1;
				gun_control_cmd->ecd_offset = gun_control_cmd->gun_motor_measure->ecd;
				gun_control_cmd->ecd_count = 0;
				gun_control_cmd->gun_block_time = 0;
			}
			//堵转保护
			else if(gun_control_cmd->gun_init_flag == 1)
			{
				//开启角度锁定
				gun_control_cmd->gun_lock_flag = 1;
				//获取当前角度
				gun_control_cmd->gun_angle_set = (gun_control_cmd->ecd_count * ECD_RANGE + gun_control_cmd->gun_motor_measure->ecd - gun_control_cmd->ecd_offset) * MOTOR_ECD_TO_ANGLE;
			}
		}	
		else
		{
			gun_control_cmd->gun_block_time = 0;
		}
	}

	//速度环
	gun_control_cmd->give_current = PID_calc(&gun_control_cmd->gun_speed_pid, gun_control_cmd->gun_speed, gun_control_cmd->gun_speed_set);
	
	if(gun_control_cmd->gun_lock_flag == 1)
	{
		gun_control_cmd->gun_done = 1;
	}
	else
	{
		gun_control_cmd->gun_done = 0;
	}
}
