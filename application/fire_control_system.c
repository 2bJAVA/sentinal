/**
  ****************************(C) COPYRIGHT 2023 ULTRA****************************
  * @file       fire_control_system.c/h
  * @brief      FreeRTOS���񣬻��ϵͳ
  *
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-1-24       ����             1. frame_done
	*  V1.0.1     2023-1-25       ����             2. done
	*  V1.0.2     2023-2-19       ����             3. add_hit_feedback
	*  V1.0.3     2023-5-2        ����             4. make concise
  *
  @verbatim Ϊ�˾�����Ӱ��ٷ��Ĳ���ϵͳ���룬�����ĵ����̡߳�
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
	
//��ǰ���ýṹ��ģ����ṹ���ƺ����ղ�����ȷ�����ݣ�ֱ�Ӱ������棬��һ��
uint8_t sentinel_side;   //�췽or����
uint8_t sentinel_SENTINEL_ID; //�ڱ�ID��
uint16_t sentinel_SENTINEL_MAX_UP; //�ڱ�����ֵ����
uint16_t sentinel_SENTINEL_UP; //�ڱ���ǰ����ֵ
uint16_t sentinel_SENTINEL_LAST_UP;//�ڱ���һ״̬����ֵ
uint16_t sentinel_BASE_UP;     //��������ֵ
uint16_t sentinel_ENEMY_BASE_UP; //�з���������ֵ
uint16_t sentinel_shooter_id1_17mm_cooling_rate; //1��17mmǹ��ÿ����ȴֵ
uint16_t sentinel_shooter_id1_17mm_cooling_limit;//1��17mmǹ����������
uint16_t sentinel_shooter_id1_17mm_speed_limit;  //1��17mmǹ�������ٶ�
uint16_t sentinel_shooter_id1_17mm_cooling_heat; //1��17mmǹ�ڵ�ǰ����
uint16_t sentinel_shooter_id2_17mm_cooling_limit;//2��17mmǹ����������
uint16_t sentinel_shooter_id2_17mm_cooling_heat; //2��17mmǹ�ڵ�ǰ����
uint8_t sentinel_bullet_freq;                    //�ӵ���Ƶ
float sentinel_SENTINEL_UP_PERCENT = 1.0f; //�ڱ�ʣ��Ѫ��(�ٷֱ�)
float sentinel_BASE_UP_PERCENT = 1.0f;  //����ʣ��Ѫ��(�ٷֱ�)
float sentinel_ENEMY_BASE_UP_PERCENT = 1.0f; //�з�����ʣ��Ѫ��(�ٷֱ�)
float sentinel_shooter_id1_17mm_cooling_heat_PERCENT = 0.0f; //�����ٷֱ�
float sentinel_shooter_id2_17mm_cooling_heat_PERCENT = 0.0f; //�����ٷֱ�
float sentinel_path_x = 0.0f; //��̨��ָ��x
float sentinel_path_y = 0.0f; //��̨��ָ��y
unsigned char sentinel_path_state_flag = 0; //��̨������״̬��Ϊ1�������ָ��

uint8_t sentinel_game_state = 0;

COOLING_RANGE_e cooling_range = general_setting; // ����ģʽ

float shoot_freq = 0;				// ��Ƶ->�������Ƶ��CAN���͵�ֵ�йأ�
unsigned char update_freq;		// ���ݸ���Ƶ��
float last_up;					// ��һ״̬����ֵ
unsigned char lose_up_flag = 0; // ��ʧѪ����־λ
unsigned char get_side_flag = 0;
unsigned char debug_game_progress = 0;

unsigned char sentinel_offline_wait_time = 0;

unsigned char gun_change_flag = 0;

unsigned int cool_system_wait_time = 0;

gun_control_t gun_cmd;

/**
 * @brief          ���ϵͳ����
 * @param[in]      argument: NULL
 * @retval         none
 */
void fire_control_system(void const *argument)
{
	vTaskDelay(FIRE_CONTROL_SYSTEM_INIT_TIME);
	get_side();																				  // ��ȡ��ս��Ϣ
	record_state_play();																	  // ��ȡ������Ϣ
	while (1)
	{
		sentinel_state_update();				   // ���±�����Ϣ
		fire_control_system_mod_choose(); //���ϵͳģʽѡ��
		cooling_system();						   // ��ȴϵͳ����
		record_update_freq();					   // ��¼���´���
		check_up();								   // ��⵱ǰѪ��
		vTaskDelay(FIRE_CONTROL_SYSTEM_WAIT_TIME); // ����Ϊ10msһˢ�£��������ø�������ϵͳˢ��Ƶ��
	}
}

/**
 * @brief ��ȡ��ս��Ϣ
 * @param None.
 * @retval None.
 */
static void get_side()
{
	sentinel_SENTINEL_ID = robot_state.robot_id;
	if (sentinel_SENTINEL_ID == 7) // �ж��ڱ�ID��
	{
		sentinel_side = RED;
	}
	else if (sentinel_SENTINEL_ID == 107)
	{
		sentinel_side = BLUE;
	}
}

/**
 * @brief ��¼������Ϣ
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
 * @brief ���±�����Ϣ(��ǰ)
 * @param None.
 * @retval None.
 */
static void sentinel_state_update()
{
	if (update_freq % 2 == 0) // ����ÿ20ms����
	{
		sentinel_shooter_id1_17mm_cooling_heat = power_heat_data_t.shooter_17mm_1_barrel_heat;										   // ����
		sentinel_shooter_id2_17mm_cooling_heat = power_heat_data_t.shooter_17mm_2_barrel_heat;				
		sentinel_shooter_id1_17mm_cooling_heat_PERCENT = sentinel_shooter_id1_17mm_cooling_heat / sentinel_shooter_id1_17mm_cooling_limit; // �����ٷֱ�
		sentinel_shooter_id2_17mm_cooling_heat_PERCENT = sentinel_shooter_id2_17mm_cooling_heat / sentinel_shooter_id2_17mm_cooling_limit; // �����ٷֱ�
		sentinel_SENTINEL_UP = robot_state.current_HP;																					   // Ѫ��
		sentinel_SENTINEL_UP_PERCENT = sentinel_SENTINEL_UP / sentinel_SENTINEL_MAX_UP;													   // Ѫ���ٷֱ�
		last_up = sentinel_SENTINEL_UP_PERCENT;
	}
	if (update_freq % 100 == 0)						 // ����1s����һ��
	{
	  if (sentinel_side == RED) // �췽
		{
			sentinel_BASE_UP_PERCENT = game_robot_HP_t.red_base_HP / sentinel_BASE_UP;
			sentinel_ENEMY_BASE_UP_PERCENT = game_robot_HP_t.blue_base_HP / sentinel_ENEMY_BASE_UP;
		}
		else if (sentinel_side == BLUE) // ����
		{
			sentinel_BASE_UP_PERCENT = game_robot_HP_t.blue_base_HP / sentinel_BASE_UP;
			sentinel_ENEMY_BASE_UP_PERCENT = game_robot_HP_t.red_base_HP / sentinel_ENEMY_BASE_UP;
		}
		//������ʼ��־λ:ע�͵�game_state.game_progress��ʾΪ�������ģ�������ʼ�������������⴮���룬watch���ڴ򿪼���
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
 * @brief ���ϵͳģʽѡ��
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
 * @brief ��ȴϵͳ(�ȶ�������ĳһ��Χ��)(50HZ->20ms����)
 * @param None.
 * @retval None.
 */
static void cooling_system()
{
	if (update_freq % 2 == 0) // ����ÿ20ms����
	{
			//ǹ�ܸ����ɹ�����ȴϵͳ�ȴ�һ��ʱ��
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
				//ѡ��ǹ��1��ǹ��1������ �� ѡ��ǹ��2��ǹ��2������
			  if(((sentinel_shooter_id1_17mm_cooling_heat_PERCENT > GENERAL_LIMIT && gun_cmd.gun_behaviour == 0)\
						|| (sentinel_shooter_id2_17mm_cooling_heat_PERCENT > GENERAL_LIMIT && gun_cmd.gun_behaviour == 1))\
						&& (cooling_range == general_setting))
					{
						//��ǹ��
						gun_change_flag = 1;
						//shoot_freq = -COOLING_SHOOT_FREQ;
					}
			}
//			//����ʼ������ֻʹ��һ��ǹ��
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
 * @brief ��¼����ˢ�´���
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
 * @brief ��Ѫ���
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
   * @brief ǹ��ת����ʼ��
   * @param ǹ�ܿ��ƽṹ��
   * @retval None.
   */
void gun_change_init(gun_control_t * gun_control_init)
{
	static const fp32 gun_angle_pid[3] = {GUN_ANGLE_PID_KP,GUN_ANGLE_PID_KI, GUN_ANGLE_PID_KD};
	static const fp32 gun_speed_pid[3] = {GUN_SPEED_PID_KP,GUN_SPEED_PID_KI, GUN_SPEED_PID_KD};	
	const static fp32 gun_order_filter[1] = {GUN_SPEED_NUM};
	
	//ǹ�ܱ�������ʼ��
	PID_init(&gun_control_init->gun_angle_pid, PID_POSITION, gun_angle_pid, GUN_ANGLE_PID_MAX_OUT, GUN_ANGLE_PID_MAX_IOUT);
	
	//ǹ���ٶȳ�ʼ��
	PID_init(&gun_control_init->gun_speed_pid, PID_POSITION, gun_speed_pid, GUN_SPEED_PID_MAX_OUT, GUN_SPEED_PID_MAX_IOUT);
	
  //��һ���˲�����б����������
  first_order_filter_init(&gun_control_init->gun_speed_slow_set, GUN_CONTROL_TIME, gun_order_filter);
	
	//gun_done�źų�ʼ��
	gun_control_init->gun_done = 0;
	
	//gun_behaviour�źų�ʼ��
	gun_control_init->gun_behaviour = 0;
	gun_control_init->last_gun_change_flag = 0;

	//��ȡ2006�������ָ��
	gun_control_init->gun_motor_measure = get_gun_motor_measure_point();
	
	//���µ���ٶ�
	gun_control_init->gun_speed = gun_control_init->gun_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED;
	
	//
	gun_control_init->ecd_count = 0;
	
	//���µ���Ƕ�
	gun_control_init->gun_angle = gun_control_init->gun_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
	
	//��ʼ����־λ
	gun_control_init->gun_init_flag = 0;
	
	//gun����Ƕ�����
	gun_control_init->gun_lock_flag = 0;
	
}

/**
   * @brief �ڱ�ǹ�ܿ���
   * @param ǹ�ܿ��ƽṹ��
   * @retval None.
   */

void gun_control(gun_control_t * gun_control_cmd)
{
	//���µ���ٶ�
	gun_control_cmd->gun_speed = gun_control_cmd->gun_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED;
	
	// ���Ȧ�����ã� ��Ϊ�������תһȦ�� �������ת 36Ȧ������������ݴ������������ݣ����ڿ��������Ƕ�
  if (gun_control_cmd->gun_motor_measure->ecd - gun_control_cmd->gun_motor_measure->last_ecd > HALF_ECD_RANGE)
  {
    gun_control_cmd->ecd_count--;
  }
  else if (gun_control_cmd->gun_motor_measure->ecd - gun_control_cmd->gun_motor_measure->last_ecd < -HALF_ECD_RANGE)
  {
    gun_control_cmd->ecd_count++;
  }
		
	//���µ���Ƕ�
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
	
	
	//�����Ƿ����ǹ��ȷ��ǹ��λ��
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
	
	//�����Ƕ�����
	if(gun_control_cmd->gun_lock_flag == 1)
	{
		//�ǶȻ�
		gun_control_cmd->gun_speed_set = PID_calc(&gun_control_cmd->gun_angle_pid, gun_control_cmd->gun_angle, gun_control_cmd->gun_angle_set);
		
		//�˲�
		first_order_filter_cali(&gun_control_cmd->gun_speed_slow_set,gun_control_cmd->gun_speed_set);
		gun_control_cmd->gun_speed_set = gun_control_cmd->gun_speed_slow_set.out;
	}
	
	if(gun_control_cmd->gun_init_flag == 0)
	{
		gun_control_cmd->gun_speed_set = GUN_INIT_SPEED;
	}
	
	if(((sentinel_game_state == START_GAME) || (sentinel_game_state == NOT_START_GAME && gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)) && gun_control_cmd->gun_lock_flag == 0)
	{
		//��ת���
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
			//��ת����
			else if(gun_control_cmd->gun_init_flag == 1)
			{
				//�����Ƕ�����
				gun_control_cmd->gun_lock_flag = 1;
				//��ȡ��ǰ�Ƕ�
				gun_control_cmd->gun_angle_set = (gun_control_cmd->ecd_count * ECD_RANGE + gun_control_cmd->gun_motor_measure->ecd - gun_control_cmd->ecd_offset) * MOTOR_ECD_TO_ANGLE;
			}
		}	
		else
		{
			gun_control_cmd->gun_block_time = 0;
		}
	}

	//�ٶȻ�
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
