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
uint32_t game_counter = 0; //������ʱʱ��
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
  * @brief         ·�����񣬼�� PATH_TASK_WAIT_TIME 1ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void path_task(void const * argument)
{
	vTaskDelay(PATH_TASK_INIT_TIME);
	
	//·�������ʼ��
	path_task_init(&path_control);

  while (1)
  {
		//ģʽ�л����
		path_mode_change_control_transit(&path_control);
	
		//���ݸ���
		path_feedback_update(&path_control);
			
		//·���������
		path_control_loop(&path_control);
		
		//·������״̬����
		path_task_state_control(&path_control);
		
		//������ʱ����ֹ������
    vTaskDelay(PATH_TASK_WAIT_TIME);
  }
}

fp32 delta_angle;
/**
   * @brief ·�������ʼ��
   * @param ·������ṹ��
   * @retval None.
   */
static void path_task_init(path_control_t * path_control_init)
{
	static const fp32 path_speed_pid[3] = {PATH_SPEED_PID_KP,PATH_SPEED_PID_KI, PATH_SPEED_PID_KD};
	
	static const fp32 laser_speed_pid[3] = {LASER_SPEED_PID_KP,LASER_SPEED_PID_KI, LASER_SPEED_PID_KD};
	
	//PID��ʼ��
	PID_init(&path_control_init->path_speed_pid, PID_POSITION, path_speed_pid, PATH_SPEED_PID_MAX_OUT, PATH_SPEED_PID_MAX_IOUT);

	PID_init(&path_control_init->laser_speed_pid[0], PID_POSITION, laser_speed_pid, LASER_SPEED_PID_MAX_OUT, LASER_SPEED_PID_MAX_IOUT);

	PID_init(&path_control_init->laser_speed_pid[1], PID_POSITION, laser_speed_pid, LASER_SPEED_PID_MAX_OUT, LASER_SPEED_PID_MAX_IOUT);

	//��ջ�Ѫ/��Ѫ��־λ
	path_control_init->cir.restore_flag = 0;

	//���yaw_offset
	path_control_init->yaw_offset = 0.0f;
	
	//��������begin
	while(sentinel_game_state != START_GAME)
	{
		vTaskDelay(1);
	}
	//end
	
	//�������������
	path_control_clear_data(path_control_init);
	
	//·������begin 
	//ʹ��path_not_event_set()���Լ�path_event_set()�������������ٱ�д

  //��5000��������λ����ǽ����Χ
//	path_not_event_set(path_control_init,101,0.1f,0.0f,AROUND,200,DISABLE_SCAN);
	
	//�����д����Ҳ��֪���ܵ������ʱ�����ģ������д��һ��x1 = 1.0f��x2 = 1.0f��y1 = 1.0f��y2 = 1.0f�����Ը���ʵ�ʸ����demo
//	path_event_set(path_control_init,102,0.1f,0.0f,LINE,LASER_ENABLE,AMEND_Y_PRIORITY,DISABLE_X2_Y2_LASER,1.0f,1.0f,1.0f,1.0f);
	
	//·����д
//	path_cir_set(path_control_init,0,101,102,1,0);

	
	//����ѡ��ǰ�涼Ϊ����
	delta_angle = rad_format(chassis_move.chassis_yaw - chassis_move.chassis_INS_angle[0]);
	//�ж��½���

	//���ж��б�д·��
	//���̽Ƕ� - ��̨�Ƕ� < �趨�Ƕȣ�����һ��attack
	if(delta_angle < SCHEM_ATTACK_ANGLE_MAX && delta_angle > SCHEM_ATTACK_ANGLE_MIN)
	{
		schem_choose = 0;
		
		//��45��ǰѹ�г�����ǰ��
		path_not_event_set(path_control_init,0,15000.0f,PI / 4.0f,LINE,500,DISABLE_SCAN,0.0f);
		path_not_event_set(path_control_init,1,15000.0f,0.0f,LINE,500,DISABLE_SCAN,0.0f);
		
		//ʹ��x1,Y1,Y2С����ǰѹ
		path_event_set(path_control_init,2,0.0001f,0.0f,LINE,LASER_ENABLE,AMEND_X_PRIORITY,DISABLE_X2_LASER,1.0f,1.0f,1.0f,1.0f);
		
		//��Ѫ300ʹ�ü��⳷�ˣ�
		path_event_set(path_control_init,3,0.0001f,0.0f,LINE,LASER_ENABLE,AMEND_X_PRIORITY,DISABLE_X2_LASER,1.0f,1.0f,1.0f,1.0f);
		//�˻�ȥ
		path_not_event_set(path_control_init,4,-15000.0f,PI / 4.0f,LINE,500,DISABLE_SCAN,0.0f);
		//���˷�����
		path_not_event_set(path_control_init,5,5000.0f,0.0f,200,AROUND,DISABLE_SCAN,0.0f);
		path_event_set(path_control_init,6,0.1f,0.0f,LINE,LASER_ENABLE,AMEND_Y_PRIORITY,DISABLE_X1_LASER,1.0f,1.0f,1.0f,1.0f);
		
		//·������
		path_cir_set(path_control_init,0,6,1,1,1);
		
		//�ڴ˴���д��Ѫ·��,ͣ��5s��Ҳ���ܻ���Ҳ˵�������˴��������Ƿ���/�����Ҷ�û������Ҫ�����
		path_not_event_set(path_control_init,100,0.1f,0.0f,LINE,500,DISABLE_SCAN,0.0f);
		path_special_set(path_control_init,101,5000,0.0f);
		path_not_event_set(path_control_init,102,0.1f,0.0f,LINE,100,DISABLE_SCAN,0.0f);
		restore_cir_set(path_control_init,100,102);
	}
	//ǿ�����������ǿ��
	else if(delta_angle < SCHEM_ATTACK_CAPTURE_MAX && delta_angle > SCHEM_ATTACK_CAPTURE_MIN)
	{
		schem_choose = 2;
		//��45��ǰѹ�볡����С���ݼ���ط�
		path_not_event_set(path_control_init,0,15000.0f,PI / 4.0f,LINE,500,DISABLE_SCAN,0.0f);
		path_event_set(path_control_init,1,0.0001f,0.0f,LINE,LASER_ENABLE,AMEND_X_PRIORITY,DISABLE_X2_LASER,1.0f,1.0f,1.0f,1.0f);
		
		//��һ�����ˣ�С����ǿ�����棡��ǰ����������
		path_event_set(path_control_init,2,0.0001f,0.0f,LINE,LASER_ENABLE,AMEND_X_PRIORITY,DISABLE_X2_LASER,1.0f,1.0f,1.0f,1.0f);
		path_event_set(path_control_init,3,0.0001f,0.0f,LINE,LASER_ENABLE,AMEND_X_PRIORITY,USE_ALL_LASER,1.0f,1.0f,1.0f,1.0f);
		
		//ռ����ϣ�С�������� + ǿ��
		path_event_set(path_control_init,4,0.0001f,0.0f,LINE,LASER_ENABLE,AMEND_X_PRIORITY,DISABLE_X2_LASER,1.0f,1.0f,1.0f,1.0f);
		
		//��Ѫ300ʹ�ü��⳷�ˣ�
		path_event_set(path_control_init,5,0.0001f,0.0f,LINE,LASER_ENABLE,AMEND_X_PRIORITY,DISABLE_X2_LASER,1.0f,1.0f,1.0f,1.0f);
		//�˻�ȥ
		path_not_event_set(path_control_init,6,-15000.0f,PI / 4.0f,LINE,500,DISABLE_SCAN,0.0f);
		//���˷�����
		path_not_event_set(path_control_init,7,5000.0f,0.0f,200,AROUND,DISABLE_SCAN,0.0f);
		path_event_set(path_control_init,8,0.1f,0.0f,LINE,LASER_ENABLE,AMEND_Y_PRIORITY,DISABLE_X1_LASER,1.0f,1.0f,1.0f,1.0f);

		//·������
		path_cir_set(path_control_init,0,8,1,1,1);
		
		//�ڴ˴���д��Ѫ·��,ͣ��5s��Ҳ���ܻ���Ҳ˵�������˴��������Ƿ���/�����Ҷ�û������Ҫ�����
		path_not_event_set(path_control_init,100,0.1f,0.0f,LINE,500,DISABLE_SCAN,0.0f);
		path_special_set(path_control_init,101,5000,0.0f);
		path_not_event_set(path_control_init,102,0.1f,0.0f,LINE,100,DISABLE_SCAN,0.0f);
		restore_cir_set(path_control_init,100,102);
		
	}
	//ǿռ�������������
	else if(delta_angle < SCHEM_DEFENCE_CAPTURE_MAX && delta_angle > SCHEM_DEFENCE_CAPTURE_MIN)
	{
		schem_choose = 3;
		//��45��ǰѹ�볡����С���ݼ���ط�
		path_not_event_set(path_control_init,0,15000.0f,PI / 4.0f,LINE,500,DISABLE_SCAN,0.0f);
		path_event_set(path_control_init,1,0.0001f,0.0f,LINE,LASER_ENABLE,AMEND_X_PRIORITY,DISABLE_X2_LASER,1.0f,1.0f,1.0f,1.0f);
		
		//��һ�����ˣ�С����ǿ�����棡��ǰ����������
		path_event_set(path_control_init,2,0.0001f,0.0f,LINE,LASER_ENABLE,AMEND_X_PRIORITY,DISABLE_X2_LASER,1.0f,1.0f,1.0f,1.0f);
		path_event_set(path_control_init,3,0.0001f,0.0f,LINE,LASER_ENABLE,AMEND_X_PRIORITY,USE_ALL_LASER,1.0f,1.0f,1.0f,1.0f);
		
		//ռ����ϣ�С���ݻ��˼����
		path_event_set(path_control_init,4,0.0001f,0.0f,LINE,LASER_ENABLE,AMEND_X_PRIORITY,DISABLE_X2_LASER,1.0f,1.0f,1.0f,1.0f);
		
		//��Ѫ����200�˻�ȥ
		path_not_event_set(path_control_init,5,-15000.0f,PI / 4.0f,LINE,500,DISABLE_SCAN,0.0f);
		//���˷�����
		path_not_event_set(path_control_init,6,5000.0f,0.0f,200,AROUND,DISABLE_SCAN,0.0f);
		path_event_set(path_control_init,7,0.1f,0.0f,LINE,LASER_ENABLE,AMEND_Y_PRIORITY,DISABLE_X1_LASER,1.0f,1.0f,1.0f,1.0f);

		//·������
		path_cir_set(path_control_init,0,7,1,1,1);
		
		//�ڴ˴���д��Ѫ·��,ͣ��5s��Ҳ���ܻ���Ҳ˵�������˴��������Ƿ���/�����Ҷ�û������Ҫ�����
		path_not_event_set(path_control_init,100,0.1f,0.0f,LINE,500,DISABLE_SCAN,0.0f);
		path_special_set(path_control_init,101,5000,0.0f);
		path_not_event_set(path_control_init,102,0.1f,0.0f,LINE,100,DISABLE_SCAN,0.0f);
		restore_cir_set(path_control_init,100,102);
	}
	//����defence
	else
	{
		schem_choose = 1;
		
		//�ڷ���������(������ʹ�ÿ�ע��)
		path_not_event_set(path_control_init,0,5000.0f,0.0f,AROUND,200,DISABLE_SCAN,0.0f);
		path_event_set(path_control_init,1,0.1f,0.0f,LINE,LASER_ENABLE,AMEND_Y_PRIORITY,DISABLE_X1_LASER,1.0f,1.0f,1.0f,1.0f);
		path_cir_set(path_control_init,0,0,1,1,1);
	
		//�ڴ˴���д��Ѫ·��,ͣ��5s��Ҳ���ܻ���Ҳ˵�������˴��������Ƿ���/�����Ҷ�û������Ҫ�����
		path_not_event_set(path_control_init,100,0.1f,0.0f,LINE,500,DISABLE_SCAN,0.0f);
		path_special_set(path_control_init,101,5000,0.0f);
		path_not_event_set(path_control_init,102,0.1f,0.0f,LINE,100,DISABLE_SCAN,0.0f);
		restore_cir_set(path_control_init,100,102);
	}
	
	
}

/**
   * @brief ң����ģʽ��������ݸ���
   * @param ·������ṹ��
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
   * @brief ��ȡ������ֵ
   * @param ·������ṹ��
   * @retval None.
   */
static void path_feedback_update(path_control_t * path_coordinates)
{
	if((path_coordinates->path_pause_flag == 1 || chassis_behaviour_mode != CHASSIS_AUTO_SENTINEL) || (sentinel_game_state == NOT_START_GAME))
	{
		return;
	}
		// ���Ȧ�����ã� ��Ϊ�������תһȦ�� �������ת 36Ȧ������������ݴ������������ݣ����ڿ��������Ƕ�
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
	//��Ϊֱ��ģʽ
	if(path_coordinates->path_ecd_set.mod_set[path_coordinates->behaviour_count] == LINE)
	{
		path_coordinates->ecd_now = -((-path_coordinates->ecd_count1 + path_coordinates->ecd_count2 + path_coordinates->ecd_count3 - path_coordinates->ecd_count4) * ECD_RANGE \
																+ (-(chassis_move.motor_chassis[0].chassis_motor_measure->ecd - path_coordinates->ecd_offset1) + \
																	(chassis_move.motor_chassis[1].chassis_motor_measure->ecd - path_coordinates->ecd_offset2) + \
																	(chassis_move.motor_chassis[2].chassis_motor_measure->ecd - path_coordinates->ecd_offset3) - \
																	(chassis_move.motor_chassis[3].chassis_motor_measure->ecd - path_coordinates->ecd_offset4))) * ECD_TRANSFER;
	}
	//��Ϊ����ģʽ
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
   * @brief ·���������
   * @param ·������ṹ��
   * @retval None.
   */
static void path_control_loop(path_control_t * path_control_loop)
{
	if((path_control_loop->path_pause_flag == 1 || chassis_behaviour_mode != CHASSIS_AUTO_SENTINEL) || (sentinel_game_state == NOT_START_GAME))
	{
		return;
	}
	
	//����ǰ·��Ϊĩβ·�����Ҵﵽ���һ���������Ҽ�����������
	if(path_control_loop->cir.mark_end_flag[path_control_loop->path_count] && path_control_loop->laser.laser_amend_x_done_flag == 1 && path_control_loop->laser.laser_amend_y_done_flag == 1 && path_control_loop->behaviour_count == path_control_loop->cir.cir_max[path_control_loop->path_count])
	{
		//���ó�ʼ����
		if(path_control_loop->path_change_flag == 0)
		{
			path_control_loop->behaviour_count = path_control_loop->cir.restore_cir_min;
			path_control_loop->path_change_flag = 1;
		}		
	}
	//������ѭ������
	else if(path_control_loop->cir.cir_time[path_control_loop->path_count] > 0)
	{
		//���ó�ʼ����
		if(path_control_loop->path_change_flag == 0)
		{
			path_control_loop->behaviour_count = path_control_loop->cir.cir_min[path_control_loop->path_count];
			path_control_loop->path_change_flag = 1;
		}
	}
	//�л�����һ��·��
	 else 
	 {
		 distance.x = 0.0f;
		 distance.y = 0.0f;
		 if(path_control_loop->path_count < 31)
		 {
			 path_control_loop->path_count++;
		 }
		}
	 
		//��·����Ϊ0
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
		
				//��������󣬶��ٶ����ݽ������
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
				
				//����Ƕȱ仯�ϴ󣬽����ӳ�
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
				//���⶯������������С����
				distance.x = 0.0f;
				distance.y = 0.0f;
				distance.spining_flag = 1;
			}
			
			//�ж��Ƿ񵽴�λ��
			if(path_abs(path_control_loop->path_ecd_set.ecd_set[path_control_loop->behaviour_count] - path_control_loop->ecd_now) < ECD_ERROR || path_control_loop->path_ecd_set.special_flag[path_control_loop->behaviour_count] == 1)
			{
				//���ѡ�����ʱ���ӳ�ģʽ
				if(path_control_loop->path_ecd_set.delay_mod[path_control_loop->behaviour_count] == NOT_EVENT)
				{
					path_counter++;
					//�ӳ�ʱ��ﵽ
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
				//��Ϊ�¼��ӳ�
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
			//�������ĩβ����ΪUNLIMIT_TIME
			if(path_control_loop->behaviour_count > path_control_loop->cir.cir_max[path_control_loop->path_count] && path_control_loop->cir.cir_time[path_control_loop->path_count] == UNLIMIT_TIME)
			{
					path_control_loop->path_change_flag = 0;
			}
			//����ﵽĩβ����ѭ��������ΪUNLIMIT_TIME�����Ҳ�Ϊ�ظ�/����·��
		  else if(path_control_loop->behaviour_count > path_control_loop->cir.cir_max[path_control_loop->path_count] && path_control_loop->cir.cir_time[path_control_loop->path_count] != UNLIMIT_TIME && path_control_loop->cir.restore_flag == 0)
			{
				//ѭ�������ݼ�
				path_control_loop->path_change_flag = 0;
				path_control_loop->cir.cir_time[path_control_loop->path_count]--;
				//��û��ѭ������
				if(path_control_loop->cir.cir_time[path_control_loop->path_count] <= 0)
				{
					path_control_loop->path_count++;
				}
			}
			//�ظ�·���ﵽĩβ����ձ�־λ������end��������
			else if(path_control_loop->behaviour_count > path_control_loop->cir.restore_cir_max && path_control_loop->cir.restore_flag == 1)
			{
				path_control_loop->cir.restore_flag = 0;
			}
		}
		//·������ECD����0,���ô���,ȫ�����
		else
		{
			path_control_clear_data(path_control_loop);
		}
}

/**
   * @brief �������
   * @param ·������ṹ��
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
   * @brief �����ֵ
   * @param ����ֵ
   * @retval ����ֵ
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
   * @brief ·��������ͣ
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
   * @brief ·�������������
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
   * @brief ·������״̬����
   * @param ·������ṹ��
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
	//�����ж����� begin
	//�����������ӣ����ֵ��ˣ�ԭ������������(������������Ʈ���ƣ���Щ���벢����ʹ��)
	if(correlate_data() == FOUND_ENEMY)
	{
		path_control.path_pause_flag = 1;
		distance.spining_flag = 1;
	}
	else
	{
		path_control.path_pause_flag = 0;
	}
	
	//��Ѫԭ������������ʱ��PATH_SCAN_COUNTms
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
	
	//�򼤹ⲿ�ַ��͵�������
	send_laser_data();
	
	//��Ѫ/������
	restore_detect(path_state);
	
	//end
	//ʹ��mid360�Ӿ��״�(ʹ�ñ������ˣ��϶�Ҫʧ�ܵ���)
//	distance.control_priority = 0;
	//byd�Ӿ�ƨ��û�У�ʹ�ܱ�����
		distance.control_priority = 1;
		
	game_counter++;
}
	

//��д�¼��ӳ�(RM�����У��¼��ӳ�ֻ�ʺϼ���)
static void PATH_EVENT1(path_control_t * path_event1)
{
	static uint16_t capture_count = 0;
	//�������
	if(schem_choose == 0 && path_event1->path_ecd_set.laser_flag[path_event1->behaviour_count] == LASER_ENABLE)
	{
	//�ṩһ��ʹ�ü�������ӣ����Ҳ�δ���ò���
		//�¼��ӳ��ұ������������
		if(path_event1->path_ecd_set.delay_mod[path_event1->behaviour_count] == EVENT && path_event1->path_ecd_set.behaviour_done_flag[path_event1->behaviour_count] == 1 && (path_event1->behaviour_count == 2 || path_event1->behaviour_count == 3))
		{
			//����С�����Լ����ⶨλ
			laser_locate(path_event1);
			//��λ��ɣ�������һ������������״̬����Ѫ�ﵽ�趨ֵ
			if(path_event1->laser.laser_amend_x_done_flag == 1 && path_event1->laser.laser_amend_y_done_flag == 1 && (sentinel_SENTINEL_UP < ATTACK_MIN_HP) && path_event1->behaviour_count == 2 && path_event1->path_ecd_set.behaviour_done_flag[path_event1->behaviour_count] == 1)
			{
				path_event1->path_ecd_set.delay_flag[path_event1->behaviour_count] = 1;
			}
		}
		
		//����ʱ�����
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
		//������Ѫֱ����
		if(sentinel_SENTINEL_UP < ATTACK_MIN_HP && schem_choose == 2)
		{
			path_event1->behaviour_count = 6;
		}
		else if(sentinel_SENTINEL_UP < DEFENCE_MIN_HP && schem_choose == 1)
		{
			path_event1->behaviour_count = 5;
		}
		//��ʼǰѹվ��
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
		//��λ��ɣ�������һ������������״̬����Ѫ�ﵽ�趨ֵ
		if(path_event1->laser.laser_amend_x_done_flag == 1 && path_event1->laser.laser_amend_y_done_flag == 1 && ((schem_choose == 2 && sentinel_SENTINEL_UP < ATTACK_MIN_HP) || (schem_choose == 3 && sentinel_SENTINEL_UP < DEFENCE_MIN_HP)) && path_event1->path_ecd_set.behaviour_done_flag[path_event1->behaviour_count] == 1)
		{
			path_event1->path_ecd_set.delay_flag[path_event1->behaviour_count] = 1;
		}
	}
	else if(schem_choose == 2 && path_event1->path_ecd_set.laser_flag[path_event1->behaviour_count] == LASER_ENABLE && path_event1->behaviour_count == 5)
	{
		laser_locate(path_event1);
		//��λ��ɣ�������һ������������״̬����Ѫ�ﵽ�趨ֵ
		if(path_event1->laser.laser_amend_x_done_flag == 1 && path_event1->laser.laser_amend_y_done_flag == 1 && path_event1->path_ecd_set.behaviour_done_flag[path_event1->behaviour_count] == 1)
		{
			path_event1->path_ecd_set.delay_flag[path_event1->behaviour_count] = 1;
		}
	}
		//�������
	else if(path_event1->path_ecd_set.laser_flag[path_event1->behaviour_count] == LASER_ENABLE)
	{
		laser_locate(path_event1);
	}
}

fp32 debug_buff[10];

/**
   * @brief ���ⶨλ�������¼��ӳ�֮��
   * @param None.
   * @retval None.
   */
static void laser_locate(path_control_t * laser_locate)
{
	//�ж�ģʽ����ֹ��ʹ�ã��¼��ӳ��ҿ�������
	if(laser_locate->path_ecd_set.delay_mod[laser_locate->behaviour_count] == EVENT && laser_locate->path_ecd_set.laser_flag[laser_locate->behaviour_count] == LASER_ENABLE)
	{
		//����С����
		distance.spining_flag = 1;
		distance.spining_angle = 0.0f;
		
		//����յ������Դӻ��ļ����ź�
		if(laser_locate->laser.valid_flag == 1)
		{
			//�ϰ����־λ
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
			
			//����X����ʧ��
			if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X1_LASER || laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X2_LASER || (laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] >= DISABLE_X1_Y1_LASER))
			{
				barrier_flag_x = 0;
			}

			//����Y����ʧ��
			if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_Y1_LASER || laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_Y2_LASER || (laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] >= DISABLE_X1_Y1_LASER))
			{
				barrier_flag_y = 0;
			}

			if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == USE_ALL_LASER)
			{
				//����xƫ��
				laser_x_error = (-(laser_locate->laser.x1 + MACHINE_X / 2.0f - laser_locate->path_ecd_set.laser_x1_set[laser_locate->behaviour_count]) + (laser_locate->laser.x2 + MACHINE_X / 2.0f - laser_locate->path_ecd_set.laser_x2_set[laser_locate->behaviour_count])) / 2.0f;					
				//����yƫ��
				laser_y_error = ((laser_locate->laser.y1 + MACHINE_Y / 2.0f - laser_locate->path_ecd_set.laser_y1_set[laser_locate->behaviour_count]) - (laser_locate->laser.y2 + MACHINE_Y / 2.0f - laser_locate->path_ecd_set.laser_y2_set[laser_locate->behaviour_count])) / 2.0f;				
			}
			else if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X1_LASER)
			{
				//����xƫ��
				laser_x_error = (laser_locate->laser.x2 + MACHINE_X / 2.0f - laser_locate->path_ecd_set.laser_x2_set[laser_locate->behaviour_count]);					
				//����yƫ��
				laser_y_error = ((laser_locate->laser.y1 + MACHINE_Y / 2.0f - laser_locate->path_ecd_set.laser_y1_set[laser_locate->behaviour_count]) - (laser_locate->laser.y2 + MACHINE_Y / 2.0f - laser_locate->path_ecd_set.laser_y2_set[laser_locate->behaviour_count])) / 2.0f;					
			}
			else if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X2_LASER)
			{
				//����xƫ��
				laser_x_error = -(laser_locate->laser.x1 + MACHINE_X / 2.0f - laser_locate->path_ecd_set.laser_x1_set[laser_locate->behaviour_count]);					
				//����yƫ��
				laser_y_error = ((laser_locate->laser.y1 + MACHINE_Y / 2.0f - laser_locate->path_ecd_set.laser_y1_set[laser_locate->behaviour_count]) - (laser_locate->laser.y2 + MACHINE_Y / 2.0f - laser_locate->path_ecd_set.laser_y2_set[laser_locate->behaviour_count])) / 2.0f;								
			}
			else if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_Y1_LASER)
			{
				//����xƫ��
				laser_x_error = (-(laser_locate->laser.x1 + MACHINE_X / 2.0f - laser_locate->path_ecd_set.laser_x1_set[laser_locate->behaviour_count]) + (laser_locate->laser.x2 + MACHINE_X / 2.0f - laser_locate->path_ecd_set.laser_x2_set[laser_locate->behaviour_count])) / 2.0f;					
				//����yƫ��
				laser_y_error =  -(laser_locate->laser.y2 + MACHINE_Y / 2.0f - laser_locate->path_ecd_set.laser_y2_set[laser_locate->behaviour_count]);								
			}
			else if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_Y2_LASER)
			{
				//����xƫ��
				laser_x_error = (-(laser_locate->laser.x1 + MACHINE_X / 2.0f - laser_locate->path_ecd_set.laser_x1_set[laser_locate->behaviour_count]) + (laser_locate->laser.x2 + MACHINE_X / 2.0f - laser_locate->path_ecd_set.laser_x2_set[laser_locate->behaviour_count])) / 2.0f;					
				//����yƫ��
				laser_y_error = (laser_locate->laser.y1 + MACHINE_Y / 2.0f - laser_locate->path_ecd_set.laser_y1_set[laser_locate->behaviour_count]);								
			}	
			else if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X1_Y1_LASER)
			{
				//����xƫ��
				laser_x_error = (laser_locate->laser.x2 + MACHINE_X / 2.0f - laser_locate->path_ecd_set.laser_x2_set[laser_locate->behaviour_count]);					
				//����yƫ��
				laser_y_error = -(laser_locate->laser.y2 + MACHINE_Y / 2.0f - laser_locate->path_ecd_set.laser_y2_set[laser_locate->behaviour_count]);							
			}
			else if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X1_Y2_LASER)
			{
				//����xƫ��
				laser_x_error = (laser_locate->laser.x2 + MACHINE_X / 2.0f - laser_locate->path_ecd_set.laser_x2_set[laser_locate->behaviour_count]);					
				//����yƫ��
				laser_y_error = (laser_locate->laser.y1 + MACHINE_Y / 2.0f - laser_locate->path_ecd_set.laser_y1_set[laser_locate->behaviour_count]);						
			}
			else if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X2_Y1_LASER)
			{
				//����xƫ��
				laser_x_error = -(laser_locate->laser.x1 + MACHINE_X / 2.0f - laser_locate->path_ecd_set.laser_x1_set[laser_locate->behaviour_count]);
				//����yƫ��
				laser_y_error =  -(laser_locate->laser.y2 + MACHINE_Y / 2.0f - laser_locate->path_ecd_set.laser_y2_set[laser_locate->behaviour_count]);							
			}
			else if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X2_Y2_LASER)
			{
				//����xƫ��
				laser_x_error = -(laser_locate->laser.x1 + MACHINE_X / 2.0f - laser_locate->path_ecd_set.laser_x1_set[laser_locate->behaviour_count]);
				//����yƫ��
				laser_y_error = (laser_locate->laser.y1 + MACHINE_Y / 2.0f - laser_locate->path_ecd_set.laser_y1_set[laser_locate->behaviour_count]);								
			}
			else
			{
				distance.x = 0.0f;
				distance.y = 0.0f;
				return;
			}
			
			//û���趨���ȼ�������ֱ�ӽ���б������
			if(laser_locate->path_ecd_set.laser_amend_priority[laser_locate->behaviour_count] == NOT_SET_PRIORITY)
			{
				//��ʮ��ɨ�������ϰ���
				if(barrier_flag_x == 1 || barrier_flag_y == 1)
				{
					distance.x = 0.0f;
					distance.y = 0.0f;
					return;
				}

				//û���ϰ������PID����
				distance.y = PID_calc(&laser_locate->laser_speed_pid[0],0.0f,laser_x_error);
				distance.x = PID_calc(&laser_locate->laser_speed_pid[1],0.0f,laser_y_error);
				
				//�����Ƿ���ɱ�־λ
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
			//���Ƚ���X����
			else if(laser_locate->path_ecd_set.laser_amend_priority[laser_locate->behaviour_count] == AMEND_X_PRIORITY)
			{
				//��x�������ϰ���
				if(barrier_flag_x == 1)
				{
					distance.x = 0.0f;
					distance.y = 0.0f;
					return;
				}		
				
				//û���ϰ������PID����
				distance.y = PID_calc(&laser_locate->laser_speed_pid[0],0.0f,laser_x_error);
				
				//�����Ƿ���ɱ�־λ
				if(path_abs(laser_x_error) < LASER_ERROR_X)
				{
					laser_locate->laser.laser_amend_x_done_flag = 1;
				}
				else
				{
					laser_locate->laser.laser_amend_x_done_flag = 0;
				}
				
				//��x������ɣ�����y����
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
			//���Ƚ���Y����
			else if(laser_locate->path_ecd_set.laser_amend_priority[laser_locate->behaviour_count] == AMEND_Y_PRIORITY)
			{
				//��y�������ϰ���
				if(barrier_flag_y == 1)
				{
					distance.x = 0.0f;
					distance.y = 0.0f;
					return;
				}
				
				//û���ϰ������PID����
				distance.x = PID_calc(&laser_locate->laser_speed_pid[1],0.0f,laser_y_error);
				
				//�����Ƿ���ɱ�־λ
				if(path_abs(laser_y_error) < LASER_ERROR_Y)
				{
					laser_locate->laser.laser_amend_y_done_flag = 1;
				}
				else
				{
					laser_locate->laser.laser_amend_y_done_flag = 0;
				}
				
				//��y������ɣ�����x����
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
			
			//����һЩǰһʱ�̵ı���
			static uint8_t laser_amend_x_done_last_flag = 0;
			static uint8_t laser_amend_y_done_last_flag = 0;
			static uint8_t laser_amend_start_flag = 0;
			static uint8_t laser_x_flag = 0;
			static uint8_t laser_y_flag = 0;
			static uint8_t laser_last_behaviour = 0;
			
			//����һЩ����
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
			//�����X��������
			if(laser_amend_x_done_last_flag != laser_locate->laser.laser_amend_x_done_flag && laser_locate->laser.laser_amend_x_done_flag == 1)
			{
				laser_x_flag = 1;
			}
			else if(laser_locate->laser.laser_amend_x_done_flag == 0)
			{
				laser_x_flag = 0;
			}
				debug_buff[0] = laser_amend_x_done_last_flag;
			//�����Y��������
			if(laser_amend_y_done_last_flag != laser_locate->laser.laser_amend_y_done_flag && laser_locate->laser.laser_amend_y_done_flag == 1)
			{
				laser_y_flag = 1;
			}
			else if(laser_locate->laser.laser_amend_y_done_flag == 0)
			{
				laser_y_flag = 0;
			}
			
			//���ݼ�����ɳ̶Ȳ�ͬ����start��־λ���и�ֵ
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
			
			//��¼һ�³�ʼ��ֵ
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
			
			//���¼���ݵ�ƽ��ֵ����¼��laser_last�ṹ����
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
			//���ڣ���¼���˸տ�ʼ�����ݣ��ô���yawƯ����
			//x��¼���
			if((record_done == 1 || record_done == 3) && laser_locate->laser.laser_amend_x_done_flag == 1)
			{
				//ʧ����X1
				if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X1_LASER || laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X1_Y1_LASER || laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X1_Y2_LASER)
				{
					//��ֵ����Ϊdelta��Ĭ��90��ֱ�ǣ�����Ϊ��
//					amend_yaw += atan2(laser_locate->laser.x2 - laser_locate->laser_last.x2, laser_locate->laser_last.x2) * yaw_offset_plus;
					amend_yaw = (laser_locate->laser.x2 - laser_locate->laser_last.x2) * yaw_offset_plus;
				}
				//ʧ����X2
				else if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X2_LASER || laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X2_Y1_LASER || laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X2_Y2_LASER)
				{
//					amend_yaw += atan2(laser_locate->laser.x1 - laser_locate->laser_last.x1, laser_locate->laser_last.x1) * yaw_offset_plus;
					amend_yaw = (laser_locate->laser.x1 - laser_locate->laser_last.x1) * yaw_offset_plus;
				}
				//û��ʧ��
				else
				{
//					amend_yaw += (atan2(laser_locate->laser.x2 - laser_locate->laser_last.x2, laser_locate->laser_last.x2) * yaw_offset_plus + atan2(laser_locate->laser.x1 - laser_locate->laser_last.x1, laser_locate->laser_last.x1) * yaw_offset_plus) / 2.0f;
						amend_yaw = (laser_locate->laser.x2 - laser_locate->laser_last.x2) * yaw_offset_plus + (laser_locate->laser.x1 - laser_locate->laser_last.x1) * yaw_offset_plus ;
				}				
			}
			if((record_done == 2 || record_done == 3) && laser_locate->laser.laser_amend_y_done_flag == 1)	
			{
				//ʧ����Y1
				if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_Y1_LASER || laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X1_Y1_LASER || laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X2_Y1_LASER)
				{
//						amend_yaw += atan2(laser_locate->laser.y2 - laser_locate->laser_last.y2, laser_locate->laser_last.y2) * yaw_offset_plus;
					amend_yaw += (laser_locate->laser.y2 - laser_locate->laser_last.y2) * yaw_offset_plus;
				}
				//ʧ����Y2
				else if(laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_Y2_LASER || laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X1_Y2_LASER || laser_locate->path_ecd_set.laser_disable[laser_locate->behaviour_count] == DISABLE_X2_Y2_LASER)
				{
//						amend_yaw += atan2(laser_locate->laser.y1 - laser_locate->laser_last.y1, laser_locate->laser_last.y1) * yaw_offset_plus;
					amend_yaw += (laser_locate->laser.y1 - laser_locate->laser_last.y1) * yaw_offset_plus;
				}
				//û��ʧ��
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
		
		//������ɣ�ԭ������
		if(laser_locate->laser.laser_amend_x_done_flag == 1 && laser_locate->laser.laser_amend_y_done_flag == 1)
		{
			distance.x = 0.0f;
			distance.y = 0.0f;
		}
	}
}

/**
   * @brief ��ʱ�����������
   * @param �ṹ�壬������ţ�������ֵ���Ƕ�ֵ��ģʽ����(LINE OR AROUND)����ʱʱ�䣬�Ƿ����ɨ��ģʽ
   * @retval None.
   */
static void path_not_event_set(path_control_t * not_event_set, uint16_t behaviour_num ,fp32 ecd_set, fp32 angle_set, uint8_t mod_set ,uint32_t delay_time,uint8_t disable_yaw_scan,	fp32 scan_angle_set)
{
	//��ֹ����������µ�����Խ��
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
   * @brief �¼��ӳٿ�������
   * @param �ṹ�壬������ţ�������ֵ���Ƕ�ֵ��ģʽ����(LINE OR AROUND)���Ƿ������⣬�����߼����ȼ��������Ƿ�ֲ�ʧ�ܣ��ĸ���λ��x1��x2��y1��y2
   * @retval None.
   */
static void path_event_set(path_control_t * event_set, uint16_t behaviour_num ,fp32 ecd_set, fp32 angle_set, uint8_t mod_set, uint8_t laser_flag,uint8_t amend_priority, uint8_t disable, fp32 x1_set, fp32 x2_set, fp32 y1_set, fp32 y2_set)
{
	//��ֹ����������µ�����Խ��
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
   * @brief ·������
	 * @param �ṹ�壬·����ţ���ʼ����������������ѭ���������Ƿ�Ϊ����·��(0������,1:��)
   * @retval None.
   */
//PS:ֻ���ڽ���·��ĩβ������ɺ��ڱ��Ż���в�Ѫ/���ĩβ��������Ϊ���⣬ʱ���ӳ٣���delay_flag����Ϊ1�Լ�ѭ������Ϊ1�����������л�����һ��·�����Լ�ȷ�������һ������
static void path_cir_set(path_control_t * path_cir, uint8_t cir_num, uint8_t cir_min, uint8_t cir_max, uint16_t cir_time, uint8_t end_flag)
{
	//��ֹ��������µ�����Խ��
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
   * @brief ���⶯��������RM�����У�λ�ڻ�Ѫ�㲹Ѫʱ������������ǹ�ڳ�һ������
   * @param ...
   * @retval None.
   */
static void path_special_set(path_control_t * path_special, uint16_t behaviour_num, uint32_t delay_time,fp32 scan_angle_set)
{
	//��ֹ�����õ��µ�����Խ��
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
   * @brief ��Ѫ/�ظ�·������
   * @param �ṹ�壬��ʼ��������������
   * @retval None.
   */
static void restore_cir_set(path_control_t * path_restore, uint8_t cir_min, uint8_t cir_max)
{
	path_restore->cir.restore_cir_min = cir_min;
	path_restore->cir.restore_cir_max = cir_max;
}

/**
   * @brief ��Ѫ/������
   * @param �ṹ��
   * @retval None.
   */
static void restore_detect(path_control_t * restore_detect)
{
	static uint8_t restore_detect_flag = 0;
	//attack
	if(schem_choose == 0 || schem_choose == 2 || schem_choose == 3)
	{
		//Ϊ����·���������������Ҽ���������ϣ������߼������˿϶���С��300Ѫ�� / 200Ѫ
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

/// @brief ������ת��Ϊ�ֽ��� IEEE754
/// @param floatNum ��Ҫת���ĸ�����
/// @param byteArry ���ڴ洢�ֽ����ĳ���Ϊ4���ֽ�����
static void FloatToByte(float floatNum, uint8_t *byteArry)
{
  uint8_t *pchar = (uint8_t *)&floatNum;
  for (int i = 0; i < sizeof(float); i++)
  {
    byteArry[3 - i] = *pchar;
    pchar++;
  }
}

/// @brief ���ֽ���ת��Ϊ������ IEEE754
/// @param Byte �ֽ�����
/// @param num �ֽ�������Ϊת��Ϊfloat��һ��������ʹ��sizeof(float)
/// @return ����ת����ĸ�����
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

/// @brief crc8У���㷨
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
   * @brief �Լ���������STM32F407ZG��������
   * @param None.
   * @retval None.
   */
static void send_laser_data()
{
	divide_count++;
	//����Ƶ�ﵽ����ʱ����������SET_COUNT��������100hz����ֹ����Ƶ�ʹ���
	if(divide_count == SET_COUNT)
	{
		divide_count = 0;
		
		//ͷ֡
		chassis_buff[0] = SEND_EOF;
		
		//��������
		FloatToByte(rad_format(chassis_move.chassis_INS_angle[0] + path_control.yaw_offset),&chassis_buff[1]);
		
		//crcУ��
		chassis_buff[5] = crc8(&chassis_buff[0],5);
		
		//β֡
		chassis_buff[6] = SEND_NOF;
		
		HAL_UART_Transmit(&huart1, chassis_buff, CHASSIS_SEND_DATA,1000);
	}
}

/**
   * @brief ������������STM32F407ZG����
   * @param None.
   * @retval None.
   */
void accept_laser_data()
{
	//ͷ֡��β֡�Լ�CRCУ��
		if(laser_buff[0] == 0x5A && crc8(&laser_buff[0],17) == laser_buff[17] && laser_buff[18] == 0xFF)
		{
			//У�����������ݽ��н��㴦��
			path_control.laser.x1 = Hex_To_Decimal(&laser_buff[1],4);
			path_control.laser.x2 = Hex_To_Decimal(&laser_buff[5],4);
			path_control.laser.y1 = Hex_To_Decimal(&laser_buff[9],4);
			path_control.laser.y2 = Hex_To_Decimal(&laser_buff[13],4);
			path_control.laser.valid_flag = 1;
		}
}

/**
   * @brief ����1�жϣ��������Լ���������STM32F407ZG����
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
			temp = huart1.Instance->SR;   	//�����־λ									
			temp = huart1.Instance->DR; 						
			HAL_UART_DMAStop(&huart1);   									
			temp = hdma_usart1_rx.Instance->NDTR; 	  		
			accept_laser_data();		
			   											
	}
	HAL_UART_Receive_DMA(&huart1,laser_buff,19); //ʹ��DMA

  HAL_UART_IRQHandler(&huart1);

}

