/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      �������.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "shoot.h"
#include "main.h"

#include "cmsis_os.h"

#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"

#include "visual_task.h"

#include "fire_control_system.h"
#include "chassis_behaviour.h"
#include "chassis_task.h"

#define shoot_fric1_on(pwm) fric1_on((pwm)) // Ħ����1pwm�궨��
#define shoot_fric2_on(pwm) fric2_on((pwm)) // Ħ����2pwm�궨��
#define shoot_fric_off() fric_off()         // �ر�����Ħ����

#define shoot_laser_on() laser_on()   // ���⿪���궨��
#define shoot_laser_off() laser_off() // ����رպ궨��
// ΢������IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

/**
 * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
 * @param[in]      void
 * @retval         void
 */
static void shoot_set_mode(void);
/**
 * @brief          ������ݸ���
 * @param[in]      void
 * @retval         void
 */
static void shoot_feedback_update(void);

/**
 * @brief          ��ת��ת����
 * @param[in]      void
 * @retval         void
 */
static void trigger_motor_turn_back(void);

/**
 * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
 * @param[in]      void
 * @retval         void
 */
static void shoot_bullet_control(void);

shoot_control_t shoot_control; // �������

unsigned int shoot_init_flag = 0;
unsigned int shoot_last_time_record_shoot = 0;
unsigned int shoot_last_time_record_shoot_ready = 0;

/**
 * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
 * @param[in]      void
 * @retval         ���ؿ�
 */
void shoot_init(void)
{
    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
		static const fp32 fric_speed_pid[3] = {FRIC_SPEED_PID_KP, FRIC_SPEED_PID_KI, FRIC_SPEED_PID_KD};
    shoot_control.shoot_mode = SHOOT_STOP;
    // ң����ָ��
    shoot_control.shoot_rc = get_remote_control_point();
    // ���ָ��
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
		//Ħ����ָ��
		shoot_control.fric_motor_measuer[0] = get_fric1_motor_measure_point();
		shoot_control.fric_motor_measuer[1] = get_fric2_motor_measure_point();
    // ��ʼ��PID
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
		 //ǹ�ܳ�ʼ��
		gun_change_init(&gun_cmd);
		//��ʼ��Ħ����PID
	uint8_t i = 0;
	for(i = 0; i < 2; i++)
	{		
		PID_init(&shoot_control.fric_motor_pid[i],PID_POSITION,fric_speed_pid,FRIC_SPEED_PID_MAX_OUT,FRIC_SPEED_PID_MAX_IOUT);
	}
		
    // ��������
    shoot_feedback_update();
	shoot_control.fric1_speed_set = 0.0f;
	shoot_control.fric2_speed_set = 0.0f;
    ramp_init(&shoot_control.fric1_ramp, SHOOT_CONTROL_TIME * 0.01f, FRIC_NORMAL_SPEED, 0);
    ramp_init(&shoot_control.fric2_ramp, SHOOT_CONTROL_TIME * 0.01f, FRIC_NORMAL_SPEED, 0);
    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.set_angle = shoot_control.angle;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;
	
}

/**
 * @brief          ���ѭ��
 * @param[in]      void
 * @retval         ����can����ֵ
 */
int16_t shoot_control_loop(void)
{

	shoot_set_mode();		 // ����״̬��
	shoot_feedback_update(); // ��������

	if (shoot_control.shoot_mode == SHOOT_STOP)
	{
		// ���ò����ֵ��ٶ�
		shoot_control.speed_set = 0.0f;
	}
	else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
	{
		// ���ò����ֵ��ٶ�
		shoot_control.speed_set = 0.0f;
	}
	else if (shoot_control.shoot_mode == SHOOT_READY_BULLET)
	{
//		if (shoot_control.key == SWITCH_TRIGGER_OFF)
//		{
//			// ���ò����ֵĲ����ٶ�,��������ת��ת����
//			shoot_control.trigger_speed_set = 0.0f;
//			trigger_motor_turn_back();
//		}
//		else
//		{
			shoot_control.trigger_speed_set = 0.0f;
			shoot_control.speed_set = 0.0f;            
//		}
		shoot_control.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
		shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
	}
	else if (shoot_control.shoot_mode == SHOOT_READY)
	{
		// ���ò����ֵ��ٶ�
		shoot_control.speed_set = 0.0f;
	}
	else if (shoot_control.shoot_mode == SHOOT_BULLET)
	{
		shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
		shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
		shoot_bullet_control();
	}
	else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
	{
		// ���ò����ֵĲ����ٶ�,��������ת��ת����
		shoot_control.trigger_speed_set = CONTINUE_TRIGGER_SPEED;
		trigger_motor_turn_back();
	}
	else if (shoot_control.shoot_mode == SHOOT_DONE)
	{
		shoot_control.speed_set = 0.0f;
	}

//	last_speed_rpm = shoot_control.fric_motor_measuer[1]->speed_rpm;
	if (shoot_control.shoot_mode == SHOOT_STOP)
	{
		shoot_laser_off();
		shoot_control.given_current = 0;
		// Ħ������Ҫһ����б������������ͬʱֱ�ӿ�����������ܵ����ת
		ramp_calc(&shoot_control.fric1_ramp, -SHOOT_FRIC_CMD_ADD_VALUE);
		ramp_calc(&shoot_control.fric2_ramp, -SHOOT_FRIC_CMD_ADD_VALUE);
	}
	else
	{
		shoot_laser_on(); // ���⿪��
		// ���㲦���ֵ��PID
		PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
		shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
		if (shoot_control.shoot_mode < SHOOT_READY_BULLET)
		{
			shoot_control.given_current = 0;
		}
		// Ħ������Ҫһ����б������������ͬʱֱ�ӿ�����������ܵ����ת
		ramp_calc(&shoot_control.fric1_ramp, SHOOT_FRIC_CMD_ADD_VALUE);
		ramp_calc(&shoot_control.fric2_ramp, SHOOT_FRIC_CMD_ADD_VALUE);
	}

	shoot_control.fric1_speed_set = (uint16_t)(shoot_control.fric1_ramp.out);
	shoot_control.fric2_speed_set = -(uint16_t)(shoot_control.fric2_ramp.out);

	if(shoot_control.fric1_speed_set == 0)
	{
		shoot_control.fric1_motor_current_set = 0;
	}
	else
	{
		shoot_control.fric1_motor_current_set =
		(int16_t)PID_calc(&shoot_control.fric_motor_pid[0],
						  shoot_control.fric_motor_measuer[0]->speed_rpm,
						  shoot_control.fric1_speed_set);
	}
	//CAN���⣬�����Ѷ�ȡ����ֵ��������Ҫ��������ĳɺ�Ħ����1һ������ʽ
	if(shoot_control.fric2_speed_set == 0)
	{
		shoot_control.fric2_motor_current_set = 0;
	}
	else
	{
		shoot_control.fric2_motor_current_set =
			(int16_t)PID_calc(&shoot_control.fric_motor_pid[1],
							  -PC_ABS(shoot_control.fric_motor_measuer[1]->speed_rpm),
							  shoot_control.fric2_speed_set);
	}
	// ǹ�ܿ���
	gun_control(&gun_cmd);

	CAN_cmd_fric(shoot_control.fric1_motor_current_set, shoot_control.fric2_motor_current_set);
//	CAN_cmd_fric(0, 0);
	// ǹ��ת��δ���
	if (gun_cmd.gun_done == 0)
	{
		shoot_control.given_current = 0;
	}
	return shoot_control.given_current;
}

/**
 * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
 * @param[in]      void
 * @retval         void
 */
static void shoot_set_mode(void)
{
      static int8_t last_s = RC_SW_UP;

    //    //�ϲ��жϣ� һ�ο������ٴιر�
	    if(sentinel_game_state == NOT_START_GAME && chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
			{
				if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode == SHOOT_STOP))
				{
						shoot_control.shoot_mode = SHOOT_READY_FRIC;
				}
				else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode != SHOOT_STOP))
				{
						shoot_control.shoot_mode = SHOOT_STOP;
				}
		    if(shoot_control.shoot_mode == SHOOT_READY_FRIC && shoot_control.fric1_ramp.out == shoot_control.fric1_ramp.max_value && shoot_control.fric2_ramp.out == shoot_control.fric2_ramp.max_value)
        {
            shoot_control.shoot_mode = SHOOT_READY_BULLET;
        }
        else if(shoot_control.shoot_mode == SHOOT_READY_BULLET && shoot_control.key == SWITCH_TRIGGER_ON)
        {
            shoot_control.shoot_mode = SHOOT_READY;
        }
        else if(shoot_control.shoot_mode == SHOOT_READY && shoot_control.key == SWITCH_TRIGGER_OFF)
        {
            shoot_control.shoot_mode = SHOOT_READY_BULLET;
        }
        else if(shoot_control.shoot_mode == SHOOT_READY)
        {
            //�²�һ�λ�����갴��һ�Σ��������״̬
            if ((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)) || (shoot_control.press_l && shoot_control.last_press_l == 0) || (shoot_control.press_r && shoot_control.last_press_r == 0))
            {
                shoot_control.shoot_mode = SHOOT_BULLET;
            }
        }
        else if(shoot_control.shoot_mode == SHOOT_DONE)
        {
            if(shoot_control.key == SWITCH_TRIGGER_OFF)
            {
                shoot_control.key_time++;
                if(shoot_control.key_time > SHOOT_DONE_KEY_OFF_TIME)
                {
                    shoot_control.key_time = 0;
                   shoot_control.shoot_mode = SHOOT_READY_BULLET;
                 }
            }
            else
            {
                shoot_control.key_time = 0;
                shoot_control.shoot_mode = SHOOT_BULLET;
            }
        }
    

        if(shoot_control.shoot_mode > SHOOT_READY_FRIC)
        {
            //��곤��һֱ�������״̬ ��������
            if ((shoot_control.press_l_time == PRESS_LONG_TIME) || (shoot_control.press_r_time == PRESS_LONG_TIME) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
            {
                shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
            }
            else if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
            {
                shoot_control.shoot_mode = SHOOT_READY_BULLET;
            }
        }

        get_shoot_heat0_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);
//        if(!toe_is_error(REFEREE_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.heat_limit))
//        {
//            if(shoot_control.shoot_mode == SHOOT_BULLET || shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
//          {
//              shoot_control.shoot_mode = SHOOT_READY_BULLET;
//          }
//        }
			}

	  if(sentinel_game_state == NOT_START_GAME && chassis_behaviour_mode == CHASSIS_AUTO_SENTINEL)
		{
			shoot_control.shoot_mode = SHOOT_STOP;
		}
		else if(sentinel_game_state == START_GAME && shoot_init_flag < SET_SHOOT_INIT_TIME && chassis_behaviour_mode == CHASSIS_AUTO_SENTINEL)
		{
			shoot_control.shoot_mode = SHOOT_READY_FRIC;
			shoot_init_flag++;
		}

		if(sentinel_game_state == START_GAME)
		{
			if (CV_EV.state == 0x02) //�����������ã���ֹ���� 
			{
					shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
			}
			else if(CV_EV.state != 0x02) //����һ������
			{
					shoot_control.shoot_mode = SHOOT_READY_BULLET;
			}
			
		}
		
    last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}
/**
 * @brief          ������ݸ���
 * @param[in]      void
 * @retval         void
 */
static void shoot_feedback_update(void)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    // �����ֵ���ٶ��˲�һ��
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    // ���׵�ͨ�˲�
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;

    // ���Ȧ�����ã� ��Ϊ�������תһȦ�� �������ת 36Ȧ������������ݴ������������ݣ����ڿ��������Ƕ�
    if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        shoot_control.ecd_count--;
    }
    else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        shoot_control.ecd_count++;
    }

    if (shoot_control.ecd_count == FULL_COUNT)
    {
        shoot_control.ecd_count = -(FULL_COUNT - 1);
    }
    else if (shoot_control.ecd_count == -FULL_COUNT)
    {
        shoot_control.ecd_count = FULL_COUNT - 1;
    }

    // ���������Ƕ�
    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
    // ΢������
    shoot_control.key = SWITCH_TRIGGER_ON;
    // ��갴��
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
    // ������ʱ
    if (shoot_control.press_l)
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME)
        {
            shoot_control.press_l_time++;
        }
    }
    else
    {
        shoot_control.press_l_time = 0;
    }

    if (shoot_control.press_r)
    {
        if (shoot_control.press_r_time < PRESS_LONG_TIME)
        {
            shoot_control.press_r_time++;
        }
    }
    else
    {
        shoot_control.press_r_time = 0;
    }

    // ��������µ�ʱ���ʱ
    if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {

        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_control.rc_s_time++;
        }
    }
    else
    {
        shoot_control.rc_s_time = 0;
    }

    // ����Ҽ����¼���Ħ���֣�ʹ�������������� �Ҽ��������
    static uint16_t up_time = 0;
    if (shoot_control.press_r)
    {
        up_time = UP_ADD_TIME;
    }

    if (up_time > 0)
    {
        shoot_control.fric1_ramp.max_value = FRIC_NORMAL_SPEED;
        shoot_control.fric2_ramp.max_value = FRIC_NORMAL_SPEED;
        up_time--;
    }
    else
    {
        shoot_control.fric1_ramp.max_value = FRIC_NORMAL_SPEED;
        shoot_control.fric2_ramp.max_value = FRIC_NORMAL_SPEED;
    }
}

static void trigger_motor_turn_back(void)
{
    if (shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.speed_set = shoot_control.trigger_speed_set;
    }
    else
    {
        shoot_control.speed_set = -shoot_control.trigger_speed_set;
    }

    if (fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.block_time++;
        shoot_control.reverse_time = 0;
    }
    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time++;
    }
    else
    {
				shoot_control.trigger_motor_pid.Iout = 0.0f;
        shoot_control.block_time = 0;
    }
}

/**
 * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
 * @param[in]      void
 * @retval         void
 */
static void shoot_bullet_control(void)
{

    // ÿ�β��� 1/4PI�ĽǶ�
    if (shoot_control.move_flag == 0)
    {
        shoot_control.set_angle = rad_format(shoot_control.angle + PI_TEN);
        shoot_control.move_flag = 1;
    }
    // ����Ƕ��ж�
    if (rad_format(shoot_control.set_angle - shoot_control.angle) > 0.05f)
    {
        // û����һֱ������ת�ٶ�
        shoot_control.trigger_speed_set = TRIGGER_SPEED;
        trigger_motor_turn_back();
				shoot_control.shoot_mode = SHOOT_DONE;
    }
    else
    {
        shoot_control.move_flag = 0;
    }
}
