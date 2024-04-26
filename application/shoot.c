/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
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

#define shoot_fric1_on(pwm) fric1_on((pwm)) // 摩擦轮1pwm宏定义
#define shoot_fric2_on(pwm) fric2_on((pwm)) // 摩擦轮2pwm宏定义
#define shoot_fric_off() fric_off()         // 关闭两个摩擦轮

#define shoot_laser_on() laser_on()   // 激光开启宏定义
#define shoot_laser_off() laser_off() // 激光关闭宏定义
// 微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

/**
 * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
 * @param[in]      void
 * @retval         void
 */
static void shoot_set_mode(void);
/**
 * @brief          射击数据更新
 * @param[in]      void
 * @retval         void
 */
static void shoot_feedback_update(void);

/**
 * @brief          堵转倒转处理
 * @param[in]      void
 * @retval         void
 */
static void trigger_motor_turn_back(void);

/**
 * @brief          射击控制，控制拨弹电机角度，完成一次发射
 * @param[in]      void
 * @retval         void
 */
static void shoot_bullet_control(void);

shoot_control_t shoot_control; // 射击数据

unsigned int shoot_init_flag = 0;
unsigned int shoot_last_time_record_shoot = 0;
unsigned int shoot_last_time_record_shoot_ready = 0;

/**
 * @brief          射击初始化，初始化PID，遥控器指针，电机指针
 * @param[in]      void
 * @retval         返回空
 */
void shoot_init(void)
{
    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
		static const fp32 fric_speed_pid[3] = {FRIC_SPEED_PID_KP, FRIC_SPEED_PID_KI, FRIC_SPEED_PID_KD};
    shoot_control.shoot_mode = SHOOT_STOP;
    // 遥控器指针
    shoot_control.shoot_rc = get_remote_control_point();
    // 电机指针
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
		//摩擦轮指针
		shoot_control.fric_motor_measuer[0] = get_fric1_motor_measure_point();
		shoot_control.fric_motor_measuer[1] = get_fric2_motor_measure_point();
    // 初始化PID
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
		 //枪管初始化
		gun_change_init(&gun_cmd);
		//初始化摩擦轮PID
	uint8_t i = 0;
	for(i = 0; i < 2; i++)
	{		
		PID_init(&shoot_control.fric_motor_pid[i],PID_POSITION,fric_speed_pid,FRIC_SPEED_PID_MAX_OUT,FRIC_SPEED_PID_MAX_IOUT);
	}
		
    // 更新数据
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
 * @brief          射击循环
 * @param[in]      void
 * @retval         返回can控制值
 */
int16_t shoot_control_loop(void)
{

	shoot_set_mode();		 // 设置状态机
	shoot_feedback_update(); // 更新数据

	if (shoot_control.shoot_mode == SHOOT_STOP)
	{
		// 设置拨弹轮的速度
		shoot_control.speed_set = 0.0f;
	}
	else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
	{
		// 设置拨弹轮的速度
		shoot_control.speed_set = 0.0f;
	}
	else if (shoot_control.shoot_mode == SHOOT_READY_BULLET)
	{
//		if (shoot_control.key == SWITCH_TRIGGER_OFF)
//		{
//			// 设置拨弹轮的拨动速度,并开启堵转反转处理
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
		// 设置拨弹轮的速度
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
		// 设置拨弹轮的拨动速度,并开启堵转反转处理
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
		// 摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
		ramp_calc(&shoot_control.fric1_ramp, -SHOOT_FRIC_CMD_ADD_VALUE);
		ramp_calc(&shoot_control.fric2_ramp, -SHOOT_FRIC_CMD_ADD_VALUE);
	}
	else
	{
		shoot_laser_on(); // 激光开启
		// 计算拨弹轮电机PID
		PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
		shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
		if (shoot_control.shoot_mode < SHOOT_READY_BULLET)
		{
			shoot_control.given_current = 0;
		}
		// 摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
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
	//CAN问题，不得已而取绝对值，后续需要解决，并改成和摩擦轮1一样的形式
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
	// 枪管控制
	gun_control(&gun_cmd);

	CAN_cmd_fric(shoot_control.fric1_motor_current_set, shoot_control.fric2_motor_current_set);
//	CAN_cmd_fric(0, 0);
	// 枪管转化未完成
	if (gun_cmd.gun_done == 0)
	{
		shoot_control.given_current = 0;
	}
	return shoot_control.given_current;
}

/**
 * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
 * @param[in]      void
 * @retval         void
 */
static void shoot_set_mode(void)
{
      static int8_t last_s = RC_SW_UP;

    //    //上拨判断， 一次开启，再次关闭
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
            //下拨一次或者鼠标按下一次，进入射击状态
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
            //鼠标长按一直进入射击状态 保持连发
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
			if (CV_EV.state == 0x02) //大于两秒重置，防止卡弹 
			{
					shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
			}
			else if(CV_EV.state != 0x02) //大于一秒重置
			{
					shoot_control.shoot_mode = SHOOT_READY_BULLET;
			}
			
		}
		
    last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}
/**
 * @brief          射击数据更新
 * @param[in]      void
 * @retval         void
 */
static void shoot_feedback_update(void)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    // 拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    // 二阶低通滤波
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;

    // 电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
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

    // 计算输出轴角度
    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
    // 微动开关
    shoot_control.key = SWITCH_TRIGGER_ON;
    // 鼠标按键
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
    // 长按计时
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

    // 射击开关下档时间计时
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

    // 鼠标右键按下加速摩擦轮，使得左键低速射击， 右键高速射击
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
 * @brief          射击控制，控制拨弹电机角度，完成一次发射
 * @param[in]      void
 * @retval         void
 */
static void shoot_bullet_control(void)
{

    // 每次拨动 1/4PI的角度
    if (shoot_control.move_flag == 0)
    {
        shoot_control.set_angle = rad_format(shoot_control.angle + PI_TEN);
        shoot_control.move_flag = 1;
    }
    // 到达角度判断
    if (rad_format(shoot_control.set_angle - shoot_control.angle) > 0.05f)
    {
        // 没到达一直设置旋转速度
        shoot_control.trigger_speed_set = TRIGGER_SPEED;
        trigger_motor_turn_back();
				shoot_control.shoot_mode = SHOOT_DONE;
    }
    else
    {
        shoot_control.move_flag = 0;
    }
}
