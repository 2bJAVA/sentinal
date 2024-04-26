/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
	*  V2.0.0     2023-9-18       天衍             4.完善，修改，整理  
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_task.h"

#include "main.h"

#include "cmsis_os.h"
#include <math.h>
#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "pid.h"
#include "gimbal_attitude.h"
#include "shoot.h"
#include "attitude_task.h"
#include "visual_task.h"
#include "fire_control_system.h"
#include "path_task.h"

// motor enconde value format, range[0-8191]
// 电机编码值规整 0—8191
#define ecd_format(ecd)    \
  {                        \
    if ((ecd) > ECD_RANGE) \
      (ecd) -= ECD_RANGE;  \
    else if ((ecd) < 0)    \
      (ecd) += ECD_RANGE;  \
  }

#define gimbal_total_pid_clear(gimbal_clear)                                               \
  {                                                                                        \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
    PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
                                                                                           \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
    PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
  }

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

float Gimbal_Vision_pid_calc(PID *pid, float now, float set);
float Gimbal_Math_pid_calc(PID *pid, float now, float set);
static void Gimbal_Vision_Control_Set(gimbal_control_t *Gimbal_Control_Set);
void Angle_Control_Limit(float *Angle_Set, float Max_Angle, float Min_Angle);
float Angle_Limit(float Angle_Set);

/**
 * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
 *                 data point initialization, and gyro sensor angle point initialization.
 * @param[out]     init: "gimbal_control" valiable point
 * @retval         none
 */
/**
 * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
 * @param[out]     init:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_init(gimbal_control_t *init);

/**
 * @brief
 *
 *
 * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
 * @retval         none
 */
/**
 * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
 * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_mode(gimbal_control_t *set_mode);
/**
 * @brief          gimbal some measure data updata, such as motor enconde, euler angle, gyro
 * @param[out]     gimbal_feedback_update: "gimbal_control" valiable point
 * @retval         none
 */
/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_feedback_update(gimbal_control_t *feedback_update);

/**
 * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
 * @param[out]     mode_change: "gimbal_control" valiable point
 * @retval         none
 */
/**
 * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
 * @param[out]     mode_change:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_mode_change_control_transit(gimbal_control_t *mode_change);

/**
 * @brief          calculate the relative angle between ecd and offset_ecd
 * @param[in]      ecd: motor now encode
 * @param[in]      offset_ecd: gimbal offset encode
 * @retval         relative angle, unit rad
 */
/**
 * @brief          计算ecd与offset_ecd之间的相对角度
 * @param[in]      ecd: 电机当前编码
 * @param[in]      offset_ecd: 电机中值编码
 * @retval         相对角度，单位rad
 */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
/**
 * @brief          set gimbal control set-point, control set-point is set by "gimbal_behaviour_control_set".
 * @param[out]     gimbal_set_control: "gimbal_control" valiable point
 * @retval         none
 */
/**
 * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
 * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_control(gimbal_control_t *set_control);


/**
 * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control.
 * @param[out]     gimbal_motor: yaw motor or pitch motor
 * @retval         none
 */
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);
/**
 * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode relative angle  to control.
 * @param[out]     gimbal_motor: yaw motor or pitch motor
 * @retval         none
 */
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);
/**
 * @brief          gimbal control mode :GIMBAL_MOTOR_RAW, current  is sent to CAN bus.
 * @param[out]     gimbal_motor: yaw motor or pitch motor
 * @retval         none
 */
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_RAW，电流值直接发送到CAN总线.
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor);
/**
 * @brief          limit angle set in GIMBAL_MOTOR_GYRO mode, avoid exceeding the max angle
 * @param[out]     gimbal_motor: yaw motor or pitch motor
 * @retval         none
 */
/**
 * @brief          在GIMBAL_MOTOR_GYRO模式，限制角度设定,防止超过最大
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
/**
 * @brief          limit angle set in GIMBAL_MOTOR_ENCONDE mode, avoid exceeding the max angle
 * @param[out]     gimbal_motor: yaw motor or pitch motor
 * @retval         none
 */
/**
 * @brief          在GIMBAL_MOTOR_ENCONDE模式，限制角度设定,防止超过最大
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);

/**
 * @brief          gimbal angle pid init, because angle is in range(-pi,pi),can't use PID in pid.c
 * @param[out]     pid: pid data pointer stucture
 * @param[in]      maxout: pid max out
 * @param[in]      intergral_limit: pid max iout
 * @param[in]      kp: pid kp
 * @param[in]      ki: pid ki
 * @param[in]      kd: pid kd
 * @retval         none
 */
/**
 * @brief          云台角度PID初始化, 因为角度范围在(-pi,pi)，不能用PID.c的PID
 * @param[out]     pid:云台PID指针
 * @param[in]      maxout: pid最大输出
 * @param[in]      intergral_limit: pid最大积分输出
 * @param[in]      kp: pid kp
 * @param[in]      ki: pid ki
 * @param[in]      kd: pid kd
 * @retval         none
 */
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 intergral_limit, fp32 kp, fp32 ki, fp32 kd);

/**
 * @brief          gimbal PID clear, clear pid.out, iout.
 * @param[out]     pid_clear: "gimbal_control" valiable point
 * @retval         none
 */
/**
 * @brief          云台PID清除，清除pid的out,iout
 * @param[out]     pid_clear:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_PID_clear(gimbal_PID_t *pid_clear);
/**
 * @brief          gimbal angle pid calc, because angle is in range(-pi,pi),can't use PID in pid.c
 * @param[out]     pid: pid data pointer stucture
 * @param[in]      get: angle feeback
 * @param[in]      set: angle set-point
 * @param[in]      error_delta: rotation speed
 * @retval         pid out
 */
/**
 * @brief          云台角度PID计算, 因为角度范围在(-pi,pi)，不能用PID.c的PID
 * @param[out]     pid:云台PID指针
 * @param[in]      get: 角度反馈
 * @param[in]      set: 角度设定
 * @param[in]      error_delta: 角速度
 * @retval         pid 输出
 */
static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);

/**
 * @brief          gimbal calibration calculate
 * @param[in]      gimbal_cali: cali data
 * @param[out]     yaw_offset:yaw motor middle place encode
 * @param[out]     pitch_offset:pitch motor middle place encode
 * @param[out]     max_yaw:yaw motor max machine angle
 * @param[out]     min_yaw: yaw motor min machine angle
 * @param[out]     max_pitch: pitch motor max machine angle
 * @param[out]     min_pitch: pitch motor min machine angle
 * @retval         none
 */
/**
 * @brief          云台校准计算
 * @param[in]      gimbal_cali: 校准数据
 * @param[out]     yaw_offset:yaw电机云台中值
 * @param[out]     pitch_offset:pitch 电机云台中值
 * @param[out]     max_yaw:yaw 电机最大机械角度
 * @param[out]     min_yaw: yaw 电机最小机械角度
 * @param[out]     max_pitch: pitch 电机最大机械角度
 * @param[out]     min_pitch: pitch 电机最小机械角度
 * @retval         none
 */
static void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);

/**
 * @brief 扫描模式下pitch轴锁定(LOCK)
 * @param pitch轴
 * @retval None.
 */
static void gimbal_scan_relative_angle_limit(gimbal_motor_t *gimbal_motor);

/**
   * @brief 扫描模式PID控制
   * @param yaw轴速度,pitch轴速度
   * @retval None.
   */
static void gimbal_scan_pid_control(fp32 yaw_speed , fp32 pitch_speed);

/**
   * @brief 步兵模式下的云台操控
   * @param add_yaw,add_pitch
   * @retval None.
   */
static void Gimbal_Infantry_Control_Set(gimbal_control_t *Gimbal_Control_Set,float add_yaw,float add_pitch);

#if GIMBAL_TEST_MODE
// j-scope 帮助pid调参
static void J_scope_gimbal_test(void);
#endif

// gimbal control data
// 云台控制所有相关数据
gimbal_control_t gimbal_control;

// motor current
// 发送的电机电流
static int16_t yaw_can_set_current = 0, pitch_can_set_current = 0;

static int16_t shoot_can_set_current = 0;

int16_t debug_shoot_can_set_current = 0;

Typedef_SENTINEL_MOD LAST_SENTINEL_GIMBAL_MOD = GIMBAL_NONE;


/**
 * @brief          gimbal task, osDelay GIMBAL_CONTROL_TIME (1ms)
 * @param[in]      pvParameters: null
 * @retval         none
 */
/**
 * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void gimbal_task(void const *pvParameters)
{
  // 等待陀螺仪任务更新陀螺仪数据
  // wait a time
  vTaskDelay(GIMBAL_TASK_INIT_TIME);
  // gimbal init
  // 云台初始化
  gimbal_init(&gimbal_control);

  // wait for all motor online
  // 判断电机是否都上线
  while (toe_is_error(YAW_GIMBAL_MOTOR_TOE) || toe_is_error(PITCH_GIMBAL_MOTOR_TOE))
  {
    vTaskDelay(GIMBAL_CONTROL_TIME);
    gimbal_feedback_update(&gimbal_control); // 云台数据反馈
  }

  while (1)
  {
    gimbal_set_mode(&gimbal_control);                    // 设置云台控制模式
    gimbal_mode_change_control_transit(&gimbal_control); // 控制模式切换 控制数据过渡
    gimbal_feedback_update(&gimbal_control);             // 云台数据反馈
    gimbal_set_control(&gimbal_control);                 // 设置云台控制量
    shoot_can_set_current = shoot_control_loop();
		debug_shoot_can_set_current = shoot_can_set_current;
#if YAW_TURN
    yaw_can_set_current = -gimbal_control.gimbal_yaw_motor.given_current;
#else
    yaw_can_set_current = gimbal_control.gimbal_yaw_motor.given_current;
#endif

#if PITCH_TURN
    pitch_can_set_current = -gimbal_control.gimbal_pitch_motor.given_current;
#else
    pitch_can_set_current = gimbal_control.gimbal_pitch_motor.given_current;
#endif

    if((sentinel_game_state == NOT_START_GAME && gimbal_behaviour == GIMBAL_AUTO_SENTINEL))
		{
			 if (sentinel_gimbal_behaviour == GIMBAL_TRACK_ENEMY_MOD)
			 {
				CAN_cmd_gimbal(yaw_can_set_current, pitch_can_set_current, 0, gun_cmd.give_current);
			 }
			 else
			 {
//				 CAN_cmd_gimbal(0, 0, 0, gun_cmd.give_current);
				 CAN_cmd_gimbal(0,0,0,0);
			 }
		}
		else if(sentinel_game_state == NOT_START_GAME && gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
		{
				CAN_cmd_gimbal(yaw_can_set_current, 0, 0, 0);
//				CAN_cmd_gimbal(yaw_can_set_current, pitch_can_set_current, shoot_can_set_current, 0);

			}
		else if(sentinel_game_state == START_GAME)
		{
			if (sentinel_gimbal_behaviour == GIMBAL_TRACK_ENEMY_MOD)
			{
//			 CAN_cmd_gimbal(0,0,0,0);
				CAN_cmd_gimbal(yaw_can_set_current,pitch_can_set_current,shoot_can_set_current, 0);

			}
			if (sentinel_gimbal_behaviour == GIMBAL_SCAN_MOD)
			{
				CAN_cmd_gimbal(0, 0, 0, 0);	
//				CAN_cmd_gimbal(0, 0,0,0);//yaw_can_set_current
			}
		}
#if GIMBAL_TEST_MODE
    J_scope_gimbal_test();
#endif

    vTaskDelay(GIMBAL_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
    gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    vTaskDelay(1);
  }
}

/**
 * @brief          gimbal cali data, set motor offset encode, max and min relative angle
 * @param[in]      yaw_offse:yaw middle place encode
 * @param[in]      pitch_offset:pitch place encode
 * @param[in]      max_yaw:yaw max relative angle
 * @param[in]      min_yaw:yaw min relative angle
 * @param[in]      max_yaw:pitch max relative angle      
 * @param[in]      min_yaw:pitch min relative angle
 * @retval         none
 */
/**
 * @brief          云台校准设置，将校准的云台中值以及最小最大机械相对角度
 * @param[in]      yaw_offse:yaw 中值
 * @param[in]      pitch_offset:pitch 中值
 * @param[in]      max_yaw:max_yaw:yaw 最大相对角度
 * @param[in]      min_yaw:yaw 最小相对角度
 * @param[in]      max_yaw:pitch 最大相对角度
 * @param[in]      min_yaw:pitch 最小相对角度
 * @retval         返回空
 * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
 */
void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch)
{
  gimbal_control.gimbal_yaw_motor.offset_ecd = yaw_offset;
  gimbal_control.gimbal_yaw_motor.max_relative_angle = max_yaw;
  gimbal_control.gimbal_yaw_motor.min_relative_angle = min_yaw;

  gimbal_control.gimbal_pitch_motor.offset_ecd = pitch_offset;
  gimbal_control.gimbal_pitch_motor.max_relative_angle = max_pitch;
  gimbal_control.gimbal_pitch_motor.min_relative_angle = min_pitch;
}

/**
 * @brief          gimbal cali calculate, return motor offset encode, max and min relative angle
 * @param[out]     yaw_offse:yaw middle place encode
 * @param[out]     pitch_offset:pitch place encode
 * @param[out]     max_yaw:yaw max relative angle
 * @param[out]     min_yaw:yaw min relative angle
 * @param[out]     max_yaw:pitch max relative angle
 * @param[out]     min_yaw:pitch min relative angle
 * @retval         none
 */
/**
 * @brief          云台校准计算，将校准记录的中值,最大 最小值返回
 * @param[out]     yaw 中值 指针
 * @param[out]     pitch 中值 指针
 * @param[out]     yaw 最大相对角度 指针
 * @param[out]     yaw 最小相对角度 指针
 * @param[out]     pitch 最大相对角度 指针
 * @param[out]     pitch 最小相对角度 指针
 * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
 * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
 */
bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
  if (gimbal_control.gimbal_cali.step == 0)
  {
    gimbal_control.gimbal_cali.step = GIMBAL_CALI_START_STEP;
    // 保存进入时候的数据，作为起始数据，来判断最大，最小值
    gimbal_control.gimbal_cali.max_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
    gimbal_control.gimbal_cali.max_pitch_ecd = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
    gimbal_control.gimbal_cali.max_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;
    gimbal_control.gimbal_cali.max_yaw_ecd = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
    gimbal_control.gimbal_cali.min_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
    gimbal_control.gimbal_cali.min_pitch_ecd = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
    gimbal_control.gimbal_cali.min_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;
    gimbal_control.gimbal_cali.min_yaw_ecd = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
    return 0;
  }
  else if (gimbal_control.gimbal_cali.step == GIMBAL_CALI_END_STEP)
  {
    calc_gimbal_cali(&gimbal_control.gimbal_cali, yaw_offset, pitch_offset, max_yaw, min_yaw, max_pitch, min_pitch);
    (*max_yaw) -= GIMBAL_CALI_REDUNDANT_ANGLE;
    (*min_yaw) += GIMBAL_CALI_REDUNDANT_ANGLE;
    (*max_pitch) -= GIMBAL_CALI_REDUNDANT_ANGLE;
    (*min_pitch) += GIMBAL_CALI_REDUNDANT_ANGLE;
    gimbal_control.gimbal_yaw_motor.offset_ecd = *yaw_offset;
    gimbal_control.gimbal_yaw_motor.max_relative_angle = *max_yaw;
    gimbal_control.gimbal_yaw_motor.min_relative_angle = *min_yaw;
    gimbal_control.gimbal_pitch_motor.offset_ecd = *pitch_offset;
    gimbal_control.gimbal_pitch_motor.max_relative_angle = *max_pitch;
    gimbal_control.gimbal_pitch_motor.min_relative_angle = *min_pitch;
    gimbal_control.gimbal_cali.step = 0;
    return 1;
  }
  else
  {
    return 0;
  }
}

/**
 * @brief          calc motor offset encode, max and min relative angle
 * @param[out]     yaw_offse:yaw middle place encode
 * @param[out]     pitch_offset:pitch place encode
 * @param[out]     max_yaw:yaw max relative angle
 * @param[out]     min_yaw:yaw min relative angle
 * @param[out]     max_yaw:pitch max relative angle
 * @param[out]     min_yaw:pitch min relative angle
 * @retval         none
 */
/**
 * @brief          云台校准计算，将校准记录的中值,最大 最小值
 * @param[out]     yaw 中值 指针
 * @param[out]     pitch 中值 指针
 * @param[out]     yaw 最大相对角度 指针
 * @param[out]     yaw 最小相对角度 指针
 * @param[out]     pitch 最大相对角度 指针
 * @param[out]     pitch 最小相对角度 指针
 * @retval         none
 */
static void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
  if (gimbal_cali == NULL || yaw_offset == NULL || pitch_offset == NULL || max_yaw == NULL || min_yaw == NULL || max_pitch == NULL || min_pitch == NULL)
  {
    return;
  }

  int16_t temp_max_ecd = 0, temp_min_ecd = 0, temp_ecd = 0;

#if YAW_TURN
  temp_ecd = gimbal_cali->min_yaw_ecd - gimbal_cali->max_yaw_ecd;

  if (temp_ecd < 0)
  {
    temp_ecd += ecd_range;
  }
  temp_ecd = gimbal_cali->max_yaw_ecd + (temp_ecd / 2);

  ecd_format(temp_ecd);
  *yaw_offset = temp_ecd;
  *max_yaw = -motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
  *min_yaw = -motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#else

  temp_ecd = gimbal_cali->max_yaw_ecd - gimbal_cali->min_yaw_ecd;

  if (temp_ecd < 0)
  {
    temp_ecd += ECD_RANGE;
  }
  temp_ecd = gimbal_cali->max_yaw_ecd - (temp_ecd / 2);

  ecd_format(temp_ecd);
  *yaw_offset = temp_ecd;
  *max_yaw = motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
  *min_yaw = motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#endif

#if PITCH_TURN

  temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
  temp_max_ecd = gimbal_cali->max_pitch_ecd + temp_ecd;
  temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
  temp_min_ecd = gimbal_cali->min_pitch_ecd + temp_ecd;

  ecd_format(temp_max_ecd);
  ecd_format(temp_min_ecd);

  temp_ecd = temp_max_ecd - temp_min_ecd;

  if (temp_ecd > HALF_ECD_RANGE)
  {
    temp_ecd -= ECD_RANGE;
  }
  else if (temp_ecd < -HALF_ECD_RANGE)
  {
    temp_ecd += ECD_RANGE;
  }

  if (temp_max_ecd > temp_min_ecd)
  {
    temp_min_ecd += ECD_RANGE;
  }

  temp_ecd = temp_max_ecd - temp_ecd / 2;

  ecd_format(temp_ecd);

  *pitch_offset = temp_ecd;

  *max_pitch = -motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
  *min_pitch = -motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);

#else
  temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
  temp_max_ecd = gimbal_cali->max_pitch_ecd - temp_ecd;
  temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
  temp_min_ecd = gimbal_cali->min_pitch_ecd - temp_ecd;

  ecd_format(temp_max_ecd);
  ecd_format(temp_min_ecd);

  temp_ecd = temp_max_ecd - temp_min_ecd;

  if (temp_ecd > HALF_ECD_RANGE)
  {
    temp_ecd -= ECD_RANGE;
  }
  else if (temp_ecd < -HALF_ECD_RANGE)
  {
    temp_ecd += ECD_RANGE;
  }

  temp_ecd = temp_max_ecd - temp_ecd / 2;

  ecd_format(temp_ecd);

  *pitch_offset = temp_ecd;

  *max_pitch = motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
  *min_pitch = motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);
#endif
}

/**
 * @brief          return yaw motor data point
 * @param[in]      none
 * @retval         yaw motor data point
 */
/**
 * @brief          返回yaw 电机数据指针
 * @param[in]      none
 * @retval         yaw电机指针
 */
const gimbal_motor_t *get_yaw_motor_point(void)
{
  return &gimbal_control.gimbal_yaw_motor;
}

/**
 * @brief          return pitch motor data point
 * @param[in]      none
 * @retval         pitch motor data point
 */
/**
 * @brief          返回pitch 电机数据指针
 * @param[in]      none
 * @retval         pitch
 */
const gimbal_motor_t *get_pitch_motor_point(void)
{
  return &gimbal_control.gimbal_pitch_motor;
}

/**
 * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
 *                 data point initialization, and gyro sensor angle point initialization.
 * @param[out]     init: "gimbal_control" valiable point
 * @retval         none
 */
/**
 * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
 * @param[out]     init:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_init(gimbal_control_t *init)
{

  static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
  static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
  // 电机数据指针获取
  init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
  init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
  // 陀螺仪数据指针获取
  init->gimbal_INT_angle_point = get_INS_angle_point();
  init->gimbal_INT_gyro_point = get_gyro_data_point();
  // 遥控器数据指针获取
  init->gimbal_rc_ctrl = get_remote_control_point();
  // 初始化电机模式
  init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
  init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
  // 初始化yaw电机pid，
  gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
  gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);
  PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
  // 初始化pitch电机pid
  gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
  gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
  PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);

  //------------------------------------------------------------------------------

  ffc_init(&init->pid_yaw_ffc,GIMBAL_YAW_PID_FFC_Ka,GIMBAL_YAW_PID_FFC_Kb);

  pid_math_init(&init->Gimbal_Yaw_Vision_Angle_Pid,
                Gimbal_Yaw_Vision_Angle_Kp,
                Gimbal_Yaw_Vision_Angle_Ki,
                Gimbal_Yaw_Vision_Angle_Kd,
                Gimbal_Yaw_Vision_Angle_Maxout,
                Gimbal_Yaw_Vision_Angle_IMaxout,
                3, 0.5f, 0.05f, 0.95f, 0.0001f);
  pid_math_init(&init->Gimbal_Pitch_Vision_Angle_Pid,
                Gimbal_Pitch_Vision_Angle_Kp,
                Gimbal_Pitch_Vision_Angle_Ki,
                Gimbal_Pitch_Vision_Angle_Kd,
                Gimbal_Pitch_Vision_Angle_Maxout,
                Gimbal_Pitch_Vision_Angle_IMaxout,
                3, 0.5f, 0.05f, 0.95f, 0.001f);

  //	pid_math_init(&init->Gimbal_Yaw_IMU_Angle_Pid,Gimbal_Yaw_IMU_Angle_Kp,Gimbal_Yaw_IMU_Angle_Ki,Gimbal_Yaw_IMU_Angle_Kd,Gimbal_Yaw_IMU_Angle_Maxout,Gimbal_Yaw_IMU_Angle_IMaxout,3,0.5f,0.05f,0.9f,0.0001f);

  pid_init(&init->Gimbal_Yaw_Speed_Pid,
           Gimbal_Yaw_Speed_Kp,
           Gimbal_Yaw_Speed_Ki,
           Gimbal_Yaw_Speed_Kd,
           Gimbal_Yaw_Speed_Maxout,
           Gimbal_Yaw_Speed_IMaxout, 1);
  pid_init(&init->Gimbal_Pitch_Speed_Pid,
           Gimbal_Pitch_Speed_Kp,
           Gimbal_Pitch_Speed_Ki,
           Gimbal_Pitch_Speed_Kd,
           Gimbal_Pitch_Speed_Maxout,
           Gimbal_Pitch_Speed_IMaxout, 1);

  // static const fp32 Pitch_speed_pid_t[3] = {PITCH_SPEED_PID_KP_t, PITCH_SPEED_PID_KI_t, PITCH_SPEED_PID_KD_t};
  // static const fp32 Yaw_speed_pid_t[3] = {YAW_SPEED_PID_KP_t, YAW_SPEED_PID_KI_t, YAW_SPEED_PID_KD_t};

  // // 初始化自瞄yaw电机pid
  // gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid_t,
  //                 YAW_GYRO_ABSOLUTE_PID_MAX_OUT_t,
  //                 YAW_GYRO_ABSOLUTE_PID_MAX_IOUT_t,
  //                 YAW_GYRO_ABSOLUTE_PID_KP_t,
  //                 YAW_GYRO_ABSOLUTE_PID_KI_t,
  //                 YAW_GYRO_ABSOLUTE_PID_KD_t);
  // gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid_t,
  //                 YAW_ENCODE_RELATIVE_PID_MAX_OUT_t,
  //                 YAW_ENCODE_RELATIVE_PID_MAX_IOUT_t,
  //                 YAW_ENCODE_RELATIVE_PID_KP_t,
  //                 YAW_ENCODE_RELATIVE_PID_KI_t,
  //                 YAW_ENCODE_RELATIVE_PID_KD_t);
  // PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION,
  //          Yaw_speed_pid_t, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
  // // 初始化自瞄pitch电机pid
  // gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid_t,
  //                 PITCH_GYRO_ABSOLUTE_PID_MAX_OUT_t,
  //                 PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT_t,
  //                 PITCH_GYRO_ABSOLUTE_PID_KP_t,
  //                 PITCH_GYRO_ABSOLUTE_PID_KI_t,
  //                 PITCH_GYRO_ABSOLUTE_PID_KD_t);
  // gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid_t,
  //                 PITCH_ENCODE_RELATIVE_PID_MAX_OUT_t,
  //                 PITCH_ENCODE_RELATIVE_PID_MAX_IOUT_t,
  //                 PITCH_ENCODE_RELATIVE_PID_KP_t,
  //                 PITCH_ENCODE_RELATIVE_PID_KI_t,
  //                 PITCH_ENCODE_RELATIVE_PID_KD_t);
  // PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION,
  //          Pitch_speed_pid_t, PITCH_SPEED_PID_MAX_OUT_t, PITCH_SPEED_PID_MAX_IOUT_t);

  //------------------------------------------------------------------------------

  // 清除所有PID
  gimbal_total_pid_clear(init);

  gimbal_feedback_update(init);

  init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
  init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
  init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;

  init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
  init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.relative_angle;
  init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;

  init->gimbal_yaw_motor.max_relative_angle = 100 * PI;
  init->gimbal_yaw_motor.min_relative_angle = -100 * PI;

  shoot_init();

}

/**
 * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
 * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
 * @retval         none
 */
/**
 * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
 * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_mode(gimbal_control_t *set_mode)
{
  if (set_mode == NULL)
  {
    return;
  }
  gimbal_behaviour_mode_set(set_mode);
}
/**
 * @brief          gimbal some measure data updata, such as motor enconde, euler angle, gyro
 * @param[out]     gimbal_feedback_update: "gimbal_control" valiable point
 * @retval         none
 */
/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
  if (feedback_update == NULL)
  {
    return;
  }
  // 云台数据更新
  feedback_update->gimbal_pitch_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);

#if PITCH_TURN
  feedback_update->gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                  feedback_update->gimbal_pitch_motor.offset_ecd);
#else

  feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                 feedback_update->gimbal_pitch_motor.offset_ecd);
#endif

  feedback_update->gimbal_pitch_motor.motor_gyro = *(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);

  feedback_update->gimbal_yaw_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);

#if YAW_TURN
  feedback_update->gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                feedback_update->gimbal_yaw_motor.offset_ecd);

#else
  feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                               feedback_update->gimbal_yaw_motor.offset_ecd);
#endif
  feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET)) - arm_sin_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));

  if (gimbal_attitude_start_flag)
  {
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) // 若有唤醒任务标志位
    {
      static BaseType_t xHigherPriorityTaskWoken;                                       // 赋予任务优先级
      vTaskNotifyGiveFromISR(gimbal_attitude_local_handler, &xHigherPriorityTaskWoken); // 唤醒任务
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
}

/**
 * @brief          calculate the relative angle between ecd and offset_ecd
 * @param[in]      ecd: motor now encode
 * @param[in]      offset_ecd: gimbal offset encode
 * @retval         relative angle, unit rad
 */
/**
 * @brief          计算ecd与offset_ecd之间的相对角度
 * @param[in]      ecd: 电机当前编码
 * @param[in]      offset_ecd: 电机中值编码
 * @retval         相对角度，单位rad
 */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
  int32_t relative_ecd = ecd - offset_ecd;
  if (relative_ecd > HALF_ECD_RANGE)
  {
    relative_ecd -= ECD_RANGE;
  }
  else if (relative_ecd < -HALF_ECD_RANGE)
  {
    relative_ecd += ECD_RANGE;
  }

  return relative_ecd * MOTOR_ECD_TO_RAD;
}

/**
 * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
 * @param[out]     gimbal_mode_change: "gimbal_control" valiable point
 * @retval         none
 */
/**
 * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
 * @param[out]     gimbal_mode_change:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
		if (gimbal_mode_change == NULL)
		{
			return;
		}
	
		if(sentinel_gimbal_behaviour != GIMBAL_NONE && sentinel_gimbal_behaviour != GIMBAL_TRACK_ENEMY_MOD && sentinel_gimbal_behaviour == GIMBAL_SCAN_MOD && LAST_SENTINEL_GIMBAL_MOD != sentinel_gimbal_behaviour)
		{
			gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
			gimbal_mode_change->Gimbal_Yaw_Control_Data = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
		}
		else if(sentinel_gimbal_behaviour != GIMBAL_NONE && sentinel_gimbal_behaviour == GIMBAL_TRACK_ENEMY_MOD && sentinel_gimbal_behaviour != GIMBAL_SCAN_MOD && LAST_SENTINEL_GIMBAL_MOD != sentinel_gimbal_behaviour)
		{
			gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
			gimbal_mode_change->Gimbal_Yaw_Control_Data = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
		}
		
		if(sentinel_gimbal_behaviour != GIMBAL_NONE && sentinel_gimbal_behaviour != GIMBAL_TRACK_ENEMY_MOD && sentinel_gimbal_behaviour == GIMBAL_SCAN_MOD && LAST_SENTINEL_GIMBAL_MOD != sentinel_gimbal_behaviour)
		{
			gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
			gimbal_mode_change->Gimbal_Pitch_Control_Data = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
		}
		else if(sentinel_gimbal_behaviour != GIMBAL_NONE && sentinel_gimbal_behaviour == GIMBAL_TRACK_ENEMY_MOD && sentinel_gimbal_behaviour != GIMBAL_SCAN_MOD && LAST_SENTINEL_GIMBAL_MOD != sentinel_gimbal_behaviour)
		{
			gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
			gimbal_mode_change->Gimbal_Pitch_Control_Data = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
		}
		
		LAST_SENTINEL_GIMBAL_MOD = sentinel_gimbal_behaviour;
}
/**
 * @brief          set gimbal control set-point, control set-point is set by "gimbal_behaviour_control_set".
 * @param[out]     gimbal_set_control: "gimbal_control" valiable point
 * @retval         none
 */
/**
 * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
 * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_control(gimbal_control_t *set_control)
{
  if (set_control == NULL)
  {
    return;
  }
  fp32 yaw_speed = 0.0f;
	fp32 pitch_speed = 0.0f;
	fp32 add_yaw_angle = 0.0f;
	fp32 add_pitch_angle = 0.0f;
	
	if(gimbal_behaviour == GIMBAL_AUTO_SENTINEL)
	{
		if(sentinel_gimbal_behaviour == GIMBAL_SCAN_MOD) //扫描模式
		{
				gimbal_scan_control(&yaw_speed, &pitch_speed,set_control, &gimbal_distance);
				gimbal_scan_pid_control(yaw_speed,pitch_speed); //为了避免麻烦，直接在这里计算PID
		}
		 else if (sentinel_gimbal_behaviour == GIMBAL_TRACK_ENEMY_MOD)
		{
			Gimbal_Vision_Control_Set(set_control);
		}
	}
	else if(gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
	{
		gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);
//		add_yaw_angle = add_yaw_angle * 0.2f;
//		gimbal_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
//		gimbal_motor_absolute_angle_control(&set_control->gimbal_yaw_motor);
//		add_pitch_angle = add_pitch_angle * 0.5f;
//		if(set_control->gimbal_pitch_motor.absolute_angle_set > Pitch_Max_Angle)
//		{
//			set_control->gimbal_pitch_motor.absolute_angle_set = Pitch_Max_Angle;
//		}
//		else if(set_control->gimbal_pitch_motor.absolute_angle_set < Pitch_Min_Angle)
//		{
//			set_control->gimbal_pitch_motor.absolute_angle_set = Pitch_Min_Angle;
//		}
//		gimbal_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
//		gimbal_motor_absolute_angle_control(&set_control->gimbal_pitch_motor);
		Gimbal_Infantry_Control_Set(set_control,add_yaw_angle,add_pitch_angle);
	}
}
/**
 * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control.
 * @param[out]     gimbal_motor: yaw motor or pitch motor
 * @retval         none
 */
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
  static fp32 bias_angle;
  static fp32 angle_set;
  if (gimbal_motor == NULL)
  {
    return;
  }
  // now angle error
  // 当前控制误差角度
//  bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
  // relative angle + angle error + add_angle > max_relative angle
  // 云台相对角度+ 误差角度 + 新增角度 如果大于 最大机械角度
//  if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
//  {
//    // 如果是往最大机械角度控制方向
//    if (add > 0.0f)
//    {
//      // calculate max add_angle
//      // 计算出一个最大的添加角度，
//      add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
//    }
//  }
//  else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle)
//  {
//    if (add < 0.0f)
//    {
//      add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
//    }
//  }
  angle_set = gimbal_motor->absolute_angle_set;
  gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}
/**
 * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode relative angle  to control.
 * @param[out]     gimbal_motor: yaw motor or pitch motor
 * @retval         none
 */
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
  if (gimbal_motor == NULL)
  {
    return;
  }
  gimbal_motor->relative_angle_set += add;
  // 是否超过最大 最小值
  if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
  {
    gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
  }
  else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
  {
    gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
  }
}


/**
 * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control.
 * @param[out]     gimbal_motor: yaw motor or pitch motor
 * @retval         none
 */
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{
  if (gimbal_motor == NULL)
  {
    return;
  }
  // 角度环，速度环串级pid调试
  gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
  gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
//  debug_dispose(&gimbal_motor->gimbal_motor_gyro_pid, &gimbal_motor->motor_gyro, &gimbal_motor->motor_gyro_set,1);
  // 控制值赋值
  gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}
/**
 * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode relative angle  to control.
 * @param[out]     gimbal_motor: yaw motor or pitch motor
 * @retval         none
 */
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{
  if (gimbal_motor == NULL)
  {
    return;
  }

  // 角度环，速度环串级pid调试
  gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, gimbal_motor->motor_gyro);
  gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
  // 控制值赋值
  gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

/**
 * @brief          gimbal control mode :GIMBAL_MOTOR_RAW, current  is sent to CAN bus.
 * @param[out]     gimbal_motor: yaw motor or pitch motor
 * @retval         none
 */
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_RAW，电流值直接发送到CAN总线.
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor)
{
  if (gimbal_motor == NULL)
  {
    return;
  }
  gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
  gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

#if GIMBAL_TEST_MODE
int32_t yaw_ins_int_1000, pitch_ins_int_1000;
int32_t yaw_ins_set_1000, pitch_ins_set_1000;
int32_t pitch_relative_set_1000, pitch_relative_angle_1000;
int32_t yaw_speed_int_1000, pitch_speed_int_1000;
int32_t yaw_speed_set_int_1000, pitch_speed_set_int_1000;
static void J_scope_gimbal_test(void)
{
  yaw_ins_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle * 1000);
  yaw_ins_set_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle_set * 1000);
  yaw_speed_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro * 1000);
  yaw_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro_set * 1000);

  pitch_ins_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle * 1000);
  pitch_ins_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle_set * 1000);
  pitch_speed_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro * 1000);
  pitch_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro_set * 1000);
  pitch_relative_angle_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle * 1000);
  pitch_relative_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle_set * 1000);
}

#endif

/**
 * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
 *                 data point initialization, and gyro sensor angle point initialization.
 * @param[out]     gimbal_init: "gimbal_control" valiable point
 * @retval         none
 */
/**
 * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
 * @param[out]     gimbal_init:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
  if (pid == NULL)
  {
    return;
  }
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;

  pid->err = 0.0f;
  pid->get = 0.0f;

  pid->max_iout = max_iout;
  pid->max_out = maxout;
}

static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
  fp32 err;
  if (pid == NULL)
  {
    return 0.0f;
  }
  pid->get = get;
  pid->set = set;

  err = set - get;
  pid->err = rad_format(err);
  pid->Pout = pid->kp * pid->err;
  pid->Iout += pid->ki * pid->err;
  pid->Dout = pid->kd * error_delta;
  abs_limit(&pid->Iout, pid->max_iout);
  pid->out = pid->Pout + pid->Iout + pid->Dout;
  abs_limit(&pid->out, pid->max_out);
  return pid->out;
}

/**
 * @brief          gimbal PID clear, clear pid.out, iout.
 * @param[out]     gimbal_pid_clear: "gimbal_control" valiable point
 * @retval         none
 */
/**
 * @brief          云台PID清除，清除pid的out,iout
 * @param[out]     gimbal_pid_clear:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear)
{
  if (gimbal_pid_clear == NULL)
  {
    return;
  }
  gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
  gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}

uint8_t pitch_turn_flag = 0;

/**
   * @brief 扫描模式PID控制
   * @param yaw轴速度
   * @retval None.
   */
static void gimbal_scan_pid_control(fp32 yaw_speed , fp32 pitch_speed)
{
	if(!(path_control.path_ecd_set.disable_yaw_scan[path_control.behaviour_count] == DISABLE_SCAN))
	{
		gimbal_control.Gimbal_Yaw_Control_Data = rad_format(gimbal_control.Gimbal_Yaw_Control_Data + yaw_speed);
		FeedforwardController(&gimbal_control.pid_yaw_ffc,rad_format(gimbal_control.Gimbal_Yaw_Control_Data - gimbal_control.gimbal_yaw_motor.absolute_angle));
	}
	else
	{
		gimbal_control.Gimbal_Yaw_Control_Data = path_control.path_ecd_set.scan_angle_set[path_control.behaviour_count];
		FeedforwardController(&gimbal_control.pid_yaw_ffc,rad_format(gimbal_control.Gimbal_Yaw_Control_Data));		
	}
		gimbal_control.Gimbal_Yaw_Apid_Out =
				Gimbal_Math_pid_calc(&gimbal_control.Gimbal_Yaw_Vision_Angle_Pid,
														 0.0f,
														 rad_format(gimbal_control.Gimbal_Yaw_Control_Data - gimbal_control.gimbal_yaw_motor.absolute_angle));

			gimbal_control.Gimbal_Yaw_Apid_Out += gimbal_control.pid_yaw_ffc.out;
		
	  gimbal_control.gimbal_yaw_motor.current_set =
      pid_calc(&gimbal_control.Gimbal_Yaw_Speed_Pid,
               0.0f,
               (gimbal_control.Gimbal_Yaw_Apid_Out - gimbal_control.gimbal_yaw_motor.motor_gyro));

	
	  gimbal_control.gimbal_yaw_motor.given_current = (int16_t)(gimbal_control.gimbal_yaw_motor.current_set);

	  if(gimbal_control.gimbal_pitch_motor.absolute_angle >= SENTINEL_PITCH_MAX_ANGLE && pitch_turn_flag == 0)
	  {
		  pitch_turn_flag = 1;
	  }
	  else if(gimbal_control.gimbal_pitch_motor.absolute_angle <= SENTINEL_PITCH_MIN_ANGLE && pitch_turn_flag == 1)
	  {
		  pitch_turn_flag = 0;
	  }
	  
	  if(pitch_turn_flag == 1)
	  {
		  pitch_speed = -pitch_speed;
	  }
		
	if(path_control.path_ecd_set.disable_yaw_scan[path_control.behaviour_count] == DISABLE_SCAN)
	{
		gimbal_control.Gimbal_Pitch_Control_Data = 0.0f;
	}
	else
	{
		gimbal_control.Gimbal_Pitch_Control_Data = rad_format(gimbal_control.Gimbal_Pitch_Control_Data + pitch_speed);	
	}		
	
		Angle_Control_Limit(&gimbal_control.Gimbal_Pitch_Control_Data, Pitch_Max_Angle, Pitch_Min_Angle); // 限幅 rad	
	
	  gimbal_control.Gimbal_Pitch_Apid_Out =
      Gimbal_Math_pid_calc(&gimbal_control.Gimbal_Pitch_Vision_Angle_Pid,
                           gimbal_control.gimbal_pitch_motor.absolute_angle,
                           gimbal_control.Gimbal_Pitch_Control_Data);

	  gimbal_control.gimbal_pitch_motor.current_set =
      pid_calc(&gimbal_control.Gimbal_Pitch_Speed_Pid,
               gimbal_control.gimbal_pitch_motor.motor_gyro,
               gimbal_control.Gimbal_Pitch_Apid_Out);
	
	  gimbal_control.gimbal_pitch_motor.given_current = (int16_t)(gimbal_control.gimbal_pitch_motor.current_set);
}

/**
 * @brief 扫描模式下pitch轴锁定(LOCK)
 * @param pitch轴
 * @retval None.
 */
static void gimbal_scan_relative_angle_limit(gimbal_motor_t *gimbal_motor)
{
  gimbal_motor->relative_angle_set = SCAN_PITCH_ANGLE;
  // 是否超过最大 最小值
  if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
  {
    gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
  }
  else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
  {
    gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
  }
}



float Angle_Limit(float Angle_Set)
{
  //	  int32_t relative_ecd =  Now_Angle - Angle_Middle;
  if (Angle_Set > 3.1415926f)
  {
    Angle_Set -= 2 * 3.1415926f;
  }
  else if (Angle_Set < -3.1415926f)
  {
    Angle_Set += 2 * 3.1415926f;
  }
  return Angle_Set;
}


// PID函数(变速积分+不完全微分)
float Gimbal_Math_pid_calc(PID *pid, float now, float set)
{
  pid->now = now;
  pid->set = set;

  pid->now_error = pid->set - pid->now; // set - measure

//  pid->now_error = Angle_Limit(pid->now_error);
  if ((pid->now_error < pid->Small_Error_Limit) && (pid->now_error > -pid->Small_Error_Limit))
    return 0;
  if (pid->pid_mode == 1) // 位置环PID
  {
    pid->pout = pid->kp * pid->now_error;
    pid->iout = pid->ki * pid->sum_of_error;
    pid->dout = pid->kd * (pid->now_error - pid->Last_error);
    pid->sum_of_error += pid->now_error;
    PID_limit(&(pid->sum_of_error), 20000);
    PID_limit(&(pid->iout), pid->IntegralLimit);
    pid->out = pid->pout + pid->iout + pid->dout;
    PID_limit(&(pid->out), pid->MaxOutput);
  }

  else if (pid->pid_mode == 2) // 增量式PID
  {
    pid->pout = pid->kp * (pid->now_error - pid->Last_error);
    pid->iout = pid->ki * pid->now_error;
    pid->dout = pid->kd * (pid->now_error - 2 * pid->Last_error + pid->Last_Last_error);
    PID_limit(&(pid->iout), pid->IntegralLimit);
    pid->plus = pid->pout + pid->iout + pid->dout;
    pid->plus_out = pid->last_plus_out + pid->plus;
    pid->out = pid->plus_out;
    PID_limit(&(pid->out), pid->MaxOutput);
    pid->last_plus_out = pid->plus_out; // update last time
  }
  else if (pid->pid_mode == 3) // 变速积分/不完全微分
  {
    pid->pout = pid->kp * pid->now_error;
    if (((pid->Set_Out_Mode == 1) && (pid->now_error > 0)) || ((pid->Set_Out_Mode == -1) && (pid->now_error < 0)))
      ;
    else
    {
      if (PID_Fabs(pid->now_error) <= pid->Set_B)
      {
        pid->sum_of_error += pid->now_error;
      }
      else if (PID_Fabs(pid->now_error) >= (pid->Set_B + pid->Set_A))
      {
        pid->sum_of_error = 0;
      }
      else
      {
        pid->Set_ratio = (pid->Set_A + pid->Set_B - PID_Fabs(pid->now_error)) / pid->Set_A;
        pid->sum_of_error += pid->Set_ratio * pid->now_error;
      }
    }
    // 变速积分
    pid->iout = pid->ki * pid->sum_of_error;
    // 不完全微分
    pid->dout = pid->kd * (pid->now_error - pid->Last_error) * (1 - pid->Set_alpha) + pid->Set_alpha * pid->Last_Ud;

    pid->out = pid->pout + pid->iout + pid->dout;
    if (pid->out > pid->MaxOutput)
    {
      pid->out = pid->MaxOutput;
      pid->Set_Out_Mode = 1;
    }
    else
    {
      pid->Set_Out_Mode = 0;
    }

    if (pid->out < -pid->MaxOutput)
    {
      pid->out = -pid->MaxOutput;
      pid->Set_Out_Mode = -1;
    }
    else
    {
      pid->Set_Out_Mode = 0;
    }
    pid->Last_Ud = pid->dout;
  }

  pid->Last_Last_error = pid->Last_error;
  pid->Last_error = pid->now_error;

  return pid->out;
}
float Gimbal_Vision_PID[3];
float Gimbal_Vision_pid_calc(PID *pid, float now, float set)
{
  //  float Vision_Error;
  //  float Vision_Now;
  //  float Vision_Pout, Vision_Iout, Vision_Dout;
  pid->now = now;
  pid->set = set;

  //    if(pid->now  > 50000)
  //			pid->now  = pid->now -65533;
  pid->now_error = pid->set - pid->now; // set - measure
  pid->now_error = Angle_Limit(pid->now_error);

  //    if(pid->pid_mode == 1) //位置环PID
  //    {
  //	      pid->pout = pid->kp * pid->now_error;
  //        pid->iout = pid->ki * pid->sum_of_error;
  //        pid->dout = pid->kd * (pid->now_error - pid->Last_error );
  //				pid->sum_of_error+=pid->now_error;
  //				PID_limit(&(pid->sum_of_error), 20000);
  //				PID_limit(&(pid->iout), pid->IntegralLimit);
  //        pid->out = pid->pout + pid->iout + pid->dout;
  //        PID_limit(&(pid->out), pid->MaxOutput);
  //    }

  //   else if(pid->pid_mode == 2)//增量式PID
  {
    pid->pout = pid->kp * (pid->now_error - pid->Last_error);
    pid->iout = pid->ki * pid->now_error;
    pid->dout = pid->kd * (pid->now_error - 2 * pid->Last_error + pid->Last_Last_error);
    PID_limit(&(pid->iout), pid->IntegralLimit);
    pid->plus = pid->pout + pid->iout + pid->dout;
    pid->plus_out = pid->last_plus_out + pid->plus;
    pid->out = pid->plus_out;
    PID_limit(&(pid->out), pid->MaxOutput);
    pid->last_plus_out = pid->plus_out; // update last time
  }

  pid->Last_Last_error = pid->Last_error;
  pid->Last_error = pid->now_error;

  return pid->out;
}

void Angle_Control_Limit(float *Angle_Set, float Max_Angle, float Min_Angle)
{
  if (*Angle_Set > Max_Angle)
    *Angle_Set = Max_Angle;
  else if (*Angle_Set < Min_Angle)
    *Angle_Set = Min_Angle;
}

float Yaw_Vision_Plus = 0.8f,Pitch_Vision_Plus = 0.6f; //视觉灵敏度
float Pitch_Vision_Plus_Auxiliary = 0.006f,Pitch_Vision_Plus_Auxiliary_min = 0.03f;
float Yaw_Divide_Plus_x = 0.00065f,Yaw_Divide_Plus_y = 0.001f; //0.005f 0.008f
float Yaw_divide_min = 0.01f,Yaw_divide_middle = 0.5f,Yaw_Allow_Error_Angle = 0.02f; //0.01f~0.04f
float Pitch_Divide_Plus_x = 0.0005f,Pitch_divide_min = 0.03f,Pitch_Allow_Error_Angle = 0.03f;

static void Gimbal_Vision_Control_Set(gimbal_control_t *Gimbal_Control_Set)
{
	fp32 yaw_diff_temp = 0.0f;
	fp32 pitch_diff_temp = 0.0f; 
	if(CV_EV.valid > 0)
	{
		Gimbal_Control_Set->Gimbal_Yaw_Vision_Data_Target = rad_format(CV_EV.yaw + Gimbal_Control_Set->gimbal_yaw_motor.absolute_angle);
		Gimbal_Control_Set->Gimbal_Pitch_Vision_Data_Target = rad_format(CV_EV.pitch + Gimbal_Control_Set->gimbal_pitch_motor.absolute_angle);
		CV_EV.valid = 0;
	}
		
	if(CV_EV.state >= 1)
	{
		yaw_diff_temp = rad_format(Gimbal_Control_Set->Gimbal_Yaw_Vision_Data_Target - Gimbal_Control_Set->Gimbal_Yaw_Control_Data);
		pitch_diff_temp = rad_format(Gimbal_Control_Set->Gimbal_Pitch_Vision_Data_Target - Gimbal_Control_Set->Gimbal_Pitch_Control_Data);
		if(PC_ABS(yaw_diff_temp) > Yaw_divide_min && yaw_diff_temp < 0)
		{
			if(PC_ABS(yaw_diff_temp) > Yaw_divide_middle)
			{
				yaw_diff_temp = -Yaw_Divide_Plus_y;
			}
			else
			{
				yaw_diff_temp = -Yaw_Divide_Plus_x;
			}
		}
		else if(PC_ABS(yaw_diff_temp) > Yaw_divide_min && yaw_diff_temp > 0)
		{
			if(PC_ABS(yaw_diff_temp) > Yaw_divide_middle)
			{
				yaw_diff_temp = Yaw_Divide_Plus_y;
			}
			else
			{
				yaw_diff_temp = Yaw_Divide_Plus_x;
			}
		}
		else if(PC_ABS(rad_format(Gimbal_Control_Set->Gimbal_Yaw_Vision_Data_Target - Gimbal_Control_Set->gimbal_pitch_motor.absolute_angle)) < (Yaw_divide_min + Yaw_Allow_Error_Angle))
		{
			yaw_diff_temp = rad_format(Gimbal_Control_Set->Gimbal_Yaw_Vision_Data_Target - Gimbal_Control_Set->Gimbal_Yaw_Control_Data) * Yaw_Vision_Plus;		
		}
		else
		{
			yaw_diff_temp = 0.0f;
		}
		
		if(PC_ABS(pitch_diff_temp) > Pitch_divide_min && pitch_diff_temp > 0)
		{
			pitch_diff_temp = Pitch_Divide_Plus_x;
		}
		else if(PC_ABS(pitch_diff_temp) > Pitch_divide_min && pitch_diff_temp < 0)
		{
			pitch_diff_temp = -Pitch_Divide_Plus_x;
		}
		else if(PC_ABS(rad_format(Gimbal_Control_Set->Gimbal_Pitch_Vision_Data_Target - Gimbal_Control_Set->gimbal_pitch_motor.absolute_angle)) < (Pitch_divide_min + Pitch_Allow_Error_Angle))
		{
			pitch_diff_temp = rad_format(Gimbal_Control_Set->Gimbal_Pitch_Vision_Data_Target - Gimbal_Control_Set->Gimbal_Pitch_Control_Data) * Pitch_Vision_Plus;
		}
		else
		{
			pitch_diff_temp = 0.0f;
		}
		
		//Gimbal_Control_Set->Gimbal_Yaw_Add_Data = rad_format(Gimbal_Control_Set->Gimbal_Yaw_Vision_Data_Target - Gimbal_Control_Set->gimbal_yaw_motor.absolute_angle) * Yaw_Vision_Plus;      //- (float)((RC_Ctl_Data.rc.ch2)/6600.000) / 180.00f * 3.1415926f
		Gimbal_Control_Set->Gimbal_Yaw_Add_Data = yaw_diff_temp;
		Gimbal_Control_Set->Gimbal_Pitch_Add_Data = pitch_diff_temp;
	}
	
	Gimbal_Control_Set->Gimbal_Yaw_Control_Data += Gimbal_Control_Set->Gimbal_Yaw_Add_Data;
  Gimbal_Control_Set->Gimbal_Pitch_Control_Data += Gimbal_Control_Set->Gimbal_Pitch_Add_Data;
		
	Gimbal_Control_Set->Gimbal_Yaw_Control_Data = rad_format(Gimbal_Control_Set->Gimbal_Yaw_Control_Data);
	Gimbal_Control_Set->Gimbal_Pitch_Control_Data = rad_format(Gimbal_Control_Set->Gimbal_Pitch_Control_Data);

  Angle_Control_Limit(&Gimbal_Control_Set->Gimbal_Pitch_Control_Data, Pitch_Max_Angle, Pitch_Min_Angle); // 限幅 rad	

  Gimbal_Control_Set->Gimbal_Pitch_Apid_Out =
      Gimbal_Math_pid_calc(&Gimbal_Control_Set->Gimbal_Pitch_Vision_Angle_Pid,
                           Gimbal_Control_Set->gimbal_pitch_motor.absolute_angle,
                           Gimbal_Control_Set->Gimbal_Pitch_Control_Data);
	
  Gimbal_Control_Set->gimbal_pitch_motor.current_set =
      pid_calc(&Gimbal_Control_Set->Gimbal_Pitch_Speed_Pid,
               Gimbal_Control_Set->gimbal_pitch_motor.motor_gyro,
               Gimbal_Control_Set->Gimbal_Pitch_Apid_Out);

	FeedforwardController(&Gimbal_Control_Set->pid_yaw_ffc,rad_format(Gimbal_Control_Set->Gimbal_Yaw_Control_Data - Gimbal_Control_Set->gimbal_yaw_motor.absolute_angle));

  Gimbal_Control_Set->Gimbal_Yaw_Apid_Out =
      Gimbal_Math_pid_calc(&Gimbal_Control_Set->Gimbal_Yaw_Vision_Angle_Pid,
                           0.0f,
                           rad_format(Gimbal_Control_Set->Gimbal_Yaw_Control_Data - Gimbal_Control_Set->gimbal_yaw_motor.absolute_angle));

	Gimbal_Control_Set->Gimbal_Yaw_Apid_Out += Gimbal_Control_Set->pid_yaw_ffc.out;

  Gimbal_Control_Set->gimbal_yaw_motor.current_set =
      pid_calc(&Gimbal_Control_Set->Gimbal_Yaw_Speed_Pid,
               Gimbal_Control_Set->gimbal_yaw_motor.motor_gyro,
               Gimbal_Control_Set->Gimbal_Yaw_Apid_Out);

  Gimbal_Control_Set->gimbal_yaw_motor.given_current = (int16_t)(Gimbal_Control_Set->gimbal_yaw_motor.current_set);
  Gimbal_Control_Set->gimbal_pitch_motor.given_current = (int16_t)( Gimbal_Control_Set->gimbal_pitch_motor.current_set);
//    Gimbal_Control_Set->gimbal_yaw_motor.given_current = 0.0f;
//     Gimbal_Control_Set->gimbal_pitch_motor.given_current = 0.0f;

  
}



float Yaw_Infantry_Plus = 1.0f, Pitch_Infantry_Plus = 0.8f; // float Yaw_Vision_Plus = 0.000157999995,Pitch_Vision_Plus = 0.00013888;
/**
   * @brief 步兵模式下的云台操控
   * @param add_yaw,add_pitch
   * @retval None.
   */
static void Gimbal_Infantry_Control_Set(gimbal_control_t *Gimbal_Control_Set,float add_yaw,float add_pitch)
{
  ////	Gimbal_Control_Set->Gimbal_Yaw_Add_Data = yaw_angle_get / 180.00f * 3.1415926f;//- (float)((RC_Ctl_Data.rc.ch2)/6600.000) / 180.00f * 3.1415926f
  ////	Gimbal_Control_Set->Gimbal_Pitch_Add_Data = pitch_angle_get / 180.00f * 3.1415926f ;
  //	Yaw_Vision_Plus = - (float)((RC_Ctl_Data.rc.ch2)/6600.000) / 100000.0 ;
  //	Pitch_Vision_Plus = -(float)((RC_Ctl_Data.rc.ch3)/6600.000) / 100000.0f ;

//	float yaw_delat_angle = 0.0f;
	
//  Gimbal_Control_Set->Gimbal_Yaw_Vision_Data_Target = CV_EV.yaw + Gimbal_Control_Set->gimbal_yaw_motor.absolute_angle + delat_yaw_angle * yaw_plus;
//	Gimbal_Control_Set->Gimbal_Pitch_Vision_Data_Target = CV_EV.pitch + Gimbal_Control_Set->gimbal_pitch_motor.absolute_angle + delat_pitch_angle * pitch_plus;
	
	Gimbal_Control_Set->Gimbal_Yaw_Add_Data = add_yaw * Yaw_Infantry_Plus;      //- (float)((RC_Ctl_Data.rc.ch2)/6600.000) / 180.00f * 3.1415926f
	Gimbal_Control_Set->Gimbal_Pitch_Add_Data = add_pitch * Pitch_Infantry_Plus; //-(float)((RC_Ctl_Data.rc.ch3)/6600.000) / 180.00f * 3.1415926f


	Gimbal_Control_Set->Gimbal_Yaw_Control_Data += Gimbal_Control_Set->Gimbal_Yaw_Add_Data;
	Gimbal_Control_Set->Gimbal_Pitch_Control_Data += Gimbal_Control_Set->Gimbal_Pitch_Add_Data;
	
//	yaw_delat_angle = Gimbal_Control_Set->Gimbal_Yaw_Add_Data;
	
	Gimbal_Control_Set->Gimbal_Yaw_Control_Data = rad_format(Gimbal_Control_Set->Gimbal_Yaw_Control_Data);
	Gimbal_Control_Set->Gimbal_Pitch_Control_Data = rad_format(Gimbal_Control_Set->Gimbal_Pitch_Control_Data);
//	Gimbal_Control_Set->Gimbal_Yaw_Vision_Data_Target = rad_format(Gimbal_Control_Set->Gimbal_Yaw_Vision_Data_Target);
//	Gimbal_Control_Set->Gimbal_Pitch_Vision_Data_Target = rad_format(Gimbal_Control_Set->Gimbal_Pitch_Vision_Data_Target);
	
  //	Gimbal_Control_Set->Gimbal_Yaw_Control_Data = Angle_Limit(Gimbal_Control_Set->Gimbal_Yaw_Control_Data);
  Angle_Control_Limit(&Gimbal_Control_Set->Gimbal_Pitch_Control_Data, Pitch_Max_Angle, Pitch_Min_Angle); // 限幅 rad
//  Angle_Control_Limit(&Gimbal_Control_Set->Gimbal_Yaw_Control_Data, Yaw_Max_Angle, Yaw_Min_Angle);
	
	FeedforwardController(&Gimbal_Control_Set->pid_yaw_ffc,rad_format(Gimbal_Control_Set->Gimbal_Yaw_Control_Data - Gimbal_Control_Set->gimbal_yaw_motor.absolute_angle));
  //	Gimbal_Control_Set->Gimbal_Yaw_Control_Data += Gimbal_Control_Set->Gimbal_Yaw_Add_Data;
  //	Gimbal_Control_Set->Gimbal_Pitch_Control_Data += Gimbal_Control_Set->Gimbal_Pitch_Add_Data;

  //	Gimbal_Control_Set->Gimbal_Yaw_Control_Data = Angle_Limit(Gimbal_Control_Set->Gimbal_Yaw_Control_Data);
  //	Angle_Control_Limit(&Gimbal_Control_Set->Gimbal_Pitch_Control_Data,Pitch_Max_Angle,Pitch_Min_Angle);

  Gimbal_Control_Set->Gimbal_Pitch_Apid_Out =
      Gimbal_Math_pid_calc(&Gimbal_Control_Set->Gimbal_Pitch_Vision_Angle_Pid,
                           Gimbal_Control_Set->gimbal_pitch_motor.absolute_angle,
                           Gimbal_Control_Set->Gimbal_Pitch_Control_Data);

  Gimbal_Control_Set->gimbal_pitch_motor.current_set =
      pid_calc(&Gimbal_Control_Set->Gimbal_Pitch_Speed_Pid,
               Gimbal_Control_Set->gimbal_pitch_motor.motor_gyro,
               Gimbal_Control_Set->Gimbal_Pitch_Apid_Out);

//	debug_angle[3] = yaw_delat_angle;

//  Gimbal_Control_Set->Gimbal_Yaw_Apid_Out = gimbal_PID_calc(&Gimbal_Control_Set->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid,\
//																						Gimbal_Control_Set->gimbal_yaw_motor.absolute_angle, \
//																						Gimbal_Control_Set->Gimbal_Yaw_Control_Data, Gimbal_Control_Set->gimbal_yaw_motor.motor_gyro);
//  Gimbal_Control_Set->gimbal_yaw_motor.current_set = PID_calc(&Gimbal_Control_Set->gimbal_yaw_motor.gimbal_motor_gyro_pid, Gimbal_Control_Set->gimbal_yaw_motor.motor_gyro, Gimbal_Control_Set->Gimbal_Yaw_Apid_Out);

  Gimbal_Control_Set->Gimbal_Yaw_Apid_Out =
      Gimbal_Math_pid_calc(&Gimbal_Control_Set->Gimbal_Yaw_Vision_Angle_Pid,
                           0.0f,
                           rad_format(Gimbal_Control_Set->Gimbal_Yaw_Control_Data - Gimbal_Control_Set->gimbal_yaw_motor.absolute_angle));

  Gimbal_Control_Set->Gimbal_Yaw_Apid_Out += Gimbal_Control_Set->pid_yaw_ffc.out;
  // Gimbal_Control_Set->Gimbal_Yaw_Apid_Out =
  //     Gimbal_Math_pid_calc(&Gimbal_Control_Set->Gimbal_Yaw_Vision_Angle_Pid,
  //                          Gimbal_Control_Set->gimbal_yaw_motor.relative_angle,
  //                          Gimbal_Control_Set->Gimbal_Yaw_Control_Data);

  Gimbal_Control_Set->gimbal_yaw_motor.current_set =
      pid_calc(&Gimbal_Control_Set->Gimbal_Yaw_Speed_Pid,
               Gimbal_Control_Set->gimbal_yaw_motor.motor_gyro,
               Gimbal_Control_Set->Gimbal_Yaw_Apid_Out);

  Gimbal_Control_Set->gimbal_yaw_motor.given_current = (int16_t)(Gimbal_Control_Set->gimbal_yaw_motor.current_set);
  Gimbal_Control_Set->gimbal_pitch_motor.given_current = (int16_t)( Gimbal_Control_Set->gimbal_pitch_motor.current_set);
  //	Pitch_Angle_Get_Last = yaw_angle_get;
  //	Pitch_Angle_Get_Last = yaw_angle_get;

  //	Gimbal_Control_Set->Gimbal_Yaw_Apid_Out = \
//	Gimbal_pid_calc(&Gimbal_Control_Set->Gimbal_Yaw_Motor_Angle_Pid,\
//	Gimbal_Control_Set->Gimbal_Yaw_Msg_t.Gimbal_Motor_Angle_TM,Gimbal_Control_Set->Gimbal_Yaw_Control_Data);

  //	Gimbal_Control_Set->Gimbal_Motor_Current_Send[0] = \
//	pid_calc(&Gimbal_Control_Set->Gimbal_Yaw_Speed_Pid,\
//	Gimbal_Control_Set->Gimbal_Yaw_Msg_t.Gimbal_IMU_Aspeed,Gimbal_Control_Set->Gimbal_Yaw_Apid_Out);
}
