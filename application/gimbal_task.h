/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ���������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
	*  V2.0.0     2023-9-18       ����             4.���ƣ��޸ģ�����  
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
// pitch speed close-loop PID params, max out and max iout
// pitch �ٶȻ� PID�����Լ� PID���������������
#define PITCH_SPEED_PID_KP 2000.0f
#define PITCH_SPEED_PID_KI 20.0f
#define PITCH_SPEED_PID_KD 0.0f
#define PITCH_SPEED_PID_MAX_OUT 25000.0f
#define PITCH_SPEED_PID_MAX_IOUT 10000.0f

// yaw speed close-loop PID params, max out and max iout
// yaw �ٶȻ� PID�����Լ� PID���������������
#define YAW_SPEED_PID_KP 1000.0f
#define YAW_SPEED_PID_KI 0.0f
#define YAW_SPEED_PID_KD 0.0f
#define YAW_SPEED_PID_MAX_OUT 20000.0f//30000
#define YAW_SPEED_PID_MAX_IOUT 5000.0f

// pitch gyro angle close-loop PID params, max out and max iout
// pitch �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������
#define PITCH_GYRO_ABSOLUTE_PID_KP 10.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.0f

#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

// yaw gyro angle close-loop PID params, max out and max iout
// yaw �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������
#define YAW_GYRO_ABSOLUTE_PID_KP 12.0f // 20.0f
#define YAW_GYRO_ABSOLUTE_PID_KI 0.0f // 0.1f
#define YAW_GYRO_ABSOLUTE_PID_KD 0.3f // 0.5f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT 1.0f

// pitch encode angle close-loop PID params, max out and max iout
// pitch �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define PITCH_ENCODE_RELATIVE_PID_KP 10.0f
#define PITCH_ENCODE_RELATIVE_PID_KI 0.00f
#define PITCH_ENCODE_RELATIVE_PID_KD 0.0f

#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

// yaw encode angle close-loop PID params, max out and max iout
// yaw �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define YAW_ENCODE_RELATIVE_PID_KP 20.0f
#define YAW_ENCODE_RELATIVE_PID_KI 0.05f
#define YAW_ENCODE_RELATIVE_PID_KD 0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT 2.0f

//Ŀǰʹ�õ�pid 
//===================================================================
//ǰ��
#define GIMBAL_YAW_PID_FFC_Ka      0.0f//һ��//800//605//750
#define GIMBAL_YAW_PID_FFC_Kb      0.0f//����//200//100//180

#define Gimbal_Yaw_Vision_Angle_Kp 28.1f	   // 20//50//30//
#define Gimbal_Yaw_Vision_Angle_Ki 0.0f	   // 
#define Gimbal_Yaw_Vision_Angle_Kd 2800.0f	   // 1200//25//60
#define Gimbal_Yaw_Vision_Angle_Maxout 20.0f  // 10
#define Gimbal_Yaw_Vision_Angle_IMaxout 2.0f // 10

#define Gimbal_Pitch_Vision_Angle_Kp 50.0f // 12//20
#define Gimbal_Pitch_Vision_Angle_Ki 0.0f // 0
#define Gimbal_Pitch_Vision_Angle_Kd 0.0f //800//0
#define Gimbal_Pitch_Vision_Angle_Maxout 10.0f	 // 10
#define Gimbal_Pitch_Vision_Angle_IMaxout 10.0f // 10

#define Gimbal_Yaw_Speed_Kp 6500.0f	  // 4800//2700//3000
#define Gimbal_Yaw_Speed_Ki 0.0f		  // 0
#define Gimbal_Yaw_Speed_Kd 200.0f		  // 0//1250
#define Gimbal_Yaw_Speed_Maxout 25000.0f // 2500
#define Gimbal_Yaw_Speed_IMaxout 5000.0f // 5000

#define Gimbal_Pitch_Speed_Kp 8500.0f	// 5200
#define Gimbal_Pitch_Speed_Ki 20.0f      // 0
#define Gimbal_Pitch_Speed_Kd 0.0f		// 0
#define Gimbal_Pitch_Speed_Maxout 25000.0f // 2500
#define Gimbal_Pitch_Speed_IMaxout 5000.0f // 5000

// // ����pitch �ٶȻ� PID�����Լ� PID���������������
// #define PITCH_SPEED_PID_KP_t 2900.0f
// #define PITCH_SPEED_PID_KI_t 60.0f
// #define PITCH_SPEED_PID_KD_t 0.0f
// #define PITCH_SPEED_PID_MAX_OUT_t 30000.0f
// #define PITCH_SPEED_PID_MAX_IOUT_t 10000.0f

// // ����yaw �ٶȻ� PID�����Լ� PID���������������
// #define YAW_SPEED_PID_KP_t 3000.0f // 3600.0f
// #define YAW_SPEED_PID_KI_t 15.0f   // 20.0f
// #define YAW_SPEED_PID_KD_t 0.0f    // 0.0f
// #define YAW_SPEED_PID_MAX_OUT_t 30000.0f
// #define YAW_SPEED_PID_MAX_IOUT_t 5000.0f

// // ����pitch �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������
// #define PITCH_GYRO_ABSOLUTE_PID_KP_t 15.0f
// #define PITCH_GYRO_ABSOLUTE_PID_KI_t 0.0f
// #define PITCH_GYRO_ABSOLUTE_PID_KD_t 0.0f

// #define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT_t 10.0f
// #define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT_t 0.0f

// // ����yaw �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������
// #define YAW_GYRO_ABSOLUTE_PID_KP_t 1.0f // 20.0f  2.0f
// #define YAW_GYRO_ABSOLUTE_PID_KI_t 0.001f // 0.1f
// #define YAW_GYRO_ABSOLUTE_PID_KD_t 0.01f // 0.5f
// #define YAW_GYRO_ABSOLUTE_PID_MAX_OUT_t 10.0f
// #define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT_t 1.0f

// // ����pitch �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
// #define PITCH_ENCODE_RELATIVE_PID_KP_t 15.0f
// #define PITCH_ENCODE_RELATIVE_PID_KI_t 0.00f
// #define PITCH_ENCODE_RELATIVE_PID_KD_t 0.0f

// #define PITCH_ENCODE_RELATIVE_PID_MAX_OUT_t 10.0f
// #define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT_t 0.0f

// // ����yaw �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
// #define YAW_ENCODE_RELATIVE_PID_KP_t 20.0f
// #define YAW_ENCODE_RELATIVE_PID_KI_t 0.05f
// #define YAW_ENCODE_RELATIVE_PID_KD_t 0.0f
// #define YAW_ENCODE_RELATIVE_PID_MAX_OUT_t 10.0f
// #define YAW_ENCODE_RELATIVE_PID_MAX_IOUT_t 2.0f

//===================================================================

// �����ʼ�� ����һ��ʱ��
#define GIMBAL_TASK_INIT_TIME 201
// yaw,pitch����ͨ���Լ�״̬����ͨ��
#define YAW_CHANNEL 2
#define PITCH_CHANNEL 3
#define GIMBAL_MODE_CHANNEL 0

// turn 180��
// ��ͷ180 ����
#define TURN_KEYBOARD KEY_PRESSED_OFFSET_F
// turn speed
// ��ͷ��̨�ٶ�
#define TURN_SPEED 0.04f
// ���԰�����δʹ��
#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R
// rocker value deadband
// ң����������������Ϊң�������ڲ��죬ҡ�����м䣬��ֵ��һ��Ϊ��
#define RC_DEADBAND 10

#define YAW_RC_SEN -0.000005f
#define PITCH_RC_SEN -0.000006f // 0.005

#define YAW_MOUSE_SEN 0.00005f
#define PITCH_MOUSE_SEN 0.00015f

#define YAW_ENCODE_SEN 0.01f
#define PITCH_ENCODE_SEN 0.01f

#define GIMBAL_CONTROL_TIME 1

// test mode, 0 close, 1 open
// ��̨����ģʽ �궨�� 0 Ϊ��ʹ�ò���ģʽ
#define GIMBAL_TEST_MODE 0

//#define PITCH_TURN 1
#define YAW_TURN 0

// �������ֵ����Լ���ֵ
#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191
// ��̨��ʼ������ֵ����������,��������Χ��ֹͣһ��ʱ���Լ����ʱ��6s������ʼ��״̬��
#define GIMBAL_INIT_ANGLE_ERROR 0.1f
#define GIMBAL_INIT_STOP_TIME 100
#define GIMBAL_INIT_TIME 6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
// ��̨��ʼ������ֵ���ٶ��Լ����Ƶ��ĽǶ�
#define GIMBAL_INIT_PITCH_SPEED 0.004f
#define GIMBAL_INIT_YAW_SPEED 0.005f

#define INIT_YAW_SET 0.0f
#define INIT_PITCH_SET 0.0f

// ��̨У׼��ֵ��ʱ�򣬷���ԭʼ����ֵ���Լ���תʱ�䣬ͨ���������ж϶�ת
#define GIMBAL_CALI_MOTOR_SET 8000
#define GIMBAL_CALI_STEP_TIME 2000
#define GIMBAL_CALI_GYRO_LIMIT 0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP 1
#define GIMBAL_CALI_PITCH_MIN_STEP 2
#define GIMBAL_CALI_YAW_MAX_STEP 3
#define GIMBAL_CALI_YAW_MIN_STEP 4

#define GIMBAL_CALI_START_STEP GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP 5

// �ж�ң�����������ʱ���Լ�ң�����������жϣ�������̨yaw����ֵ�Է�������Ư��
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX 3000

// �������ֵת���ɽǶ�ֵ
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif

#define Yaw_Max_Angle PI // rad
#define Yaw_Min_Angle -PI
#define Pitch_Max_Angle 0.3f
#define Pitch_Min_Angle -0.3f

typedef enum
{
	GIMBAL_MOTOR_RAW = 0, // ���ԭʼֵ����
	GIMBAL_MOTOR_GYRO,	  // ��������ǽǶȿ���
	GIMBAL_MOTOR_ENCONDE, // �������ֵ�Ƕȿ���
} gimbal_motor_mode_e;

typedef struct
{
	fp32 kp;
	fp32 ki;
	fp32 kd;

	fp32 set;
	fp32 get;
	fp32 err;

	fp32 max_out;
	fp32 max_iout;

	fp32 Pout;
	fp32 Iout;
	fp32 Dout;

	fp32 out;
} gimbal_PID_t;

typedef struct
{
	const motor_measure_t *gimbal_motor_measure;
	gimbal_PID_t gimbal_motor_absolute_angle_pid;
	gimbal_PID_t gimbal_motor_relative_angle_pid;

	gimbal_PID_t gimbal_motor_absolute_angle_pid_t;
	gimbal_PID_t gimbal_motor_relative_angle_pid_t;

	pid_type_def gimbal_motor_gyro_pid;
	gimbal_motor_mode_e gimbal_motor_mode;
	gimbal_motor_mode_e last_gimbal_motor_mode;
	uint16_t offset_ecd;
	fp32 max_relative_angle; // rad
	fp32 min_relative_angle; // rad

	fp32 relative_angle;	 // rad
	fp32 relative_angle_set; // rad
	fp32 absolute_angle;	 // rad
	fp32 absolute_angle_set; // rad
	fp32 motor_gyro;		 // rad/s ���ٶ�
	fp32 motor_gyro_set;
	fp32 motor_speed;
	fp32 raw_cmd_current;
	fp32 current_set;
	int16_t given_current;

} gimbal_motor_t;

typedef struct
{
	// //��̨����������
	// Motor_Msg_t* Gimbal_Motor_Msg_Get;
	// �����ǽǶȻ�ȡ��ַ
	const float *Gimbal_IMU_Angle_Data;
	// �����ǽ��ٶȻ�ȡ��ַ
	const float *Gimbal_IMU_Aspeed_Data;
	// ��̨��е�Ƕ�ֵ
	float Gimbal_Motor_Angle_Msg;
	// ��̨�����ǽǶ�ֵ
	float Gimbal_IMU_Angle_Msg;
	// ��̨�Ƕ��е���Ϣ
	float Gimbal_Angle_Middle_Msg;
	// ��̨���ԽǶ�-��е�ǶȲ�
	float Gimbal_Motor_Angle_TM;
	// ��̨���ԽǶ�-�����ǽǶȲ�
	float Gimbal_IMU_Angle_Set;
	float Gimbal_Max_Angle;
	float Gimbal_Min_Angle;
	// ��̨���ٶ�-��е���ٶ�ֵ
	float Gimbal_Motor_Aspeed;
	// ��̨�Ƕ���-�����ǽ��ٶ�ֵ
	float Gimbal_IMU_Aspeed;
	// Ŀ��Ƕ�
	float Motor_Target_Angle;

} Gimbal_Msg_t;

typedef struct
{
	fp32 max_yaw;
	fp32 min_yaw;
	fp32 max_pitch;
	fp32 min_pitch;
	uint16_t max_yaw_ecd;
	uint16_t min_yaw_ecd;
	uint16_t max_pitch_ecd;
	uint16_t min_pitch_ecd;
	uint8_t step;
} gimbal_step_cali_t;

typedef struct
{
	const RC_ctrl_t *gimbal_rc_ctrl;
	const fp32 *gimbal_INT_angle_point;
	const fp32 *gimbal_INT_gyro_point;
	gimbal_motor_t gimbal_yaw_motor;
	gimbal_motor_t gimbal_pitch_motor;
	gimbal_step_cali_t gimbal_cali;
	
	PID_FFC pid_yaw_ffc;   
	// CVison
	// ��ȡ�ڱ��˶�ģʽ
	//  const Sentry_Mode_t* Sentry_Mode_Gimbal_Get;
	//  //��ȡ�ڱ��Զ��˶�ģʽ
	//  const Sentry_Auto_Mode_t* Sentry_Auto_Mode_Gimbal_Get;

	// ��̨ģʽ
	//  Gimbal_Mode_t Gimbal_Mode;
	// �Ӿ��ӽ�
	//  Gimbal_Now_Gimbal_Target_t Gimbal_Now_Gimbal_Target;

	/**********��̨���PID�ṹ��**********/
	// ��̨���ԽǶȼ���PID-��е�Ƕ�
	PID Gimbal_Yaw_Motor_Angle_Pid;
	PID Gimbal_Pitch_Motor_Angle_Pid;
	// ��̨���ԽǶȼ���PID-�����ǽǶ�
	PID Gimbal_Yaw_IMU_Angle_Pid;
	PID Gimbal_Pitch_IMU_Angle_Pid;
	// ��̨�Ӿ�����PID-�����ǽǶ�
	PID Gimbal_Yaw_Vision_Angle_Pid;   //
	PID Gimbal_Pitch_Vision_Angle_Pid; //

	// ��̨���ٶȼ���PID
	PID Gimbal_Yaw_Speed_Pid;	//
	PID Gimbal_Pitch_Speed_Pid; //

	// ��̨����PID
	PID Gimbal_Yaw_Motor_Pid;
	PID Gimbal_Pitch_Motor_Pid;

	/**********��̨�����Ϣ�ṹ��**********/
	Gimbal_Msg_t Gimbal_Yaw_Msg_t;
	Gimbal_Msg_t Gimbal_Pitch_Msg_t; //

	// ������������ݻ�ȡ
	const float *Gimbal_IMU_Angle;
	const float *Gimbal_IMU_Aspeed;
	// //ң��������ֵ��ȡ
	// const RC_Ctl_t* Gimbal_RC_Ctl_Data;

	// float Gimbal_Yaw_RC_Data;
	// float Gimbal_Pitch_RC_Data;

	float Gimbal_Yaw_Add_Data;	 //
	float Gimbal_Pitch_Add_Data; //
	// ��̨����ֵ��ȡ
	float Gimbal_Yaw_Control_Data;	 //
	float Gimbal_Pitch_Control_Data; //
	// ��̨���ƽǶȲ�-���ģʽ
	float Gimbal_Control_Motor_Angle_TM[2];
	// ��̨���ƽǶ�-IMUģʽ
	float Gimbal_Control_IMU_Angle[2];
	// ��̨����Ƕ�PID���-���ٶ�PID����
	float Gimbal_Yaw_Apid_Out;	 //
	float Gimbal_Pitch_Apid_Out; //

	// �Ӿ���Ϣ
	float Gimbal_Yaw_Vision_Data[5];
	float Gimbal_Pitch_Vision_Data[5];
	float Gimbal_Vision_Flag[5];

	float Gimbal_Yaw_Vision_Data_Target; //
	float Gimbal_Pitch_Vision_Data_Target;
	int Gimbal_Vision_Flag_Target;

	// ��̨Ŀ��Ƕ�
	float Motor_Yaw_Target_Angle;
	float Motor_Pitch_Target_Angle;

	// Gimbal_Judge_Msg_t Gimbal_Judge_Msg;

	// //��̨���͵���ֵ
	// float Gimbal_Motor_Current_Send[2];

} gimbal_control_t;

extern gimbal_control_t gimbal_control;

/**
 * @brief          return yaw motor data point
 * @param[in]      none
 * @retval         yaw motor data point
 */
/**
 * @brief          ����yaw �������ָ��
 * @param[in]      none
 * @retval         yaw���ָ��
 */
extern const gimbal_motor_t *get_yaw_motor_point(void);

/**
 * @brief          return pitch motor data point
 * @param[in]      none
 * @retval         pitch motor data point
 */
/**
 * @brief          ����pitch �������ָ��
 * @param[in]      none
 * @retval         pitch
 */
extern const gimbal_motor_t *get_pitch_motor_point(void);

/**
 * @brief          gimbal task, osDelay GIMBAL_CONTROL_TIME (1ms)
 * @param[in]      pvParameters: null
 * @retval         none
 */
/**
 * @brief          ��̨���񣬼�� GIMBAL_CONTROL_TIME 1ms
 * @param[in]      pvParameters: ��
 * @retval         none
 */

extern void gimbal_task(void const *pvParameters);

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
 * @brief          ��̨У׼���㣬��У׼��¼����ֵ,��� ��Сֵ����
 * @param[out]     yaw ��ֵ ָ��
 * @param[out]     pitch ��ֵ ָ��
 * @param[out]     yaw �����ԽǶ� ָ��
 * @param[out]     yaw ��С��ԽǶ� ָ��
 * @param[out]     pitch �����ԽǶ� ָ��
 * @param[out]     pitch ��С��ԽǶ� ָ��
 * @retval         ����1 ����ɹ�У׼��ϣ� ����0 ����δУ׼��
 * @waring         �������ʹ�õ�gimbal_control ��̬�������º�������������ͨ��ָ�븴��
 */
extern bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);

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
 * @brief          ��̨У׼���ã���У׼����̨��ֵ�Լ���С����е��ԽǶ�
 * @param[in]      yaw_offse:yaw ��ֵ
 * @param[in]      pitch_offset:pitch ��ֵ
 * @param[in]      max_yaw:max_yaw:yaw �����ԽǶ�
 * @param[in]      min_yaw:yaw ��С��ԽǶ�
 * @param[in]      max_yaw:pitch �����ԽǶ�
 * @param[in]      min_yaw:pitch ��С��ԽǶ�
 * @retval         ���ؿ�
 * @waring         �������ʹ�õ�gimbal_control ��̬�������º�������������ͨ��ָ�븴��
 */
extern void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);

extern void gimbal_track_enemy_relative_angle_limit(gimbal_motor_t *gimbal_motor, float record_angle);

extern float track_record_yaw_angle;
extern float track_record_pitch_angle;
extern float Yaw_Vision_Plus;

#endif
