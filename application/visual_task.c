/**
  ****************************(C) COPYRIGHT 2023 ULTRA****************************
  * @file       visual_task.c/h
  * @brief      FreeRTOS���񣬽����Ӿ�����
  *
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-2-20     	����             1.done.
  *  V1.0.1     2023-5-2       	����             2.standard.
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 ULTRA****************************
  */

#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "visual_task.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "referee.h"
#include "chassis_behaviour.h"
#include "user_lib.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "CRC8_CRC16.h"
#include "fire_control_system.h"

uint8_t dat[MINIPC_SENDLENGTH];
ext_gimbal_CV_ctrl_t CV_EV; // ����ṹ���������
void myprintf(const char *fmt, ...);
void McuToNuc(void);
void NucToMcu(uint8_t *Buf, const uint32_t *Len);
void Parse_data(void);
static void get_math_information(void);
static void clear_data(void);
void visual_task_init(void);
static void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z);

uint8_t visual_rx_buffer[sizeof(CV_R_t)]; // �������ݻ�������
uint8_t visual_rx_buffer1[sizeof(CV_R_t)];
uint8_t last_CV_EV_state = 0;

CV_R_t CV_R;
CV_T_t CV_T;

uint16_t cv_lost_counter = 0;
uint16_t delay_counter = 0;

uint16_t cv_rx_data_len = 0;
uint8_t cv_rx_flag = 0;
uint16_t cv_rx_counter = 0;

uint32_t cv_now_bias_time = 0;
uint32_t cv_last_bias_time = 0;

struct SolveTrajectoryParams cv_st;
struct tar_pos tar_position[4]; // ���ֻ���Ŀ�װ�װ�
float t_r = 0.2f;               // ����ʱ��
uint8_t select_idx = 0;

uint32_t fps_counter = 0;
uint32_t time_counter = 0;
float fps = 0.0f;
uint32_t fps_counter_x = 0;
float fps_x = 0.0f;
float packet_lose_rate = 0.0f;

uint8_t visual_rx_buffer_last[sizeof(CV_R_t)]; // �������ݻ�������

uint32_t debug_now_time = 0;
uint32_t debug_last_time = 0;
uint32_t debug_delta_time = 0;

void visual_reception(void const *argument)
{
  vTaskDelay(VISUAL_RECEPTION_INIT_TIME);
  visual_task_init();
  while (1)
  {
    McuToNuc(); // ��Nuc��������
    // NucToMcu();  //���ڽ��ղ���������3
    time_counter++;
    if (time_counter == 500)
    {
      debug_now_time = HAL_GetTick();
      time_counter = 0;
      fps = fps_counter;
      fps_counter = 0;
      fps_x = fps_counter_x;
      fps_counter_x = 0;
      debug_delta_time = debug_now_time - debug_last_time;
      debug_last_time = debug_now_time;
      if (fps_x != 0)
      {
        packet_lose_rate = (fps_x - fps) / fps_x;
      }
    }
    //	CV_EV.state = 2;
    vTaskDelay(VISUAL_RECEPTION_WAIT_TIME);
  }
}

HAL_StatusTypeDef y;

/// @brief ��Ƭ��ͨ��������NUC�����������
/// @param state ��ǰ��������Ϣ
/// @param q0 ��������Ԫ��֮һ
/// @param q1 ��������Ԫ��֮һ
/// @param q2 ��������Ԫ��֮һ
/// @param q3 ��������Ԫ��֮һ
/// @param timestamp �����Ƿ�����ʱ���
void McuToNuc()
{
  // ͷ֡��ֵ
  CV_T.header = CV_SOF;

  // ��ɫ��Ϣ��ֵ
  get_math_information();

  // ŷ���Ǹ�ֵ
  CV_T.pitch = chassis_move.chassis_INS_angle[1];
  CV_T.roll = chassis_move.chassis_INS_angle[2];
  CV_T.yaw = chassis_move.chassis_INS_angle[0];

  CV_T.reset_tracker = 0;

  CV_T.reserved = 1;

  CV_T.aim_x = cv_st.aim_x;
  CV_T.aim_y = cv_st.aim_y;
  CV_T.aim_z = cv_st.aim_z;

  // CRCУ��
  CV_T.checksum = get_CRC16_check_sum((uint8_t *)&CV_T, MINIPC_SENDLENGTH - 2, 0xffff);
  // Append_CRC16_Check_Sum(&CV_T, MINIPC_SENDLENGTH - 2, CV_SEND);
  memcpy(dat, (uint8_t *)&CV_T, MINIPC_SENDLENGTH);

  CDC_Transmit_FS(dat, MINIPC_SENDLENGTH);
  // DMA����
  // y = HAL_UART_Transmit(&huart1, dat, sizeof(CV_T_t), 1000);
}

/// @brief ��Ƭ������NUCͨ�����ڷ�����������

void NucToMcu(uint8_t *Buf, const uint32_t *Len)
{
  if (verify_CRC16_check_sum(Buf, *Len) != true)
  {
    return;
  }
	fps_counter_x++;
  if (Buf[0] == 0xA5)
  {
		fps_counter++;
    /* store the receive data */
    memcpy(&CV_R, Buf, *Len);
		Parse_data();
  }
}

fp32 debug_error_angle[2];
fp32 fast_conpen[2] = {CV_YAW_CONPEN,CV_PITCH_CONPEN};
fp32 cv_buff[2] = {0};
fp32 visual_control_time = VISION_CONTROL_TIME;
/// @brief ��������;
void Parse_data()
{
  float aim_x = 0.0f, aim_y = 0.0f, aim_z = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
  fp32 attenuation_plus = 0.2f;//˥��ϵ���ɵ���Ҫ��
	
  cv_st.xw = CV_R.x;
  cv_st.yw = CV_R.y;
  cv_st.zw = CV_R.z;
  cv_st.vzw = CV_R.vx;
  cv_st.vyw = CV_R.vy;
  cv_st.vzw = CV_R.vz;
  cv_st.armor_id = CV_R.id;
  cv_st.dz = CV_R.dz;
  cv_st.r1 = CV_R.r1;
  cv_st.r2 = CV_R.r2;
  cv_st.v_yaw = CV_R.v_yaw;
  cv_st.tar_yaw = CV_R.yaw;
  cv_st.armor_num = CV_R.armors_num;
  cv_st.current_pitch = chassis_move.chassis_INS_angle[1];
  cv_st.current_yaw = chassis_move.chassis_INS_angle[0];

  float x_armor, y_armor, armor_distance = 0.0f;
  float allow_error_distance = 0.0f;
  float allow_yaw_error = 0.0f;
  float yaw_diff, min_yaw_diff = 0.0f;
  float armorlock_yaw = 0.0f;

  // ��ȡ����ʱ����bias_time
  cv_now_bias_time = HAL_GetTick();

  // Ԥ��ʱ����Ҫ���Ͽ���ʱ�䣬Ϊ1/������/1000.0f
  cv_st.bias_time = PC_ABS(cv_now_bias_time - cv_last_bias_time) / 1000.0f + visual_control_time / 1000.0f + t_r;
	
  cv_last_bias_time = cv_now_bias_time;


  // �Ӿ����ݽ���
  autoSolveTrajectory(&pitch, &yaw, &aim_x, &aim_y, &aim_z);

  x_armor = tar_position[select_idx].x;
  y_armor = tar_position[select_idx].y;

  allow_error_distance = (CV_R.armors_num == 2 || CV_R.id == 1) ? LargeArmor_HalfWidth : LittleArmor_HalfWidth;

  // ����Ŀ�����
  armor_distance = sqrt(x_armor * x_armor + y_armor * y_armor);
	
  armorlock_yaw = atan2(aim_y, aim_x);
	
  //�����������Ƕ�
  allow_yaw_error =  (CV_R.armors_num == 2 || CV_R.id == 1) ? CV_YAW_LARGE_ERROR : CV_YAW_LITTLE_ERROR;//CV_YAW_ERROR����������װ�װ�/Сװ�װ����ò�ͬ������Ƕȣ���Ȼ��Ҳ����ֱ�Ӷ���ΪCV_YAW_ERROR
  //����ΪCV_YAW_ERROR���ô���Сװ�װ���ã���װ�װ嵱Ȼ�ܴ��У�Ҫ�Ƕ���������дΪallow_yaw_error = CV_YAW_ERROR;
  //��������װ�װ��������һЩ��������д������������Ӣ�۶��ԣ���Ҫ��׼�ȸ���һЩ������CV_YAW_LARGE_ERROR��CV_YAW_LITTLE_ERRORΪһ��ֵ���Ƽ�0.0f~0.025f


  /* calculate the yaw angle diff */
  yaw_diff = armorlock_yaw - cv_st.current_yaw;

  /* calculate the Minimum absolute yaw angle diff */
  min_yaw_diff = fabsf(CV_EV.yaw) > PI ? (CV_EV.yaw / fabsf(CV_EV.yaw) * 2 * PI - yaw_diff) : (CV_EV.yaw);

  debug_error_angle[0] = allow_yaw_error;
  debug_error_angle[1] = min_yaw_diff;

	cv_st.aim_x = aim_x;
	cv_st.aim_y = aim_y;
	cv_st.aim_z = aim_z;
	
	/*
	    //˳ʱ�룬������ < 0.0f����ȷ��
    if(cv_st.v_yaw > 0.0f)
    {
        if(!(DAMP_RANGE_CLOCK_WISE_MIN < tar_position[select_idx].yaw && tar_position[select_idx].yaw < DAMP_RANGE_CLOCK_WISE_MAX)) //˳ʱ��˥���Ƕȷ�Χ
        {
            allow_yaw_error = allow_yaw_error / 2.0f - PC_ABS(tar_position[select_idx].yaw * attenuation_plus); //�ȶ԰�˥��������ϵ��˥��
        }
    }
    //��ʱ��
    else if(cv_st.v_yaw < 0.0f)
    {
        if(!(DAMP_RANGE_CLOCK_ANTI_WISE_MIN < tar_position[select_idx].yaw && tar_position[select_idx].yaw < DAMP_RANGE_CLOCK_ANTI_WISE_MAX)) //��ʱ��˥���Ƕȷ�Χ
        {
            allow_yaw_error = allow_yaw_error / 2.0f - PC_ABS(tar_position[select_idx].yaw * attenuation_plus); //�ȶ԰�˥��������ϵ��˥��
        }
    }

    //���⣬�и���THERSHOLD_V_YAW�ĳ��������Գ����޸ģ��Ƽ�ֵ��0.1~1.5;���ֵӦ���ǹ���ƽ��/��תװ�װ�ѡ���Ĳ���

	*/
	if (CV_R.tracking == 1 && PC_ABS(CV_EV.pitch) <= CV_PITCH_ERROR && CV_EV.yaw  < CV_YAW_ERROR)//0.02f
	{
		CV_EV.now_state = 2;
	}
	else if (CV_R.tracking == 1 && !(PC_ABS(CV_EV.pitch) <= CV_PITCH_ERROR && CV_EV.yaw < CV_YAW_ERROR))
	{
		CV_EV.now_state = 1;
	}
	else
	{
		CV_EV.now_state = 0;
	}

  if (CV_EV.now_state > 0)
  {
    CV_EV.lost_flag = 0;
    cv_lost_counter = 0;
    CV_EV.state = CV_EV.now_state;
  }

  // Ŀ��ͻȻ��ʧ
  if (CV_EV.last_state != CV_EV.now_state && CV_EV.now_state == 0)
  {
    CV_EV.lost_flag = 1;
  }
  // ��ʧ�ﵽ����ֵ200
  else if (CV_EV.now_state == 0 && CV_EV.lost_flag == 1)
  {
    cv_lost_counter++;
    // �ж�Ϊ���ٶ�ʧ
    if (cv_lost_counter >= 150)
    {
      CV_EV.state = 0;
      cv_lost_counter = 0;
			CV_EV.lost_flag = 0;
    }
  }

  // cv_st����ʶ���װ�װ�id�ţ������������
  if (CV_EV.state > 0)
  {
    CV_EV.pitch = -(cv_st.current_pitch - pitch) + fast_conpen[1];
    CV_EV.yaw = -(cv_st.current_yaw - yaw) + fast_conpen[0];
    if (CV_EV.last_pitch != CV_EV.last_pitch || CV_EV.last_yaw != CV_EV.yaw)
    {
      CV_EV.valid = 1;
    }
    else
    {
      CV_EV.valid = 0;
    }

    CV_EV.last_pitch = CV_EV.pitch;
    CV_EV.last_yaw = CV_EV.yaw;
    CV_EV.per_yaw = CV_EV.last_yaw;
  }
  else
  {
    CV_EV.pitch = 0.0f;
    CV_EV.yaw = 0.0f;
    CV_EV.valid = 0;
  }

	if(CV_EV.lost_flag == 1)
	{
		CV_EV.state = 1;
		CV_EV.yaw = 0.0f;
		CV_EV.pitch = 0.0f;
	}
  CV_EV.last_state = CV_EV.now_state;
}

ext_gimbal_CV_ctrl_t *get_cv_msg_point(void)
{
  return &CV_EV;
}

/**
 * @brief ��ȡ������Ϣ
 * @param None.
 * @retval None.
 */
static void get_math_information()
{
//    	if(sentinel_side == RED)
//      {
  			CV_T.detect_color = 1;
//      }
//    	else if(sentinel_side == BLUE)
//    	{
//    		CV_T.detect_color = 0;
//			}
//		CV_T.detect_color = 1;
}


/*
@brief �����������������ģ��
@param s:m ����
@param v:m/s �ٶ�
@param angle:rad �Ƕ�
@return z:m
*/
float monoDirectionalAirResistanceModel(float s, float v, float angle)
{
  float z;
  // tΪ����v��angleʱ�ķ���ʱ��
  t_r = (float)((exp(cv_st.k * s) - 1) / (cv_st.k * v * cos(angle)));
  // zΪ����v��angleʱ�ĸ߶�
  z = (float)(v * sin(angle) * t_r - GRAVITY * t_r * t_r / 2.0f);
  // printf("model %f %f\n", t, z);
  return z;
}

/*
@brief pitch�����
@param s:m ����
@param z:m �߶�
@param v:m/s
@return angle_pitch:rad
*/
float pitchTrajectoryCompensation(float s, float z, float v)
{
  float z_temp, z_actual, dz;
  float angle_pitch;
  int i = 0;
  z_temp = z;
  if (s < 0)
  {
    return 0;
  }
  // iteration
  for (i = 0; i < 20; i++)
  {
    angle_pitch = atan2(z_temp, s); // rad
    z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
    dz = 0.3f * (z - z_actual);
    z_temp = z_temp + dz;
    // printf("iteration num %d: angle_pitch %f, temp target z:%f, err of z:%f, s:%f\n",
    //     i + 1, angle_pitch * 180 / PI, z_temp, dz,s);
    if (fabsf(dz) < 0.00001f)
    {
      break;
    }
  }
  return angle_pitch;
}

float pitch_x;
float pitch_y;
/*
@brief �������ž��ߵó�������װ�װ� �Զ����㵯��
@param pitch:rad  ����pitch
@param yaw:rad    ����yaw
@param aim_x:����aim_x  ���Ŀ���x
@param aim_y:����aim_y  ���Ŀ���y
@param aim_z:����aim_z  ���Ŀ���z
*/
static void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z)
{

  // ����Ԥ��
  float timeDelay = cv_st.bias_time / 1000.0 + t_r;
  cv_st.tar_yaw += cv_st.v_yaw * timeDelay;

  // �����Ŀ�װ�װ��λ��
  // װ�װ�id˳��,���Ŀ�װ�װ�Ϊ��,��ʱ����
  //       2
  //    3     1
  //       0
  int use_1 = 1;
  int i = 0;
  int idx = 0; // ѡ���װ�װ�
	
  // armor_num = ARMOR_NUM_BALANCE Ϊƽ�ⲽ��
  if (cv_st.armor_num == ARMOR_NUM_BALANCE)
  {
    for (i = 0; i < 2; i++)
    {
      float tmp_yaw = cv_st.tar_yaw + i * PI;
      float r = cv_st.r1;
      tar_position[i].x = cv_st.xw - r * cos(tmp_yaw);
      tar_position[i].y = cv_st.yw - r * sin(tmp_yaw);
      tar_position[i].z = cv_st.zw;
      tar_position[i].yaw = tmp_yaw;
    }

    float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);

    // ��Ϊ��ƽ�ⲽ�� ֻ���ж�����װ�װ�
    float temp_yaw_diff = fabsf(*yaw - tar_position[1].yaw);
    if (temp_yaw_diff < yaw_diff_min)
    {
      yaw_diff_min = temp_yaw_diff;
      idx = 1;
    }
  }
  else if (cv_st.armor_num == ARMOR_NUM_OUTPOST)
  { // ǰ��վ
    for (i = 0; i < 3; i++)
    {
      float tmp_yaw = cv_st.tar_yaw + i * 2.0 * PI / 3.0; // 2/3PI
      float r = (cv_st.r1 + cv_st.r2) / 2;                // ������r1=r2 ����ȡ��ƽ��ֵ
      tar_position[i].x = cv_st.xw - r * cos(tmp_yaw);
      tar_position[i].y = cv_st.yw - r * sin(tmp_yaw);
      tar_position[i].z = cv_st.zw;
      tar_position[i].yaw = tmp_yaw;
    }

    // TODO ѡ������װ�װ� ѡ���߼������Լ�д��һ���Ӣ����
  }
  else
  {

    for (i = 0; i < 4; i++)
    {
      float tmp_yaw = cv_st.tar_yaw + i * PI / 2.0f;
      float r = use_1 ? cv_st.r1 : cv_st.r2;
      tar_position[i].x = cv_st.xw - r * cos(tmp_yaw);
      tar_position[i].y = cv_st.yw - r * sin(tmp_yaw);
      tar_position[i].z = use_1 ? cv_st.zw : cv_st.zw + cv_st.dz;
      tar_position[i].yaw = tmp_yaw;
      use_1 = !use_1;
    }

    // 2�ֳ������߷���:
    // 1.����ǹ�ܵ�Ŀ��װ�װ�yaw��С��װ�װ�
    // 2.������������װ�װ�

    // ������������װ�װ�
	float dis_diff_min = sqrt(tar_position[0].x * tar_position[0].x + tar_position[0].y * tar_position[0].y);
	int idx = 0;
			for (i = 1; i<4; i++)
	{
		float temp_dis_diff = sqrt(tar_position[i].x * tar_position[0].x + tar_position[i].y * tar_position[0].y);
		if (temp_dis_diff < dis_diff_min)
		{
			dis_diff_min = temp_dis_diff;
			idx = i;
		}
	}

    // ����ǹ�ܵ�Ŀ��װ�װ�yaw��С��װ�װ�
//    float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);
//    for (i = 1; i < 4; i++)
//    {
//      float temp_yaw_diff = fabsf(*yaw - tar_position[i].yaw);
//      if (temp_yaw_diff < yaw_diff_min)
//      {
//        yaw_diff_min = temp_yaw_diff;
//        idx = i;
//      }
//    }
  }

	/*
	if(PC_ABS(cv_st.v_yaw) < THERSHOLD_V_YAW)
	{
		select_idx = 0;
		idx = 0;
	}
	else
	{
		select_idx = idx;
	}
	*/
	select_idx = idx;
	*aim_z = tar_position[idx].z + cv_st.vzw * timeDelay;
	*aim_x = tar_position[idx].x + cv_st.vxw * timeDelay;
	*aim_y = tar_position[idx].y + cv_st.vyw * timeDelay;
	
  // ע��pitch��yaw�ķ���
  *pitch = -pitchTrajectoryCompensation(sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)) - cv_st.s_bias,
                                        *aim_z + cv_st.z_bias, cv_st.current_v);
  pitch_x = (*aim_x) * (*aim_x) + (*aim_y) * (*aim_y);
  *yaw = (float)(atan2(*aim_y, *aim_x));
}

// ��������������ԭ�㣬��ʱ�뷽��Ϊ��
/**
 * @brief �Ӿ������ʼ��
 * @param None.
 * @retval None.
 */
void visual_task_init()
{
  // �������
  cv_st.k = 0.092f;
  cv_st.bullet_type = BULLET_17;
  cv_st.current_v = 24.5f;
  cv_st.current_pitch = chassis_move.chassis_INS_angle[1];
  cv_st.current_yaw = chassis_move.chassis_INS_angle[0];
  cv_st.bias_time = VISUAL_RECEPTION_WAIT_TIME / 1000.0f; // ����ƫ��ʱ�䣬�Ӿ���Ч���ݷ��͵�Ƶ�ʣ����õ�������Ƶ��
  cv_st.s_bias = 0.192f;                                 // 0.18258f
  cv_st.z_bias = 0.0f;                                 // -0.0351ff
  cv_st.armor_id = ARMOR_INFANTRY3;
  cv_st.armor_num = ARMOR_NUM_NORMAL;

  CV_EV.now_state = 0;
  CV_EV.last_state = 0;
  CV_EV.per_yaw = 0;
  CV_EV.last_yaw = 0;
	
	MX_USB_DEVICE_Init();
}
