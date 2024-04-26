/**
  ******************************************************************************
  * @file    protocol.c
  * @version V1.0
  * @date    2020-xx-xx
  * @brief   Ұ��PID��������ͨѶЭ�����
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� F407 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 

#include "bsp_debug_pid.h"
#include <string.h>

struct prot_frame_parser_t
{
    uint8_t *recv_ptr;
    uint16_t r_oft;
    uint16_t w_oft;
    uint16_t frame_len;
    uint16_t found_frame_head;
};

struct prot_frame_parser_t parser;

uint8_t debug_rx_buff[PROT_FRAME_LEN_RECV];

uint8_t debug_pid_state_flag = 0;
uint8_t debug_pid_count = 0;
uint8_t debug_pid_rx_data_flag;
uint8_t debug_pid_rx_data = 0;
uint8_t debug_pid_set_count = 0;
uint8_t channel_enable[2] = {0};

static void debug_normal_data_dispose(void);

static void debug_pid_data_dispose(void);

float p_temp = 0.0f, i_temp = 0.0f, d_temp = 0.0f;
float px_temp = 0.0f,ix_temp = 0.0f,dx_temp = 0.0f;
float py_temp = 0.0f,iy_temp = 0.0f,dy_temp = 0.0f;
float debug_x_set[2] = {0};
float debug_x_ref[2] = {0};

void debug_pid(void const *pvParameters)
{
	vTaskDelay(350);
	HAL_UART_Receive_IT(&UartHandle,&debug_pid_rx_data,1);
	debug_pid_state_flag = 1;
	while(1)
	{
		if(debug_pid_state_flag == 0xA0)
		{
			if(channel_enable[0] == 1)
			{
				float pid_temp[3] = {px_temp,ix_temp,dx_temp};			
				set_computer_value(SEND_P_I_D_CMD,CURVES_CH1,pid_temp,3);   //��ͨ�� 1 ����PID�趨ֵ
				uint32_t temp_period[1] = {PID_CONTROL_TIME};
				set_computer_value(SEND_PERIOD_CMD,CURVES_CH1,temp_period,1);
				int32_t debug_set_temp = debug_x_set[0] * MAGNIFY_REF_SET;
				int32_t debug_ref_temp = debug_x_ref[0] * MAGNIFY_REF_SET;
				set_computer_value(SEND_TARGET_CMD, CURVES_CH1,&debug_set_temp, 1);     // ��ͨ�� 1 ����Ŀ��ֵ
				set_computer_value(SEND_FACT_CMD,CURVES_CH1,&debug_ref_temp,1);         //��ͨ�� 1 ����ʵ��ֵ
			}
			if(channel_enable[1] == 1)
			{
				float pid_temp_x[3] = {py_temp,iy_temp,dy_temp};			
				set_computer_value(SEND_P_I_D_CMD,CURVES_CH2,pid_temp_x,3);   //��ͨ�� 1 ����PID�趨ֵ
				uint32_t temp_period_x[1] = {PID_CONTROL_TIME};
				set_computer_value(SEND_PERIOD_CMD,CURVES_CH2,temp_period_x,1);
				int32_t debug_set_temp_x = debug_x_set[1] * MAGNIFY_REF_SET_X;
				int32_t debug_ref_temp_x = debug_x_ref[1] * MAGNIFY_REF_SET_X;
				set_computer_value(SEND_TARGET_CMD, CURVES_CH2,&debug_set_temp_x, 1);     // ��ͨ�� 1 ����Ŀ��ֵ
				set_computer_value(SEND_FACT_CMD,CURVES_CH2,&debug_ref_temp_x,1);         //��ͨ�� 1 ����ʵ��ֵ				
			}
		}

		vTaskDelay(PID_WAIT_TIME);
	}
}
/**
  * @brief ����У���
  * @param ptr����Ҫ���������
  * @param len����Ҫ����ĳ���
  * @retval У���
  */
uint8_t check_sum(uint8_t init, uint8_t *ptr, uint8_t len )
{
  uint8_t sum = init;
  
  while(len--)
  {
    sum += *ptr;
    ptr++;
  }
  
  return sum;
}



/**
 * @brief   ���յ����ݴ���
 * @param   PID����,�趨ֵ������ֵ��ͨ��
 * @return  -1��û���ҵ�һ����ȷ������.
 */
void debug_pid_dispose(pid_type_def *pid_control,fp32 *ref,fp32 *set,uint8_t channel)
{
	if(channel == 1)
	{
		px_temp = pid_control->Kp;
		ix_temp = pid_control->Ki;
		dx_temp = pid_control->Kd;
		debug_x_ref[0] = *ref;
		debug_x_set[0] = *set;
		channel_enable[0] = 1;
	}
	else if(channel == 2)
	{
		py_temp = pid_control->Kp;
		iy_temp = pid_control->Ki;
		dy_temp = pid_control->Kd;
		debug_x_ref[1] = *ref;
		debug_x_set[1] = *set;
		channel_enable[1] = 1;		
	}
//	if(debug_pid_state_flag == 0xC0)
//	{
//		pid_control->Kp = p_temp;
//		pid_control->Ki = i_temp;
//		pid_control->Kd = d_temp;
//		debug_pid_state_flag = 0xA0;
//	}		
}

/**
  * @brief ������λ����ֵ
  * @param cmd������
  * @param ch: ����ͨ��
  * @param data������ָ��
  * @param num����������
  * @retval ��
  */
void set_computer_value(uint8_t cmd, uint8_t ch, void *data, uint8_t num)
{
  uint8_t sum = 0;    // У���
  num *= 4;           // һ������ 4 ���ֽ�
  
  static packet_head_t set_packet;
  
  set_packet.head = FRAME_HEADER;     // ��ͷ 0x59485A53
  set_packet.len  = 0x0B + num;      // ����
  set_packet.ch   = ch;              // ����ͨ��
  set_packet.cmd  = cmd;             // ��������
  
  sum = check_sum(0, (uint8_t *)&set_packet, sizeof(set_packet));       // �����ͷУ���
  sum = check_sum(sum, (uint8_t *)data, num);                           // �������У���
  
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&set_packet, sizeof(set_packet),0xffff);    // ��������ͷ
  HAL_UART_Transmit(&UartHandle, (uint8_t *)data, num,0xffff);                          // ���Ͳ���
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&sum, sizeof(sum),0xffff);                  // ����У���
}

/**********************************************************************************************/

/**
   * @brief �����жϻص�����
   * @param ����
   * @retval None.
   */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &UartHandle)
	{
		switch(debug_pid_rx_data_flag)
		{
			case 0:
			if(debug_pid_rx_data == 0x53)
			{
				debug_pid_count = 0;
				debug_pid_rx_data_flag = 1;
			}
			else
			{
				debug_pid_count = 0;
				debug_pid_rx_data_flag = 0;				
			}break;
			case 1:
			if(debug_pid_rx_data == 0x5A)
			{
				debug_pid_count = 0;
				debug_pid_rx_data_flag = 2;
			}
			else
			{
				debug_pid_count = 0;
				debug_pid_rx_data_flag = 0;
			}break;
			case 2:
			if(debug_pid_rx_data == 0x48)
			{
				debug_pid_count = 0;
				debug_pid_rx_data_flag = 3;
			}
			else
			{
				debug_pid_count = 0;
				debug_pid_rx_data_flag = 0;
			}break;	
			case 3:			
			if(debug_pid_rx_data == 0x59)
			{
				debug_pid_count = 0;
				debug_pid_rx_data_flag = 4;
			}
			else
			{
				debug_pid_count = 0;
				debug_pid_rx_data_flag = 0;
			}break;			
			case 4:
			{
				debug_rx_buff[debug_pid_count] = debug_pid_rx_data;
				debug_pid_count++;
				if(debug_rx_buff[1] == 0x17)
				{
					if(debug_pid_count == 19)
					{
						debug_pid_count = 0;
						debug_pid_rx_data_flag = 0;
//					debug_pid_data_dispose();
				}
				}
				else if(debug_rx_buff[1] == 0x0B)
				{
					if(debug_pid_count == 7)
					{
						debug_pid_count = 0;
						debug_pid_rx_data_flag = 0;
						debug_normal_data_dispose();
					}
				}
			}
			if(debug_pid_count > 19)
			{
				debug_pid_count = 0;
				debug_pid_rx_data_flag = 0;
				uint8_t i;
				for(i=0;i<PROT_FRAME_LEN_RECV;i++)
				{
					debug_rx_buff[i] = 0;
				}
			}
		}

		HAL_UART_Receive_IT(&UartHandle,&debug_pid_rx_data,1);
	}
}

/**
   * @brief ���ݽ���(��ָ���PIDʱ)
   * @param None.
   * @retval None.
   */
static void debug_normal_data_dispose()
{
		if(debug_rx_buff[5] == START_CMD)
		{
			 set_computer_value(SEND_START_CMD, CURVES_CH1, NULL, 0);               // ͬ����λ����������ť״̬
			 set_computer_value(SEND_START_CMD, CURVES_CH2, NULL, 0);               // ͬ����λ����������ť״̬
			 if(debug_pid_state_flag > 0)
			 {
				 debug_pid_state_flag = 0xA0;
			 }
		}
		else if(debug_rx_buff[5] == STOP_CMD)
		{
			 set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0); 
       set_computer_value(SEND_STOP_CMD, CURVES_CH2, NULL, 0); 
			 if(debug_pid_state_flag > 0)
			 {
				 debug_pid_state_flag = 0xB0;
			 }
		}
		uint8_t i;
		for(i=0;i<PROT_FRAME_LEN_RECV;i++)
		{
			debug_rx_buff[i] = 0;
		}
}

/**
   * @brief ���ݽ���(��ָ����PIDʱ)
   * @param None.
   * @retval None.
   */
static void debug_pid_data_dispose()
{
	if(debug_rx_buff[5] == SET_P_I_D_CMD)
	{
			uint32_t temp0 = COMPOUND_32BIT(&debug_rx_buff[6]);
			uint32_t temp1 = COMPOUND_32BIT(&debug_rx_buff[10]);
			uint32_t temp2 = COMPOUND_32BIT(&debug_rx_buff[14]);
						
			p_temp = *(float *)&temp0;
			i_temp = *(float *)&temp1;
			d_temp = *(float *)&temp2;
		
			debug_pid_state_flag = 0xC0;
	}
	uint8_t i;
	for(i=0;i<PROT_FRAME_LEN_RECV;i++)
	{
		debug_rx_buff[i] = 0;
	}
}
