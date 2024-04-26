
#ifndef __BSP_DEBUG_PID_H__
#define __BSP_DEBUG_PID_H__

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/
#include "usart.h"
#include "pid.h"
#include "chassis_task.h"
#include "freertos.h"
#include "task.h"

#ifdef _cplusplus
extern "C" {
#endif   

//���ﶨ��DEBUG PIDʱ�Ĵ���
#define UartHandle huart1

//set��ref�ķŴ���(ͨ��1)
#define MAGNIFY_REF_SET   1

//set��ref�ķŴ���(ͨ��2)
#define MAGNIFY_REF_SET_X 2000	
	
//����PID�ķ�������(ÿ�뷢�ĸ���)
#define PID_CONTROL_TIME 200

//PID_CONTROL_TIME�ĵ���
#define PID_WAIT_TIME 5
	
/* ���ݽ��ջ�������С */
#define PROT_FRAME_LEN_RECV  20

/* �������ݵĳ��� */
#define PROT_FRAME_LEN_CHECKSUM    1

/* ����ͷ�ṹ�� */
typedef __packed struct
{
  uint32_t head;    // ��ͷ
  uint8_t ch;       // ͨ��
  uint32_t len;     // ������
  uint8_t cmd;      // ����
//  uint8_t sum;      // У���
  
}packet_head_t;

#define FRAME_HEADER     0x59485A53    // ֡ͷ

/* ͨ���궨�� */
#define CURVES_CH1      0x01
#define CURVES_CH2      0x02
#define CURVES_CH3      0x03
#define CURVES_CH4      0x04
#define CURVES_CH5      0x05

/* ָ��(��λ�� -> ��λ��) */
#define SEND_TARGET_CMD      0x01     // ������λ��ͨ����Ŀ��ֵ
#define SEND_FACT_CMD        0x02     // ����ͨ��ʵ��ֵ
#define SEND_P_I_D_CMD       0x03     // ���� PID ֵ
#define SEND_START_CMD       0x04     // ��������ָ��
#define SEND_STOP_CMD        0x05     // ����ָֹͣ��
#define SEND_PERIOD_CMD      0x06     // ��������

/* ָ��(��λ�� -> ��λ��) */
#define SET_P_I_D_CMD        0x10     // ����PIDֵ
#define SET_TARGET_CMD       0x11     // ����Ŀ��ֵ
#define START_CMD            0x12     // ����ָ��
#define STOP_CMD             0x13     // ָֹͣ��
#define RESET_CMD            0x14     // ��λָ��
#define SET_PERIOD_CMD       0x15     // ����ָ��

/* ��ָ�� */
#define CMD_NONE             0xFF     // ��ָ��

/* ����ֵ�궨�� */
#define HEAD_INDEX_VAL       0x3u     // ��ͷ����ֵ(4byte)
#define CHX_INDEX_VAL        0x4u     // ͨ������ֵ(1byte)
#define LEN_INDEX_VAL        0x5u     // ��������ֵ(4byte)
#define CMD_INDEX_VAL        0x9u     // ��������ֵ(1byte)

#define EXCHANGE_H_L_BIT(data)      ((((data) << 24) & 0xFF000000) |\
                                     (((data) <<  8) & 0x00FF0000) |\
                                     (((data) >>  8) & 0x0000FF00) |\
                                     (((data) >> 24) & 0x000000FF))     // �任�ߵ��ֽ�

#define COMPOUND_32BIT(data)        (((*(data-0) << 24) & 0xFF000000) |\
                                     ((*(data-1) << 16) & 0x00FF0000) |\
                                     ((*(data-2) <<  8) & 0x0000FF00) |\
                                     ((*(data-3) <<  0) & 0x000000FF))      // �ϳ�Ϊһ����
                                     
/**
 * @brief   �������ݴ���
 * @param   *data:  Ҫ��������ݵ�����
 * @param   data_len: ���ݵĴ�С
 * @return  void.
 */
void protocol_data_recv(uint8_t *data, uint16_t data_len);

/**
 * @brief   ���յ����ݴ���
 * @param   PID����,�趨ֵ������ֵ��ͨ��
 * @return  -1��û���ҵ�һ����ȷ������.
 */
void debug_pid_dispose(pid_type_def *pid_control,fp32 *ref,fp32 *set,uint8_t channel);

/**
  * @brief ������λ����ֵ
  * @param cmd:����
  * @param ch: ����ͨ��
  * @param data:����ָ��
  * @param num:��������
  * @retval ��
  */
void set_computer_value(uint8_t cmd, uint8_t ch, void *data, uint8_t num);

extern void debug_pid(void const *pvParameters);

#ifdef _cplusplus
}
#endif   

#endif
