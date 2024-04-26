/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pidʵ�ֺ�����������ʼ����PID���㺯����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H
#include "struct_typedef.h"
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID ������
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //������
    fp32 max_iout; //���������

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    fp32 error[3]; //����� 0���� 1��һ�� 2���ϴ�

} pid_type_def;


// PID�ṹ��
typedef struct PID
{
	float kp;			   // kp
	float ki;			   // ki
	float kd;			   // kd
	double pout;		   // k���
	float iout;			   // i���
	float dout;			   // d���
	float now_error;	   // ��ǰ���
	float Last_error;	   // ��һ�����
	float Last_Last_error; // ���ϴ����
	float sum_of_error;	   // ��ʷ�����
	float set;			   // ����
	float now;			   // ��ǰ
	float out;			   // ���

	int pid_mode; // PIDģʽ���ã�1Ϊλ�û�PID��2Ϊ����ʽPID

	float MaxOutput;	 // PID����޷�
	float IntegralLimit; // I����޷�
	float plus;			 // ��������ֵ
	float plus_out;		 // ����ʽ���ֵplus_out = last_plus_out + plus
	float last_plus_out; // �ϴ�����ʽ���ֵ

	float Max_Error_Data;

	float Small_Error_Limit;

	int Set_Out_Mode;
	float Set_A;
	float Set_B;
	float Set_ratio;
	float Set_alpha;
	float Last_Ud;
} PID;

typedef struct{
  float Ka;
  float Kb;
  float rin;
  float lastRin;
  float perrRin;
  float out;
}PID_FFC;


// PID�������ֵ����
void PID_limit(float *a, float PID_MAX);

// PID������Сֵ����
void PID_limitmin(float *a, float PID_MIN);

// PID�޷�����
float xianfu(float a, float max);

// PID����ֵ����
void PID_juedui(float *a);

float PID_Fabs(float ffabs);

// PID��ʼ��
void pid_init(PID *pid, float p, float i, float d, int maxout, int imaxout, int mode);

// PIDȫ����ʼ��
void pid_math_init(PID *pid, float p, float i, float d, int maxout, int imaxout, int mode, float pid_Aa, float pid_Bb, float pid_Alpha, float Limit_Data);

// pid��������
void pid_change(PID *pid, float p, float i, float d, int maxout, int imaxout, int mode);

// PID����
float pid_calc(PID *pid, float now, float set);

void ffc_init(PID_FFC * FFC_init,float Ka,float Kb);

float FeedforwardController(PID_FFC *vFFC,float Rin);

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      mode: PID_POSITION:��ͨPID
  *                 PID_DELTA: ���PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid������
  * @param[in]      max_iout: pid���������
  * @retval         none
  */
extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid����
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      ref: ��������
  * @param[in]      set: �趨ֵ
  * @retval         pid���
  */
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid ������
  * @param[out]     pid: PID�ṹ����ָ��
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

#endif
