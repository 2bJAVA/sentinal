/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
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
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "pid.h"
#include "main.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

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
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

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
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

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
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}



//PID�������ֵ����
void PID_limit(float *a, float PID_MAX)
{
    if(*a > PID_MAX)
        *a = PID_MAX;
    if(*a < -PID_MAX)
        *a = -PID_MAX;
}

//PID������Сֵ����
void PID_limitmin(float *a, float PID_MIN)
{
		if(*a < PID_MIN && *a > 0)
        *a = PID_MIN;
    if(*a > -PID_MIN && *a < 0)
        *a = -PID_MIN;
}

//PID�޷�����
float xianfu(float a,float max)
{
	if(a > 50000)
			a=a-max;
	if(a < 50000)
			a=max-a;
 return a;
}

//PID����ֵ����
void PID_juedui(float *a)
{
		if(*a > 0)
        *a = *a;
    if(*a < 0)
        *a = -*a;
}

float PID_Fabs(float ffabs)
{
		if(ffabs >= 0)
        return ffabs;
    if(ffabs <= 0)
        return -ffabs;
		return ffabs;
}

//PID��ʼ��
void pid_init(PID*pid,float p,float i,float d,int maxout,int imaxout,int mode)
{
	pid->kp=p;
	pid->ki=i;
	pid->kd =d;
	pid->pout =0;
	pid->iout =0;
	pid->dout =0;
	pid->Last_error =0;
	pid->Last_Last_error =0;
	pid->now_error =0;
	pid->sum_of_error =0;
	pid->pid_mode =mode;//1Ϊλ�û�PID��2Ϊ����ʽPID
	pid->MaxOutput =maxout;//PID�޷�
	pid->IntegralLimit=imaxout;	
	pid->plus =0;
	pid->plus_out =0;
	pid->last_plus_out =0;
}

//PID��ʼ��
void pid_math_init(PID*pid,float p,float i,float d,int maxout,int imaxout,int mode,float pid_Aa,float pid_Bb,float pid_Alpha,float Limit_Data)
{
	pid->kp=p;
	pid->ki=i;
	pid->kd =d;
	pid->pout =0;
	pid->iout =0;
	pid->dout =0;
	pid->Last_error =0;
	pid->Last_Last_error =0;
	pid->now_error =0;
	pid->sum_of_error =0;
	pid->pid_mode =mode;//1Ϊλ�û�PID��2Ϊ����ʽPID
	pid->MaxOutput =maxout;//PID�޷�
	pid->IntegralLimit=imaxout;	
	pid->plus =0;
	pid->plus_out =0;
	pid->last_plus_out =0;
	
	pid->Small_Error_Limit = Limit_Data;
	
	
	pid->Set_A = pid_Aa;
	pid->Set_B = pid_Bb;
	pid->Set_alpha = pid_Alpha;
	
}

//pid��������
void pid_change(PID*pid,float p,float i,float d,int maxout,int imaxout,int mode)
{
	pid->kp=p;
	pid->ki=i;
	pid->kd =d;
	pid->pid_mode =mode;//1Ϊλ�û�PID��2Ϊ����ʽPID
	pid->MaxOutput =maxout;//PID�޷�
	pid->IntegralLimit=imaxout;	
}

//PID����
float pid_calc(PID*pid, float now, float set)
	{
    pid->now = now;
    pid->set = set;

		pid->now_error = pid->set - pid->now;	//set - measure

    if(pid->pid_mode == 1) //λ�û�PID
    {
	      pid->pout = pid->kp * pid->now_error;
        pid->iout = pid->ki * pid->sum_of_error;
        pid->dout = pid->kd * (pid->now_error - pid->Last_error );
				pid->sum_of_error+=pid->now_error;	
				PID_limit(&(pid->sum_of_error), 10000);
				PID_limit(&(pid->iout), pid->IntegralLimit);
        pid->out = pid->pout + pid->iout + pid->dout;
        PID_limit(&(pid->out), pid->MaxOutput);
    }	
		
    else if(pid->pid_mode == 2)//����ʽPID
    {
        pid->pout = pid->kp * (pid->now_error - pid->Last_error);
        pid->iout = pid->ki * pid->now_error;
        pid->dout = pid->kd * (pid->now_error - 2*pid->Last_error + pid->Last_Last_error);        
				PID_limit(&(pid->iout), pid->IntegralLimit);
        pid->plus = pid->pout + pid->iout + pid->dout;
        pid->plus_out = pid->last_plus_out + pid->plus;
			  pid->out = pid->plus_out; 
				PID_limit(&(pid->out), pid->MaxOutput);
        pid->last_plus_out = pid->plus_out;	//update last time
    }
		
    pid->Last_Last_error= pid->Last_error;
    pid->Last_error = pid->now_error;

    return pid->out;
}

void ffc_init(PID_FFC * FFC_init,float Ka,float Kb)
{
	FFC_init->Ka = Ka;
	FFC_init->Kb = Kb;
	FFC_init->perrRin = 0.0f;
	FFC_init->lastRin = 0.0f;
}

/*ʵ��ǰ��������*/
float FeedforwardController(PID_FFC *vFFC,float Rin)
{
  float result;
 
  vFFC->rin = Rin;
	
  result=vFFC->Ka*(vFFC->rin-vFFC->lastRin)+vFFC->Kb*(vFFC->rin-2*vFFC->lastRin+vFFC->perrRin);
 
  vFFC->perrRin= vFFC->lastRin;
  vFFC->lastRin= vFFC->rin;
	
  vFFC->out = result;
	
  return result;
}
