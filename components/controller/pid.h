/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
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
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;


// PID结构体
typedef struct PID
{
	float kp;			   // kp
	float ki;			   // ki
	float kd;			   // kd
	double pout;		   // k输出
	float iout;			   // i输出
	float dout;			   // d输出
	float now_error;	   // 当前误差
	float Last_error;	   // 上一次误差
	float Last_Last_error; // 上上次误差
	float sum_of_error;	   // 历史总误差
	float set;			   // 设置
	float now;			   // 当前
	float out;			   // 输出

	int pid_mode; // PID模式设置，1为位置环PID，2为增量式PID

	float MaxOutput;	 // PID输出限幅
	float IntegralLimit; // I输出限幅
	float plus;			 // 本次增量值
	float plus_out;		 // 增量式输出值plus_out = last_plus_out + plus
	float last_plus_out; // 上次增量式输出值

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


// PID限制最大值函数
void PID_limit(float *a, float PID_MAX);

// PID限制最小值函数
void PID_limitmin(float *a, float PID_MIN);

// PID限幅函数
float xianfu(float a, float max);

// PID绝对值函数
void PID_juedui(float *a);

float PID_Fabs(float ffabs);

// PID初始化
void pid_init(PID *pid, float p, float i, float d, int maxout, int imaxout, int mode);

// PID全部初始化
void pid_math_init(PID *pid, float p, float i, float d, int maxout, int imaxout, int mode, float pid_Aa, float pid_Bb, float pid_Alpha, float Limit_Data);

// pid参数更新
void pid_change(PID *pid, float p, float i, float d, int maxout, int imaxout, int mode);

// PID函数
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
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
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
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

#endif
