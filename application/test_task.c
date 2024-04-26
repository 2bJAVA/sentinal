/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       test_task.c/h
  * @brief      buzzer warning task.��������������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "test_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_buzzer.h"
#include "detect_task.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot.h"
#include "path_task.h"
#include <stdio.h>
#include "visual_task.h"
static void buzzer_warn_error(uint8_t num);

const error_t *error_list_test_local;
extern path_control_t path_control;



/**
  * @brief          test task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          test����
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void test_task(void const * argument)
{
    static uint8_t error, last_error;
    static uint8_t error_num;
    error_list_test_local = get_error_list_point();

    while(1)
    {
        error = 0;
//	printf("%f,%f \n",chassis_move.motor_chassis->speed,chassis_move.motor_chassis->speed_set);
		printf("%f,%f \n",gimbal_control.gimbal_yaw_motor.absolute_angle,gimbal_control.Gimbal_Yaw_Control_Data);
		//printf("%f,%f \n",gimbal_control.gimbal_pitch_motor.absolute_angle,gimbal_control.Gimbal_Pitch_Control_Data);
		//printf("%f,%f \n",gimbal_control.gimbal_pitch_motor.motor_gyro,gimbal_control.Gimbal_Pitch_Apid_Out);
//      printf("%f,%f \n",gimbal_control.gimbal_yaw_motor.motor_gyro,gimbal_control.Gimbal_Yaw_Apid_Out);
		//printf("%f,%f,%f \n",chassis_move.chassis_yaw_motor->relative_angle,chassis_move.chassis_INS_angle[0],chassis_move.chassis_yaw);
        //find error
        //���ִ���
        for(error_num = 0; error_num < REFEREE_TOE; error_num++)
        {
            if(error_list_test_local[error_num].error_exist)
            {
                error = 1;
                break;
            }
        }

        //no error, stop buzzer
        //û�д���, ֹͣ������
        if(error == 0 && last_error != 0)
        {
            buzzer_off();
        }
        //have error
        //�д���
        if(error)
        {
            buzzer_warn_error(error_num+1);
        }

        last_error = error;
        osDelay(10);
    }
}


/**
  * @brief          make the buzzer sound
  * @param[in]      num: the number of beeps 
  * @retval         none
  */
/**
  * @brief          ʹ�÷�������
  * @param[in]      num:��������
  * @retval         none
  */
static void buzzer_warn_error(uint8_t num)
{
    static uint8_t show_num = 0;
    static uint8_t stop_num = 100;
    if(show_num == 0 && stop_num == 0)
    {
        show_num = num;
        stop_num = 100;
    }
    else if(show_num == 0)
    {
        stop_num--;
        buzzer_off();
    }
    else
    {
        static uint8_t tick = 0;
        tick++;
        if(tick < 50)
        {
            buzzer_off();
        }
        else if(tick < 100)
        {
            buzzer_on(1, 30000);
        }
        else
        {
            tick = 0;
            show_num--;
        }
    }
}


