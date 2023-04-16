/*
 * L298N_motor_controller.c
 *
 *  Created on: Apr 16, 2023
 *      Author: USUARIO
 */

#include <stdio.h>
#include "main.h" //+++
#include "L298N_motor_controller.h"

#define DEBUG_CCR_VALUES

static TIM_HandleTypeDef *htim_motors_pwm;

/*
 * SYSCLK = 80 MHz
 * APBx_PRES = 1
 * APBx_CLK = 80 MHz
 * TIM_CLK = 80 MHz
 * TIM_Prescaler = 1600 - 1 = 1599
 * ARR = 4999 -> TIM_Period = 100 ms
 * Duty_cicle = 0   -> CCRx = 0
 * Duty_cicle = 100 -> CCrx = 4999
 * */

void L289N_set_motor_speed(motor_t motor, direction_t direction, uint8_t duty_cicle)
{

	if(htim_motors_pwm == NULL)
		return;

	if(motor == RIGHT_MOTOR)
	{
		// set direction
		if(direction == FORWARD)
		{
			// Reseting OUT2 and setting OUT1
			HAL_GPIO_WritePin(RM_OUT2_GPIO_Port, RM_OUT2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RM_OUT1_GPIO_Port, RM_OUT1_Pin, GPIO_PIN_SET);
		}
		else
		{
			// Reseting OUT1 and setting OUT2
			HAL_GPIO_WritePin(RM_OUT1_GPIO_Port, RM_OUT1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RM_OUT2_GPIO_Port, RM_OUT2_Pin, GPIO_PIN_SET);
		}
		// set speed (PWM duty cicle 0-100%)
		htim_motors_pwm->Instance->CCR1 = (duty_cicle * 4999)/100;
	}
	else if(motor == LEFT_MOTOR)
	{
		// set direction
		if(direction == FORWARD)
		{
			// Reseting OUT2 and setting OUT1
			HAL_GPIO_WritePin(LM_OUT2_GPIO_Port, LM_OUT2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LM_OUT1_GPIO_Port, LM_OUT1_Pin, GPIO_PIN_SET);
		}
		else
		{
			// Reseting OUT1 and setting OUT2
			HAL_GPIO_WritePin(LM_OUT1_GPIO_Port, LM_OUT1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LM_OUT2_GPIO_Port, LM_OUT2_Pin, GPIO_PIN_SET);
		}
		// set speed (PWM duty cicle 0-100%)
		htim_motors_pwm->Instance->CCR2 = (duty_cicle * 4999)/100;
	}
	else
		return;

#ifdef DEBUG_CCR_VALUES
	char line[80];
	snprintf(line, sizeof(line), "M: %c  Dir: %c  DC: %3d  CCRx: %5d\r\n",(motor == RIGHT_MOTOR)?'R':'L', (direction == FORWARD)?'F':'B', duty_cicle, (duty_cicle * 4999)/100);
	debugPrint(&huart2, line);
#endif

}

void L289N_Init(TIM_HandleTypeDef *htim)
{
	htim_motors_pwm = htim;
	HAL_TIM_PWM_Start(htim_motors_pwm, TIM_CHANNEL_1);	// init PWM
	HAL_TIM_PWM_Start(htim_motors_pwm, TIM_CHANNEL_2);	// init PWM
}
