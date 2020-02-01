/*
 * servos.c
 *
 *  Created on: 1 feb. 2020
 *      Author: USUARIO
 */


#include "servos.h"

#include <stdio.h> //+++
#include "main.h" //+++

static TIM_HandleTypeDef *htim_pwm;


static uint16_t compute_CCR1_value(uint16_t iBUS_value)
{
	return (2*PWM_TIM_CCR1_MIN - PWM_TIM_CCR1_MAX + ((PWM_TIM_CCR1_MAX-PWM_TIM_CCR1_MIN)*iBUS_value)/1000);
}

void SERVOS_update_pantilt(uint16_t iBUS_pan, uint16_t iBUS_tilt)
{
	uint16_t pan;
	uint16_t tilt;

	if(iBUS_pan > PAN_IBUS_MAX){
		iBUS_pan = PAN_IBUS_MAX;
	}else if(iBUS_pan < PAN_IBUS_MIN){
		iBUS_pan = PAN_IBUS_MIN;
	}

	if(iBUS_tilt > TILT_IBUS_MAX){
		iBUS_tilt = TILT_IBUS_MAX;
	}else if(iBUS_tilt < TILT_IBUS_MIN){
		iBUS_tilt = TILT_IBUS_MIN;
	}

	pan = compute_CCR1_value(iBUS_pan);
	tilt = compute_CCR1_value(iBUS_tilt);

	htim_pwm->Instance->CCR1 = pan;
	htim_pwm->Instance->CCR2 = tilt;

	char line[64];
	snprintf(line, 64, "PAN: %d TILT:%d\r\n",pan, tilt);
	debugPrint(&huart2, line);

}

void SERVOS_Init(TIM_HandleTypeDef *htim)
{
	htim_pwm = htim;
	HAL_TIM_PWM_Start(htim_pwm, TIM_CHANNEL_1);	// init PWM
	HAL_TIM_PWM_Start(htim_pwm, TIM_CHANNEL_2);	// init PWM
}
