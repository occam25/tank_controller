/*
 * servos.h
 *
 *  Created on: 1 feb. 2020
 *      Author: USUARIO
 */

#ifndef SERVOS_H_
#define SERVOS_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

/************************
 * 	PWM period: 20ms  -> 50Hz
 * 	Timer clock freq:  80MHz
 * 	timer preescaler: 1600 - 1
 * 	counter period:    1000
 */

#define	PWM_TIM_CCR1_MAX			125		/* T_ON = 2.5ms */
#define PWM_TIM_CCR1_MIN			 25 	/* T_OFF = 0.5ms */

/* Calibration values: Max and min values for servos */
#define TILT_IBUS_MAX				1825
#define TILT_IBUS_MIN				1200
#define PAN_IBUS_MAX				1900
#define PAN_IBUS_MIN				1080

typedef enum
{
	SERVOS_OK 		= 0,
	SERVOS_ERROR	= 1
}SERVOS_STATUS;

void SERVOS_Init(TIM_HandleTypeDef *htim);
void SERVOS_update_pantilt(uint16_t pan, uint16_t tilt);

#endif /* SERVOS_H_ */
