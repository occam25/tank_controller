/*
 * rc_controller.c
 *
 *  Created on: Apr 16, 2023
 *      Author: USUARIO
 */



#include <stdint.h>
#include <stdlib.h>
#include <stdio.h> //+++
#include "main.h" //+++

#include "rc_controller.h"

//#define DEBUG_MOTOR_VALUES


static int16_t rc_value_to_joystick(uint16_t rc_value)
{
	int16_t joystick_value;

	joystick_value = rc_value - 1500;

	// Dead zone
	if(abs(joystick_value) <= 20){
		return 0;
	}

	if(joystick_value > 500){
		joystick_value = 500;
	}else if(joystick_value < -500){
		joystick_value = -500;
	}

	return joystick_value;
}

void rc_controller_setMotors_RC_values(uint16_t rc_x, uint16_t rc_y)
{
	direction_t left_motor_dir = FORWARD;
	direction_t right_motor_dir = FORWARD;

	int16_t left_motor_value;
	int16_t right_motor_value;
	uint32_t left_motor_duty_cicle;		// converted from 0 <-> 1000 to 0 <-> 100
	uint32_t right_motor_duty_cicle;	// converted from 0 <-> 1000 to 0 <-> 100
	int16_t x;
	int16_t y;

	x = rc_value_to_joystick(rc_x);
	y = rc_value_to_joystick(rc_y);

	left_motor_value = y + x;
	right_motor_value = y - x;

	if(left_motor_value < 0)
		left_motor_dir = BACKWARD;

	if(right_motor_value < 0)
		right_motor_dir = BACKWARD;

	left_motor_duty_cicle = (abs(left_motor_value) * 100)/1000;
	right_motor_duty_cicle = (abs(right_motor_value) * 100)/1000;

#ifdef DEBUG_MOTOR_VALUES
	char line[80];
	snprintf(line, sizeof(line), "X: %5d  Y: %5d  ->  L: %5d  R: %5d  ->  L: %5d[%c]  R: %5d[%c]\r\n",x, y, left_motor_value, right_motor_value,
			(uint16_t)left_motor_duty_cicle, (left_motor_dir) ? 'B':'F', (uint16_t)right_motor_duty_cicle, (right_motor_dir) ? 'B':'F');
	debugPrint(&huart2, line);
#endif

	// Set motors
	L289N_set_motor_speed(LEFT_MOTOR, left_motor_dir, left_motor_duty_cicle);
	L289N_set_motor_speed(RIGHT_MOTOR, right_motor_dir, right_motor_duty_cicle);

}


void rc_controller_stopMotor(motor_t motor)
{
	L289N_set_motor_speed(motor, 0, 0);
}

void rc_controller_init(TIM_HandleTypeDef *htim)
{
	L289N_Init(htim);
}
