
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h> //+++
#include "main.h" //+++
#include "P2927_MC.h"

P2927_MC_STATUS P2927_MC_Init(I2C_HandleTypeDef *hi2c)
{
	if(PCA9685_OK != PCA9685_Init(hi2c))
		return P2927_MC_ERROR;

	return P2927_MC_OK;
}

P2927_MC_STATUS P2927_MC_setMotor(P2927_MC_MOTOR motor, P2927_MC_DIRECTION direction, uint16_t speed)
{
	uint8_t pin_pwm, pin_in1, pin_in2;
	uint16_t in1_value = 4095;

	if(speed == 0){
		in1_value = 0;
	}

	if(speed > 4095){
		speed = 4095;
	}

	switch(motor){
	case P2927_MC_MOTOR1:
		pin_pwm = P2927_MC_PWM_MOTOR1;
		pin_in1 = P2927_MC_IN1_MOTOR1;
		pin_in2 = P2927_MC_IN2_MOTOR1;
		break;
	case P2927_MC_MOTOR2:
		pin_pwm = P2927_MC_PWM_MOTOR2;
		pin_in1 = P2927_MC_IN1_MOTOR2;
		pin_in2 = P2927_MC_IN2_MOTOR2;
		break;
	case P2927_MC_MOTOR3:
		pin_pwm = P2927_MC_PWM_MOTOR3;
		pin_in1 = P2927_MC_IN1_MOTOR3;
		pin_in2 = P2927_MC_IN2_MOTOR3;
		break;
	case P2927_MC_MOTOR4:
		pin_pwm = P2927_MC_PWM_MOTOR4;
		pin_in1 = P2927_MC_IN1_MOTOR4;
		pin_in2 = P2927_MC_IN2_MOTOR4;
		break;
	default:
		return P2927_MC_ERROR;
	}

	if(direction == P2927_MC_BACKWARD){
		// Transpose pins
		uint8_t tmp = pin_in1;
		pin_in1 = pin_in2;
		pin_in2 = tmp;
	}

	if(PCA9685_OK != PCA9685_SetPin(pin_pwm, speed, 0))
		return P2927_MC_ERROR;

	if(PCA9685_OK != PCA9685_SetPin(pin_in1, in1_value, 0))
		return P2927_MC_ERROR;

	if(PCA9685_OK != PCA9685_SetPin(pin_in2, 0, 0))
		return P2927_MC_ERROR;

	return P2927_MC_OK;
}

int16_t rc_value_to_joystick(uint16_t rc_value)
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

P2927_MC_STATUS P2927_MC_setMotors_RC_values(uint16_t rc_x, uint16_t rc_y)
{
	P2927_MC_STATUS return_value = P2927_MC_OK;
	P2927_MC_MOTOR left_motor = P2927_MC_MOTOR3;
	P2927_MC_MOTOR right_motor = P2927_MC_MOTOR4;
	P2927_MC_DIRECTION left_motor_dir = P2927_MC_FORWARD;
	P2927_MC_DIRECTION right_motor_dir = P2927_MC_FORWARD;

	int16_t left_motor_value;
	int16_t right_motor_value;
	uint32_t left_motor_p2927;		// converted from -1000 <-> 1000 to 0 <-> 4095
	uint32_t right_motor_p2927;		// converted from -1000 <-> 1000 to 0 <-> 4095
	int16_t x;
	int16_t y;

	x = rc_value_to_joystick(rc_x);
	y = rc_value_to_joystick(rc_y);

	left_motor_value = y + x;
	right_motor_value = y - x;

	if(left_motor_value < 0)
		left_motor_dir = P2927_MC_BACKWARD;

	if(right_motor_value < 0)
		right_motor_dir = P2927_MC_BACKWARD;

	// Map -1000 <-> 1000 to 0 <-> 4095
	// 1. ABS value 0 <-> 1000 to 0 <-> 4095
	left_motor_p2927 = abs(left_motor_value);
	right_motor_p2927 = abs(right_motor_value);
	// 2. Map 0 - 1000 to 0 - 8190 -> multiply by 8.190
	left_motor_p2927 *= 8190;
	right_motor_p2927 *= 8190;
	// Round
	if(left_motor_p2927 % 1000 >= 500)
		left_motor_p2927 += 1000;
	if(right_motor_p2927 % 1000 >= 500)
		right_motor_p2927 += 1000;

	left_motor_p2927 /= 1000;
	right_motor_p2927 /= 1000;


	char line[64];
	snprintf(line, 64, "X: %d Y:%d -> L: %d R: %d -> L: %d[%c] R: %d[%c]\r\n",x, y, left_motor_value, right_motor_value,
			(uint16_t)left_motor_p2927, (left_motor_dir) ? 'B':'F', (uint16_t)right_motor_p2927, (right_motor_dir) ? 'B':'F');
	debugPrint(&huart2, line);

	// Set motors
	return_value += P2927_MC_setMotor(left_motor, left_motor_dir, left_motor_p2927);
	return_value += P2927_MC_setMotor(right_motor, right_motor_dir, right_motor_p2927);

	return return_value;

//	uint16_t speed;
//	P2927_MC_DIRECTION dir = P2927_MC_FORWARD;
//	uint32_t abs_value; // 0 - 500
//
//	// Dead zone
//	if(rc_value <= 1520 && rc_value >= 1480){
//		return P2927_MC_stopMotor(motor);
//	}
//
//	if(rc_value < 1500){
//		dir = P2927_MC_BACKWARD;
//		abs_value = 1500 - rc_value;
//	}else{
//		abs_value = rc_value - 1500;
//	}
//
//	if(abs_value > 500)
//		abs_value = 500;
//
//	// Map 0-500 to 0-4096 -> multiply by 8.19
//	abs_value *= 819;
//
//	if((abs_value % 100) >= 50)
//		abs_value += 100;
//
//	speed = abs_value / 100;
//
//	char line[40];
//	snprintf(line, 40, "%d -> %d -> %d\r\n",rc_value, abs_value, speed);
//	debugPrint(&huart2, line);
//
//	return P2927_MC_setMotor(motor, dir, speed);

}


P2927_MC_STATUS P2927_MC_stopMotor(P2927_MC_MOTOR motor)
{
	return P2927_MC_setMotor(motor, P2927_MC_FORWARD, 0);
}
