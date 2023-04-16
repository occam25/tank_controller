#ifndef _L298N_MOTOR_CONTROLLER_H_
#define _L298N_MOTOR_CONTROLLER_H_

typedef enum motor_e{
	RIGHT_MOTOR,
	LEFT_MOTOR,
} motor_t;

typedef enum direction_e{
	FORWARD,
	BACKWARD,
} direction_t;

void L289N_set_motor_speed(motor_t motor, direction_t direction, uint8_t duty_cicle);
void L289N_Init(TIM_HandleTypeDef *htim);

#endif
