#ifndef _RC_CONTROLLER_H_
#define _RC_CONTROLLER_H_

#include "L298N_motor_controller.h"

void rc_controller_init(TIM_HandleTypeDef *htim);
void rc_controller_setMotors_RC_values(uint16_t rc_x, uint16_t rc_y);
void rc_controller_stopMotor(motor_t motor);

#endif
