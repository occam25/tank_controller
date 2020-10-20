
#include <stdlib.h>
#include <stdio.h>

#include "stm32l4xx_hal.h"
#include "MPU6050.h"
#include "telemetry.h"

static t_data_mask data_mask;
static UART_HandleTypeDef *huart_tel = NULL;
static char telemetry_msg[TELEMETRY_MSG_LEN];

float speed;

void telemetry_init(UART_HandleTypeDef *huart, t_data_mask mask)
{
	huart_tel = huart;
	data_mask = mask;
}

size_t build_msg(void)
{
	size_t cnt = 0;

	if(data_mask & TELEMETRY_SEND_SPEED){
		cnt += snprintf(telemetry_msg + cnt, sizeof(telemetry_msg) - cnt - 1, "<SPD>%.2f\n",speed);
	}
//	if(data_mask & TELEMETRY_SEND_TEMPERATURE){
//
//	}
//	if(data_mask & TELEMETRY_SEND_PRESSURE){
//
//	}
//	if(data_mask & TELEMETRY_SEND_HUMIDITY){
//
//	}
	if(data_mask & TELEMETRY_SEND_ACC){
		cnt += snprintf(telemetry_msg + cnt, sizeof(telemetry_msg) - cnt - 1, "<ACC>%.2f,%.2f,%.2f\n",Ax, Ay, Az);
	}
	if(data_mask & TELEMETRY_SEND_GYRO){
		cnt += snprintf(telemetry_msg + cnt, sizeof(telemetry_msg) - cnt - 1, "<GYR>%.2f,%.2f,%.2f\n",Gx, Gy, Gz);
	}

	return cnt;
}

TELEMETRY_STATUS telemetry_send_data(void)
{
	size_t bytes_to_send = 0;

	if(huart_tel == NULL)
		return TELEMETRY_ERROR;

	bytes_to_send = build_msg();

	if(bytes_to_send)
		HAL_UART_Transmit(huart_tel, (uint8_t *) telemetry_msg, bytes_to_send, 10);

	return TELEMETRY_OK;
}
