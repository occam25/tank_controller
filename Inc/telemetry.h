
#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include <stdint.h>

#define TELEMETRY_MSG_LEN			128


#define TELEMETRY_SEND_SPEED		0x01
#define TELEMETRY_SEND_TEMPERATURE	0x02
#define TELEMETRY_SEND_PRESSURE		0x04
#define TELEMETRY_SEND_HUMIDITY		0x08
#define TELEMETRY_SEND_ACC			0x10
#define TELEMETRY_SEND_GYRO			0x20

typedef uint8_t t_data_mask;

typedef enum
{
	TELEMETRY_OK = 0,
	TELEMETRY_ERROR = 1
}TELEMETRY_STATUS;

extern float speed;

void telemetry_init(UART_HandleTypeDef *huart, t_data_mask mask);
TELEMETRY_STATUS telemetry_send_data(void);

#endif /* TELEMETRY_H_ */
