
#ifndef __IBUS_H
#define __IBUS_H

#define IBUS_BUFFER_SIZE				32 // 2 + 14*2 + 2
#define IBUS_NUMBER_OF_CHANNELS			10

extern uint8_t iBus_read_finished_f;
extern uint16_t iBus_data[IBUS_NUMBER_OF_CHANNELS];

void iBusReadInit(UART_HandleTypeDef *huart);
void iBusComputeValues(void);

#endif // __IBUS_H
