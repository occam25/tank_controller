#include <stdint.h>
#include "main.h"
#include "iBus.h"

static uint8_t data;
static uint8_t iBus_buffer[IBUS_BUFFER_SIZE];
static uint8_t iBus_index;

uint8_t iBus_read_finished_f;
uint16_t iBus_data[IBUS_NUMBER_OF_CHANNELS];

void iBusReadInit(UART_HandleTypeDef *huart)
{
	iBus_index = 0;
	HAL_UART_Receive_IT(huart, &data, 1);
}

void iBusComputeValues(void)
{
	uint8_t index = 0;

	for(int i = 2; i < IBUS_BUFFER_SIZE - 2; i += 2){
		iBus_data[index] = iBus_buffer[i+1] << 8 | iBus_buffer[i];
		index++;
		if(index > IBUS_NUMBER_OF_CHANNELS - 1)
			return;
	}
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  huart UART handle.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(iBus_index == 0 && data != 0x20){
		goto wrong;
	}

	if(iBus_index == 1 && data != 0x40){
		iBus_index = 0;
		goto wrong;
	}

	iBus_buffer[iBus_index] = data;
	iBus_index++;

	if(iBus_index == IBUS_BUFFER_SIZE){
		iBus_read_finished_f = 1;
		return;
	}

wrong:
	HAL_UART_Receive_IT(huart, &data, 1);
}


/**
  * @brief  UART error callback.
  * @param  huart UART handle.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{

	uint32_t g_uart_sr = huart->Instance->ISR;

	uint32_t g_uart_dr = huart->Instance->RDR;

	HAL_UART_Receive_IT(huart, &data, 1);

}
