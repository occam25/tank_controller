/*
 * MPU6050.c
 *
 *  Created on: 17 feb. 2020
 *      Author: USUARIO
 */

#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "MPU6050.h"


static I2C_HandleTypeDef *hi2c;

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Ax, Ay, Az, Gx, Gy, Gz;



MPU6050_STATUS MPU6050_Init(I2C_HandleTypeDef *i2c_handler)
{
	uint8_t check, data;

	hi2c = i2c_handler;

	if(HAL_OK != HAL_I2C_IsDeviceReady(hi2c, MPU6050_ADDR, 3, 100))
		return MPU6050_ERROR;

//	HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, MPU6050_WHO_AM_I_REG, 1, &check, 1, 1000);

//	if(check == 104){ // Device is present
		// wake up sensor
		data = 0;
		HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_PWR_MGMT_1_REG, 1, &data, 1, 1000);

		//+++
		HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_WHO_AM_I_REG, 1, &check, 1, 1000);
		if(check == 104){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//			debugPrint(&huart2, "I2C Device ready\r\n");
		}
		//+++
		// set data rate to 1KHz
		data = 0x07;
		HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_SMPLRT_DIV_REG, 1, &data, 1, 1000);
		// set accelerometer configuration
		data = 0x00;
		HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_ACCEL_CONFIG_REG, 1, &data, 1, 1000);
		// set gyroscopic configuration
		data = 0x00;
		HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_GYRO_CONFIG_REG, 1, &data, 1, 1000);
//	}

		return MPU6050_OK;

}

void MPU5060_Read_Accel(void)
{
	uint8_t acc_data[6];

	HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU5060_ACCEL_XOUT_H_REG, 1, acc_data, 6, 1000);


	Accel_X_RAW = (int16_t)(acc_data[0] << 8 | acc_data [1]);
	Accel_Y_RAW = (int16_t)(acc_data[2] << 8 | acc_data [3]);
	Accel_Z_RAW = (int16_t)(acc_data[4] << 8 | acc_data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	Ax = Accel_X_RAW/16384.0;
	Ay = Accel_Y_RAW/16384.0;
	Az = Accel_Z_RAW/16384.0;

//	  char line[80];
////	  snprintf(line, 80, "%4d %4d %4d\r\n",Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW);
//	  snprintf(line, 80, "ACC: %.2f %.2f %.2f\r\n",Ax, Ay, Az);
//	  debugPrint(&huart2, line);
}
void MPU6050_Read_Gyro(void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (hi2c, MPU6050_ADDR, MPU5060_GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into dps (°/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

	Gx = Gyro_X_RAW/131.0;
	Gy = Gyro_Y_RAW/131.0;
	Gz = Gyro_Z_RAW/131.0;

//	  char line[80];
////	  snprintf(line, 80, "%4d %4d %4d\r\n",Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW);
//	  snprintf(line, 80, "GYR: %.2f %.2f %.2f\r\n",Gx, Gy, Gz);
//	  debugPrint(&huart2, line);
}

