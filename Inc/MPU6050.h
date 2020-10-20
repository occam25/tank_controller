/*
 * MPU6050.h
 *
 *  Created on: 17 feb. 2020
 *      Author: USUARIO
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_


#define MPU6050_ADDR				0xD0
#define MPU6050_PWR_MGMT_1_REG		0x6B
#define MPU6050_SMPLRT_DIV_REG		0x19
#define MPU6050_ACCEL_CONFIG_REG	0x1C
#define MPU6050_GYRO_CONFIG_REG		0x1B
#define MPU5060_ACCEL_XOUT_H_REG	0x3B
#define MPU5060_GYRO_XOUT_H_REG		0x43
#define MPU6050_WHO_AM_I_REG		0x75


typedef enum
{
	MPU6050_OK = 0,
	MPU6050_ERROR = 1
}MPU6050_STATUS;


extern float Ax, Ay, Az, Gx, Gy, Gz;


MPU6050_STATUS MPU6050_Init(I2C_HandleTypeDef *i2c_handler);
void MPU5060_Read_Accel(void);
void MPU6050_Read_Gyro(void);

#endif /* INC_MPU6050_H_ */
