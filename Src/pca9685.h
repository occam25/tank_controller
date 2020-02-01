

#ifndef PCA9685_H_
#define PCA9685_H_

#include "stm32l4xx_hal.h"

#define PCA9685_ADDRESS (0x60<<1)

//
//	Registers
//
#define PCA9685_REG_MODE1 	0x00
#define PCA9685_REG_MODE2 	0x01
#define PCA9685_REG_SUBADR1 0x02
#define PCA9685_REG_SUBADR2 0x03
#define PCA9685_REG_SUBADR3 0x04

#define PCA9685_REG_LED0_ON_L 	0x06
#define PCA9685_REG_LED0_ON_H	0x07
#define PCA9685_REG_LED0_OFF_L	0x08
#define PCA9685_REG_LED0_OFF_H 	0x09

//#define PCA9685_REG_LED1_ON_L 	0x0A
//#define PCA9685_REG_LED1_ON_H	0x0B
//#define PCA9685_REG_LED1_OFF_L	0x0C
//#define PCA9685_REG_LED1_OFF_H 	0x0D
//
//#define PCA9685_REG_LED2_ON_L 	0x0E
//#define PCA9685_REG_LED2_ON_H	0x0F
//#define PCA9685_REG_LED2_OFF_L	0x10
//#define PCA9685_REG_LED2_OFF_H 	0x11
//
//#define PCA9685_REG_LED3_ON_L 	0x12
//#define PCA9685_REG_LED3_ON_H	0x13
//#define PCA9685_REG_LED3_OFF_L	0x14
//#define PCA9685_REG_LED3_OFF_H 	0x15
//
//#define PCA9685_REG_LED4_ON_L 	0x16
//#define PCA9685_REG_LED4_ON_H	0x17
//#define PCA9685_REG_LED4_OFF_L	0x18
//#define PCA9685_REG_LED4_OFF_H 	0x19

#define PCA9685_REG_ALLLED_ON_L 	0xFA
#define PCA9685_REG_ALLLED_ON_H 	0xFB
#define PCA9685_REG_ALLLED_OFF_L 	0xFC
#define PCA9685_REG_ALLLED_OFF_H 	0xFD
#define PCA9685_REG_PRESCALE 		0xFE


typedef enum
{
	PCA9685_MODE1_SUB1_BIT 	= 3,
	PCA9685_MODE1_SUB2_BIT	= 2,
	PCA9685_MODE1_SUB3_BIT	= 1
}SubaddressBit;

#define PCA9685_MODE1_ALCALL_BIT	0
#define PCA9685_MODE1_SLEEP_BIT		4
#define PCA9685_MODE1_AI_BIT		5
#define PCA9685_MODE1_EXTCLK_BIT	6
#define PCA9685_MODE1_RESTART_BIT	7


typedef enum
{
	PCA9685_OK 		= 0,
	PCA9685_ERROR	= 1
}PCA9685_STATUS;

PCA9685_STATUS PCA9685_SoftwareReset(void);
PCA9685_STATUS PCA9685_SleepMode(uint8_t Enable);
PCA9685_STATUS PCA9685_RestartMode(uint8_t Enable);
PCA9685_STATUS PCA9685_AutoIncrement(uint8_t Enable);

#ifndef PCA9685_SERVO_MODE
PCA9685_STATUS PCA9685_SetPwmFrequency(uint16_t Frequency);
#endif

PCA9685_STATUS PCA9685_SetPwm(uint8_t Channel, uint16_t OnTime, uint16_t OffTime);
PCA9685_STATUS PCA9685_SetPin(uint8_t Channel, uint16_t Value, uint8_t Invert);


PCA9685_STATUS PCA9685_Init(I2C_HandleTypeDef *hi2c);


#endif /* PCA9685_H_ */