/*
 * isl29035_i2cdrv.h
 *
 *  Created on: 31 mai 2015
 *      Author: thierry
 */

#ifndef __ISL29035_I2CDRV_H_
#define __ISL29035_I2CDRV_H_

#include <stdbool.h>
#include "em_device.h"

// Sensor Address
#define ISL29035_ADDR			0x88

#define ISL29035_COMMAND_1_REG 		0x00
#define ISL29035_COMMAND_2_REG 		0x01
#define ISL29035_DATA_LSB_REG 		0x02
#define ISL29035_DATA_MSB_REG 		0x03
#define ISL29035_INT_LT_LSB_REG		0x04
#define ISL29035_INT_LT_MSB_REG 	0x05
#define ISL29035_INT_HT_LSB_REG		0x06
#define ISL29035_INT_HT_MSB_REG		0x07
#define ISL29035_ID_REG		 		0x0F

/** Command 1 */

// Command 1 operation mode bits 7,6,5
#define ISL29035_CMD1_SLEEP			0b00000000
#define ISL29035_CMD1_ASL_ONCE		0b00100000
#define ISL29035_CMD1_IR_ONCE		0b01000000
#define ISL29035_CMD1_ASL_CONT		0b10100000
#define ISL29035_CMD1_IR_CONT		0b11000000

// Interrupt persits bits 1,0
#define ISL29035_CMD1_INT1			0b00000000
#define ISL29035_CMD1_INT4			0b00000001
#define ISL29035_CMD1_INT8			0b00000010
#define ISL29035_CMD1_INT16			0b00000011

// Interrupt status
#define ISL29035_CMD1_INT_STATUS	0b00000100

/** Command 2 */

// Scale Command 2 bits 1,0
#define ISL29035_CMD2_SCALE_1000	0b00000000
#define ISL29035_CMD2_SCALE_4000	0b00000001
#define ISL29035_CMD2_SCALE_16000	0b00000010
#define ISL29035_CMD2_SCALE_64000	0b00000011

// ADC resolution Command 2 bits 3,2
#define ISL29035_CMD2_ADC_16		0b00000000
#define ISL29035_CMD2_ADC_12		0b00000100
#define ISL29035_CMD2_ADC_8			0b00001000
#define ISL29035_CMD2_ADC_4			0b00001100

#define ISL29035_DEFAULD_ID			0b00101000


bool Isl29035_Detect(uint8_t *deviceId);
bool Isl29035_Start(uint8_t mode);
bool Isl29035_SetRes(uint8_t res);
bool Isl29035_SetScale(uint8_t scale);
bool Isl29035_ReadRegister(uint8_t address, uint8_t *regValue);
bool Isl29035_WriteRegister(uint8_t address, uint8_t regValue);
bool Isl29035_ReadValue(uint16_t *value);

#endif /* __ISL29035_I2CDRV_H_ */
