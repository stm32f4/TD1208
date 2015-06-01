/*
 * isl29035_i2cdrv.c
 *
 *  Created on: 31 mai 2015
 *      Author: thierry
 */

#include <stddef.h>
#include <isl29035_i2cdrv.h>
#include <i2cdrv.h>

bool Isl29035_Detect(uint8_t *deviceId) {
	if (Isl29035_ReadRegister(ISL29035_ID_REG, deviceId)) {
		if ((*deviceId & ISL29035_DEFAULD_ID) == ISL29035_DEFAULD_ID) {
			return true;
		} else {
			return false;
		}
	} else {
		return false;
	}
}

bool Isl29035_ReadValue(uint16_t *value) {
	uint8_t LSBValue;
	uint8_t MSBValue;
	uint8_t scale;
	uint16_t maxCount = 0;
	bool success;

	success = Isl29035_ReadRegister(ISL29035_COMMAND_2_REG, &scale);
	if (!success)
		return false;

	success = Isl29035_ReadRegister(ISL29035_DATA_LSB_REG, &LSBValue);
	if (!success)
		return false;

	success = Isl29035_ReadRegister(ISL29035_DATA_MSB_REG, &MSBValue);
	if (!success)
		return false;

	*value = (MSBValue << 8) + LSBValue;

	maxCount = (scale & 0b00001100) >> 2;
	maxCount = 0xFFFF >> (maxCount * 4);

	scale = scale & 0b00000011;
	switch (scale) {
	case ISL29035_CMD2_SCALE_1000:
		*value = (1000L * (long) *value) / maxCount;
		break;
	case ISL29035_CMD2_SCALE_4000:
		*value = (4000L * (long) *value) / maxCount;
		break;
	case ISL29035_CMD2_SCALE_16000:
		*value = (16000L * (long) *value) / maxCount;
		break;
	case ISL29035_CMD2_SCALE_64000:
		*value = (64000L * (long) *value) / maxCount;
		break;
	}

	return true;
}

bool Isl29035_Start(uint8_t mode) {
	uint8_t regValue;
	if (Isl29035_ReadRegister(ISL29035_COMMAND_1_REG, &regValue)) {
		regValue = (regValue & 0b00011111) | mode;
		return Isl29035_WriteRegister(ISL29035_COMMAND_1_REG, regValue);
	} else {
		return false;
	}
}

bool Isl29035_SetRes(uint8_t res) {
	uint8_t regValue;
	if (Isl29035_ReadRegister(ISL29035_COMMAND_2_REG, &regValue)) {
		regValue = (regValue & 0b11110011) | res;
		return Isl29035_WriteRegister(ISL29035_COMMAND_2_REG, regValue);
	} else {
		return false;
	}
}

bool Isl29035_SetScale(uint8_t scale) {
	uint8_t regValue;
	if (Isl29035_ReadRegister(ISL29035_COMMAND_2_REG, &regValue)) {
		regValue = (regValue & 0b11111100) | scale;
		return Isl29035_WriteRegister(ISL29035_COMMAND_2_REG, regValue);
	} else {
		return false;
	}
}

bool Isl29035_ReadRegister(uint8_t address, uint8_t *regValue) {
	I2C_TransferSeq_TypeDef seq;
	I2C_TransferReturn_TypeDef ret;
	uint8_t i2c_read_data[1];
	uint8_t i2c_write_data[1];

	seq.addr = ISL29035_ADDR;
	seq.flags = I2C_FLAG_WRITE_READ;
	/* Select command to issue */
	i2c_write_data[0] = address;
	seq.buf[0].data = i2c_write_data;
	seq.buf[0].len = 1;
	/* Select location/length of data to be read */
	seq.buf[1].data = i2c_read_data;
	seq.buf[1].len = 1;

	ret = I2CDRV_Transfer(&seq);
	if (ret == i2cTransferDone) {
		*regValue = i2c_read_data[0];
		return true;
	} else {
		return false;
	}
}

bool Isl29035_WriteRegister(uint8_t regAddr, uint8_t regValue) {
	I2C_TransferSeq_TypeDef seq;
	I2C_TransferReturn_TypeDef ret;
	uint8_t i2c_write_data[2];

	seq.addr = ISL29035_ADDR;
	seq.flags = I2C_FLAG_WRITE;
	/* Select command to issue */
	i2c_write_data[0] = regAddr;
	i2c_write_data[1] = regValue;
	seq.buf[0].data = i2c_write_data;
	seq.buf[0].len = 2;

	ret = I2CDRV_Transfer(&seq);
	if (ret != i2cTransferDone) {
		return false;
	} else {
		return true;
	}
}
