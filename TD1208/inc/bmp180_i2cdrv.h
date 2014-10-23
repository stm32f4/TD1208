/***************************************************************************//**
 * @file bmp180_i2cdrv.h
 * @brief based on the I2C0 poll based driver for master mode operation on DVK.
 * @author Energy Micro AS
 * @version 1.00
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2010 Energy Micro AS, http://www.energymicro.com</b>
 *******************************************************************************
 *
 * This source code is the property of Energy Micro AS. The source and compiled
 * code may only be used on Energy Micro "EFM32" microcontrollers.
 *
 * This copyright notice may not be removed from the source code nor changed.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/

#ifndef __I2CDRV_H
#define __I2CDRV_H

#include "em_i2c.h"

#ifdef __cplusplus
extern "C" {
#endif
    
/*******************************************************************************
 *****************************   PROTOTYPES   **********************************
 ******************************************************************************/

void I2CDRV_Init(void);
I2C_TransferReturn_TypeDef I2CDRV_Transfer(I2C_TransferSeq_TypeDef *seq);
int BMP180_WriteReg(uint8_t addr, uint8_t reg, uint8_t value);
uint16_t BMP180_ReadReg(uint8_t addr, uint8_t reg);
uint8_t BMP180_Read8bitReg(uint8_t addr, uint8_t reg);
void BMP180_CalculateTempPressure(int32_t UT, int32_t UP, int32_t* T_return, int32_t* P_return);
void BMP180_GetCalData(void);
int32_t BMP180_GetPressureData(void);
int32_t BMP180_GetTempData(void);

#ifdef __cplusplus
}
#endif

#endif /* __I2CDRV_H */
