/***************************************************************************//**
 * @file
 * @brief Driver for the Si7020 Temperature / Humidity sensor
 * @version 3.20.5
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#ifndef __Si7020_H
#define __Si7020_H

#include "em_device.h"
#include <stdbool.h>

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

/** I2C device address for Si7020 */
#define Si7020_ADDR      0x80

/*******************************************************************************
 *****************************   PROTOTYPES   **********************************
 ******************************************************************************/

int32_t Si7020_MeasureRHAndTemp(uint32_t *rhData, int32_t *tData);

int32_t Si7020_GetFirmwareRevision(uint8_t *fwRev);

bool Si7020_Detect(uint8_t *deviceId);

#endif /* __Si7020_H */
