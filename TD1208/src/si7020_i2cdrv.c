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

#include <stddef.h>
#include "Si7020_i2cdrv.h"
#include "i2cdrv.h"

#include "stddef.h"

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/


/** Si7020 Read Temperature Command */
#define Si7020_READ_TEMP        		0xE0 /* Read previous T data from RH measurement command*/
#define Si7020_READ_TEMP_ONLY   		0xE3 /* Read T data. Hold Master Mode*/
#define Si7020_READ_TEMP_ONLY_NOHOLD   	0xF3 /* Read T data. No Hold Master Mode*/
/** Si7020 Read RH Command */
#define Si7020_READ_RH          		0xE5 /* Perform RH (and T) measurement. Hold Master Mode */
#define Si7020_READ_RH_NOHOLD   		0xF5 /* Perform RH (and T) measurement. No Hold Master Mode */
/** Si7020 Read ID */
#define Si7020_READ_ID1_1       		0xFA
#define Si7020_READ_ID1_2       		0x0F
#define Si7020_READ_ID2_1       		0xFc
#define Si7020_READ_ID2_2       		0xc9
/** Si7020 Read Firmware Revision */
#define Si7020_READ_FWREV_1     		0x84
#define Si7020_READ_FWREV_2     		0xB8
/** Si7020 control */
#define Si7020_READ_USER_REG			0xE7
#define Si7020_WRITE_USER_REG			0xE6
#define Si7020_RESET     				0xFE


/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/


/**************************************************************************//**
 * @brief
  *  Reads data from the Si7020 sensor.
 * @param[in] i2c
 *   The I2C peripheral to use (not used).
 * @param[in] addr
 *   The I2C address of the sensor.
 * @param[out] data
 *   The data read from the sensor.
 * @param[in] command
 *   The command to send to device. See the \#define's for details.
 * @return
 *   Returns number of bytes read on success. Otherwise returns error codes
 *   based on the I2CDRV.
 *****************************************************************************/
static int32_t Si7020_Measure(uint32_t *data, uint8_t command)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[2];
  uint8_t                    i2c_write_data[1];

  seq.addr  = Si7020_ADDR;
  seq.flags = I2C_FLAG_WRITE_READ;
  /* Select command to issue */
  i2c_write_data[0] = command;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 1;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 2;

  ret = I2CDRV_Transfer(&seq);

  if (ret != i2cTransferDone)
  {
    *data = 0;
    return((int) ret);
  }

  *data = ((uint32_t) i2c_read_data[0] << 8) + (i2c_read_data[1] & 0xfc);

  return((int) 2);
}


/**************************************************************************//**
 * @brief
  *  Reads Firmware Revision from a Si7020 sensor.
 * @param[in] i2c
 *   The I2C peripheral to use.
 * @param[in] addr
 *   The I2C address of the sensor.
 * @param[out] fwRev
 *   The internal firmware revision. 0xFF === 1.0
 * @return
 *   Returns zero on OK, non-zero otherwise.
 *****************************************************************************/
int32_t Si7020_GetFirmwareRevision(uint8_t *fwRev)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_write_data[2];
  uint8_t                    i2c_read_data[1];

  seq.addr  = Si7020_ADDR;
  seq.flags = I2C_FLAG_WRITE_READ;
  /* Select command to issue */
  i2c_write_data[0] = Si7020_READ_FWREV_1;
  i2c_write_data[1] = Si7020_READ_FWREV_2;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 2;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 1;

  ret = I2CDRV_Transfer(&seq);

  if (ret != i2cTransferDone)
  {
    *fwRev = 0;
    return (uint32_t)ret;
  }
  *fwRev = i2c_read_data[0];

  return (uint32_t)i2cTransferDone;
}


/**************************************************************************//**
 * @brief
  *  Reads relative humidity and temperature from a Si7020 sensor.
 * @param[in] i2c
 *   The I2C peripheral to use.
 * @param[in] addr
 *   The I2C address of the sensor.
 * @param[out] rhData
 *   The relative humidity in percent (multiplied by 1000).
 * @param[out] tData
 *   The temperature in milli-Celsius.
 * @return
 *   Returns zero on OK, non-zero otherwise.
 *****************************************************************************/
int32_t Si7020_MeasureRHAndTemp(uint32_t *rhData, int32_t *tData)
{
  int ret = Si7020_Measure(rhData, Si7020_READ_RH);

  if (ret == 2)
  {
    /* convert to milli-percent */
    *rhData = (((*rhData) * 15625L) >> 13) - 6000;
  }
  else
  {
    return -1;
  }

  ret = Si7020_Measure((uint32_t *) tData, Si7020_READ_TEMP);

  if (ret == 2)
  {
    *tData = (((*tData) * 21965L) >> 13) - 46850; /* convert to milli-degC */
  }
  else
  {
    return -1;
  }

  return 0;
}


/**************************************************************************//**
 * @brief
 *   Checks if a Si7020 is present on the I2C bus or not.
 * @param[in] i2c
 *   The I2C peripheral to use (Not used).
 * @param[in] addr
 *   The I2C address to probe.
 * @param[out] deviceId
 *   Write device ID from SNB_3 if device reponds. Pass in NULL to discard.
 * @return
 *   True if a Si7020 is detected, false otherwise.
 *****************************************************************************/
bool Si7020_Detect(uint8_t *deviceId)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[8];
  uint8_t                    i2c_write_data[2];

  seq.addr  = Si7020_ADDR;
  seq.flags = I2C_FLAG_WRITE_READ;
  /* Select command to issue */
  i2c_write_data[0] = Si7020_READ_ID2_1;
  i2c_write_data[1] = Si7020_READ_ID2_2;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 2;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 8;

  ret = I2CDRV_Transfer(&seq);
  if (ret != i2cTransferDone)
  {
    return false;
  }
  if (NULL != deviceId)
  {
    *deviceId = i2c_read_data[0];
  }
  return true;
}
