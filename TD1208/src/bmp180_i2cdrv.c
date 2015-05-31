/*******************************************************************************
* @file bmp180_i2cdrv.c
* @brief Based on the I2C0 poll based driver for master mode operation on DVK.
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

#include <stddef.h>
#include "i2cdrv.h"
#include "bmp180_i2cdrv.h"
#include "td_rtc.h"

#define OSS 0           // no oversampling

/*******************************************************************************
*************************  Variables declation   ******************************
******************************************************************************/
int16_t AC1;
int16_t AC2;
int16_t AC3;
uint16_t AC4;
uint16_t AC5;
uint16_t AC6;
int16_t B1;
int16_t B2;
int16_t MB;
int16_t MC;
int16_t MD;

/*******************************************************************************
**************************   GLOBAL FUNCTIONS   *******************************
******************************************************************************/

/*******************************************************************************
* @brief
*   Write data to BMP180.
*
* @param[in] addr
*   Chip Address for writing
*
* @param[in] reg
*   Register to write
*
* @param[in] value
*   Value to write in register
*
* @return Return code
******************************************************************************/
int BMP180_WriteReg(uint8_t addr, uint8_t reg, uint8_t value){
    
    I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef ret;
    uint8_t data[3];
    
    seq.addr = addr;
    seq.flags = I2C_FLAG_WRITE;
    
    /* Select register to be written */
    data[0] = reg;
    seq.buf[0].data = data;
    
    /* Only 1 byte reg */
    data[1] = value;
    seq.buf[0].len = 2;
    
    ret = I2CDRV_Transfer(&seq);
    if (ret != i2cTransferDone)
    {
        return((int)ret);
    }
    
    return(ret);
    
}

/*******************************************************************************
* @brief
*   Read data from BMP180 (2 bytes).
*
* @param[in] addr
*   Chip Address for reading
*
* @param[in] reg
*   Register to read
*
* @return Register value
******************************************************************************/
uint16_t BMP180_ReadReg(uint8_t addr, uint8_t reg){
    
    I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef ret;
    uint8_t regid[1];
    uint8_t data[2];
    
    seq.addr = addr;
    seq.flags = I2C_FLAG_WRITE_READ;
    /* Select register to be read */
    regid[0] = reg;
    seq.buf[0].data = regid;
    seq.buf[0].len = 1;
    
    /* 2 bytes reg */
    data[0] = 0;
    data[1] = 0;
    seq.buf[1].data = data;
    seq.buf[1].len = 2;
    
    ret = I2CDRV_Transfer(&seq);
    if (ret != i2cTransferDone)
    {
        return(0);
    }
    
    return((data[0] << 8) | data[1]);
}

/******************************************************************************
* @brief
*   Read data from BMP180 (1 byte).
*
* @param[in] addr
*   Chip Address for reading
*
* @param[in] reg
*   Register to read
*
* @return Register value
******************************************************************************/
uint8_t BMP180_Read8bitReg(uint8_t addr, uint8_t reg){
    
    I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef ret;
    uint8_t regid[1];
    uint8_t data[2];
    
    seq.addr = addr;
    seq.flags = I2C_FLAG_WRITE_READ;
    /* Select register to be read */
    regid[0] = reg;
    seq.buf[0].data = regid;
    seq.buf[0].len = 1;
    
    /* 1 bytes reg */
    data[0] = 0;
    seq.buf[1].data = data;
    seq.buf[1].len = 1;
    
    ret = I2CDRV_Transfer(&seq);
    if (ret != i2cTransferDone)
    {
        return(0);
    }
    
    return (data[0]);
}

/******************************************************************************
* @brief
*   Read sensor calibration data.
*
******************************************************************************/
void BMP180_GetCalData(void){
    
    AC1 = BMP180_ReadReg(0xee, 0xaa);  
    AC2 = BMP180_ReadReg(0xee, 0xac);
    AC3 = BMP180_ReadReg(0xee, 0xae);
    AC4 = BMP180_ReadReg(0xee, 0xb0);
    AC5 = BMP180_ReadReg(0xee, 0xb2);
    AC6 = BMP180_ReadReg(0xee, 0xb4);
    B1 = BMP180_ReadReg(0xee, 0xb6);
    B2 = BMP180_ReadReg(0xee, 0xb8);
    MB = BMP180_ReadReg(0xee, 0xba);
    MC = BMP180_ReadReg(0xee, 0xbc);
    MD = BMP180_ReadReg(0xee, 0xbe);
}

/******************************************************************************
* @brief
*   Read raw temperature from BMP180
*
* @return Raw temperature
******************************************************************************/
int32_t BMP180_GetTempData(void){
    
    BMP180_WriteReg(0xee, 0xf4, 0x2e);
    
    TD_RTC_Delay(3276);
    
    return BMP180_ReadReg(0xee, 0xf6);
}

/******************************************************************************
* @brief
*   Read raw pressure from BMP180.
*
* @return Raw pressure
******************************************************************************/
int32_t BMP180_GetPressureData(void){
    
    uint16_t MSB;
    uint8_t XLSB;
    
    BMP180_WriteReg(0xee, 0xf4, 0x34 + (OSS << 6));
    
    TD_RTC_Delay(3276);
    
    MSB = BMP180_ReadReg(0xee, 0xf6);
    XLSB = BMP180_Read8bitReg(0xee, 0xf8);
    
    return ( ( MSB << 8 ) + XLSB ) >> ( 8 - OSS );    
}

/******************************************************************************
* @brief
*   Compute pressure and temperature from raw data
*
* @param[in] UT
*   Raw temperature value
*
* @param[in] UP
*   Raw pressure value
*
* @param[out] T_return
*   Calculated temperature
*
* @param[out] P_return
*   Calculated pressure
*
* @return Register value
******************************************************************************/
void BMP180_CalculateTempPressure(int32_t UT, int32_t UP, int32_t* T_return, int32_t* P_return){
    
    int32_t X1;
    int32_t X2;
    int32_t X3;
    int32_t B3;
    uint32_t B4;
    int32_t B5;
    int32_t B6;
    int32_t B7;
    int32_t T;
    int32_t P;
    
    X1 = (UT - AC6) * AC5 / (1 << 15);
    X2 = MC * (1 << 11) / (X1 + MD);
    B5 = X1 + X2;
    
    T = (B5 + 8)/(1 << 4);
    
    B6 = B5 - 4000;
    X1 = (B2 * (B6 * B6 / (1<<12) )) / (1<<11);
    X2 = AC2 * B6 / (1<<11);
    X3 = X1 + X2;
    B3 = (((AC1 * 4 + X3) << OSS) + 2) / 4;
    X1 = AC3 * B6 / (1<<13);
    X2 = (B1 * (B6 * B6 / (1<<12))) / (1<<16);
    X3 = ((X1 + X2) + 2) / (1<<2);
    B4 = AC4 * (uint32_t)(X3 + 32768) / (1<<15);
    B7 = ((uint32_t) UP - B3) * (50000 >> OSS);
    
    if(B7 < 0x80000000){
        P = (B7 * 2) / B4;
    }
    else{
        P = (B7 / B4) * 2;
    }
    
    X1 = (P / (1<<8)) * (P / (1<<8));
    X1 = (X1 * 3038) / (1<<16);
    X2 = (-7357 * P) / (1<<16);
    
    P = P + (X1 + X2 + 3791) / (1<<4); 
    
    *T_return = T;
    *P_return = P;
}
