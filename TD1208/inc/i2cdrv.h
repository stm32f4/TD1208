/*
 * i2cdrv.h
 *
 *  Created on: 30 mai 2015
 *      Author: thierry
 */


#ifndef __I2CDRV_H_
#define __I2CDRV_H_

#include "em_i2c.h"
#include "em_cmu.h"

void I2CDRV_Init(void);
I2C_TransferReturn_TypeDef I2CDRV_Transfer(I2C_TransferSeq_TypeDef *seq);

#endif /* __I2CDRV_H_ */
