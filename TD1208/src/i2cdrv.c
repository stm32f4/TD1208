#include "i2cdrv.h"
#include "em_gpio.h"

/*******************************************************************************
 * @brief
 *   Initalize basic I2C master mode driver for use on the DVK.extern
 *
 * @details
 *   This driver only supports master mode, single bus-master. In addition
 *   to configuring the EFM32 I2C peripheral module, it also configures DVK
 *   specific setup in order to use the I2C bus.
 *
 * @param[in] init
 *   Pointer to I2C initialization structure.
 ******************************************************************************/
void I2CDRV_Init(void) {

	const I2C_Init_TypeDef init = I2C_INIT_DEFAULT;

	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_I2C0, true);

	GPIO_PinModeSet(gpioPortA, 0, gpioModeWiredAndPullUp, 1); //sda
	GPIO_PinModeSet(gpioPortA, 1, gpioModeWiredAndPullUp, 1); //scl

	/* Enable pins at location 0 */
	I2C0->ROUTE = I2C_ROUTE_SDAPEN |
	I2C_ROUTE_SCLPEN | (I2C_ROUTE_LOCATION_LOC0);

	I2C_Init(I2C0, &init);
}

/*******************************************************************************
 * @brief
 *   Perform I2C transfer.
 *
 * @details
 *   This driver only supports master mode, single bus-master. It does not
 *   return until the transfer is complete, polling for completion.
 *
 * @param[in] seq
 *   Pointer to sequence structure defining the I2C transfer to take place. The
 *   referenced structure must exist until the transfer has fully completed.
 ******************************************************************************/
I2C_TransferReturn_TypeDef I2CDRV_Transfer(I2C_TransferSeq_TypeDef *seq) {
	uint32_t timeout = 100000;
	I2C_TransferReturn_TypeDef ret;

	/* Do a polled transfer */
	ret = I2C_TransferInit(I2C0, seq);
	if (ret == i2cTransferUsageFault) {
		return i2cTransferUsageFault;
	}
	while (ret == i2cTransferInProgress && timeout--) {
		ret = I2C_Transfer(I2C0);
	}

	return (ret);
}
