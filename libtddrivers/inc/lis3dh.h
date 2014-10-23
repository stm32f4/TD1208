/***************************************************************************//**
 * @file
 * @brief Driver definition for the LIS3DH accelerometer used in TD12xx RF modules.
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2013-2014 Telecom Design S.A., http://www.telecomdesign.fr</b>
 ******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Telecom Design SA has no
 * obligation to support this Software. Telecom Design SA is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Telecom Design SA will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/
#ifndef __LIS3DH_H
#define __LIS3DH_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

	/**************************************************************************//**
	 * @addtogroup TD_LIS3DH LIS3DH
	 * @brief LIS3DH accelerometer driver.
	 * @{
	 ******************************************************************************/

	/******************************************************************************
	 **************************  DEFINES   ****************************************
	 ******************************************************************************/

	/** @addtogroup TD_LIS3DH_DEFINES Defines
	 * @{ */

// LIS3DH accelerometer IRQ sources
#define LIS3DH_IRQ_OVERRUN		(1 << 1)	///< LIS3DH FIFO Overrun interrupt on INT1
#define LIS3DH_IRQ_WTM			(1 << 2)	///< LIS3DH FIFO Watermark interrupt on INT1
#define LIS3DH_IRQ_DRDY2		(1 << 3)	///< LIS3DH DRDY2 interrupt on INT1
#define LIS3DH_IRQ_DRDY1		(1 << 4)	///< LIS3DH DRDY1 interrupt on INT1
#define LIS3DH_IRQ_AOI2			(1 << 5)	///< LIS3DH AOI2 interrupt on INT1
#define LIS3DH_IRQ_AOI1			(1 << 6)	///< LIS3DH AOI1 interrupt on INT1
#define LIS3DH_IRQ_CLICK		(1 << 7)	///< LIS3DH CLICK interrupt on INT11

// LIS3DH accelerometer IRQ events
#define LIS3DH_INT1_CFG_XLIE	(1 << 0)	///< LIS3DH Enable interrupt generation on X low event or on Direction recognition
#define LIS3DH_INT1_CFG_XHIE	(1 << 1)	///< LIS3DH Enable interrupt generation on X high event or on Direction recognition
#define LIS3DH_INT1_CFG_YLIE	(1 << 2)	///< LIS3DH Enable interrupt generation on Y low event or on Direction recognition
#define LIS3DH_INT1_CFG_YHIE	(1 << 3)	///< LIS3DH Enable interrupt generation on Y high event or on Direction recognition
#define LIS3DH_INT1_CFG_ZLIE	(1 << 4)	///< LIS3DH Enable interrupt generation on Z low event or on Direction recognition
#define LIS3DH_INT1_CFG_ZHIE	(1 << 5)	///< LIS3DH Enable interrupt generation on Z high event or on Direction recognition
#define LIS3DH_INT1_CFG_6D		(1 << 6)	///< LIS3DH 6 direction detection function enabled
#define LIS3DH_INT1_CFG_AOI		(1 << 7)	///< LIS3DH And/Or combination of Interrupt events

	/** LIS3DH accelerometer IRQ events for all 3 axis */
#define LIS3DH_3D \
	LIS3DH_INT1_CFG_XLIE | \
	LIS3DH_INT1_CFG_XHIE | \
	LIS3DH_INT1_CFG_YLIE | \
	LIS3DH_INT1_CFG_YHIE | \
	LIS3DH_INT1_CFG_ZLIE | \
	LIS3DH_INT1_CFG_ZHIE

	/** @} */

	/*******************************************************************************
	 *************************   PROTOTYPES   **************************************
	 ******************************************************************************/

	/** @addtogroup TD_LIS3DH_GLOBAL_FUNCTIONS Global Functions
	 * @{ */

	void TD_LIS3DH_ClearIRQ(void);
	void TD_LIS3DH_SetFilterRef(void);
	void TD_LIS3DH_ConfigHighPass(bool enable, uint8_t cutoff);
	void TD_LIS3DH_Configure(uint8_t mode, uint8_t rate, uint8_t axis, uint8_t scale);
	void TD_LIS3DH_SetFIFOMode(uint8_t mode, uint8_t watermak);
	uint8_t TD_LIS3DH_GetFIFOStatus();
	void TD_LIS3DH_GetXYZ(uint16_t *x, uint16_t *y, uint16_t *z);
	uint8_t TD_LIS3DH_GetEventSource(void);
	void TD_LIS3DH_DebugReg(void);
	void TD_LIS3DH_ConfigureIRQ(uint8_t source, uint8_t event, uint8_t threshold, uint8_t duration);
	uint8_t TD_LIS3DH_Status(void);
	uint8_t TD_LIS3DH_Who(void);
	bool TD_LIS3DH_Init(void);
	void TD_LIS3DH_Dump(void);
	uint8_t TD_LIS3DH_ReadRegister(uint8_t address);
	void TD_LIS3DH_WriteRegister(uint8_t address, uint8_t value);

	/** @} */

	/** @addtogroup TD_LIS3DH_USER_FUNCTIONS User Functions
	 * @{ */

	/** @} */

	/** @} (end addtogroup TD_LIS3DH) */

#ifdef __cplusplus
}
#endif

#endif // __LIS3DH_H

