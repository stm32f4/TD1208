/***************************************************************************//**
 * @file
 * @brief SIGFOX V1 API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 3.0.0
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
#ifdef SIGFOX_V1

#ifndef __TD_SIGFOX_V1_H
#define __TD_SIGFOX_V1_H

#include <stdint.h>
#include <stdbool.h>

#ifdef SIGFOX_DOWNLINK
#include "td_sigfox_downlink.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup SIGFOX SIGFOX
	 * @brief SIGFOX API for the TDxxxx RF modules
	 * @{
	 ******************************************************************************/

	/*******************************************************************************
	 ***********************   ENUMERATIONS   **************************************
	 ******************************************************************************/

	/** @addtogroup SIGFOX_ENUMERATIONS Enumerations
	 * @{ */

	/** SIGFOX transmit modes */
	typedef enum {
		MODE_BIT = 0,				/**< Single bit mode */
		MODE_FRAME,					/**< Byte frame mode */
		MODE_OOB,					/**< Out Of Band mode */
	}
	TD_SIGFOX_mode_t;

	/** @} */

	/*******************************************************************************
	 *************************   PROTOTYPES   **************************************
	 ******************************************************************************/

	/** @addtogroup SIGFOX_USER_FUNCTIONS User Functions
	 * @{ */

	bool TD_SIGFOX_Init_(bool init);
	bool TD_SIGFOX_Send_(uint8_t *message, uint8_t size, uint8_t retry);
	bool TD_SIGFOX_SendV1(uint8_t mode, bool value, uint8_t *message, uint8_t size, uint8_t retry, bool ack, bool reserved);
	bool TD_SIGFOX_SendTest(uint16_t count, uint8_t time, uint16_t slot, uint8_t retry);
	bool TD_SIGFOX_RfPower(uint8_t power);
	uint8_t TD_SIGFOX_PowerVoltage(void);
	uint32_t TD_SIGFOX_PowerVoltageExtended(void);
	void TD_SIGFOX_KeepAlive(void);
	char const *TD_SIGFOX_VersionLib(void);

	/** @} */

	/** @addtogroup SIGFOX_GLOBAL_FUNCTIONS Global Functions
	 * @{ */

	bool TD_SIGFOX_SendOOB(uint8_t *message, uint8_t size, uint8_t retry);
	bool TD_SIGFOX_PinConf(uint8_t config);
	bool TD_SIGFOX_SetKeepAliveRetries(uint8_t count);
	bool TD_SIGFOX_FrequencyConf(uint32_t frequency, uint16_t allowed, uint16_t forbidden, uint16_t hop);
	void TD_SIGFOX_SendTestPA(uint16_t slot, uint8_t mode, uint32_t count);

	/** @} */

	/** @} (end addtogroup SIGFOX) */

#ifdef __cplusplus
}
#endif

#endif // __TD_SIGFOX_V1_H
#endif
