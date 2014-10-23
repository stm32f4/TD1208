/***************************************************************************//**
 * @file
 * @brief Driver definition for the UBlox 7 GPS used in TD12xx RF modules.
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
#ifndef __UBX7_H
#define __UBX7_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup TD_UBX7 Ublox7
	 * @brief UBlox 7 GPS driver.
	 * @{
	 ******************************************************************************/

	/*******************************************************************************
	 **************************  TYPEDEFS   ****************************************
	 ******************************************************************************/

	/** @addtogroup TD_UBX7_TYPEDEFS Typedefs
	 * @{ */

	/** UBlox 7 GPS start modes */
	typedef enum {
		TD_UBX7_HOT = 0,
		TD_UBX7_WARM,
		TD_UBX7_COLD
	}
	TD_UBX7_StartMode_t;

	/** @} */

	/*******************************************************************************
	 *************************   PROTOTYPES   **************************************
	 ******************************************************************************/

	/** @addtogroup TD_UBX7_GLOBAL_FUNCTIONS Global Functions
	 * @{ */

	void TD_UBX7_Init(void);
	void TD_UBX7_Reset(void);
	void TD_UBX7_WakeUp(void);
	void TD_UBX7_SoftBackup(void);
	bool TD_UBX7_SendeFuse(void);
	void TD_UBX7_HardwareBackup(void);
	bool TD_UBX7_PowerUp(bool power_save);
	void TD_UBX7_PowerOff(void);
	void TD_UBX7_PowerOn(void);
	uint16_t TD_UBX7_Poll(uint8_t class, uint8_t id, uint8_t *reply, uint16_t reply_len);
	void TD_UBX7_PollNavClock(void);
	bool TD_UBX7_PollRxmRTC5(void);
	void TD_UBX7_PollNavSvInfo(void);
	bool TD_UBX7_Process(uint8_t *data, uint8_t max_read);
	void TD_UBX7_PollMonExcept(void);
	bool TD_UBX7_SendEphemeris(uint8_t *buf, uint16_t sz);
	void TD_UBX7_PollMonTxBuf(void);
	uint16_t TD_UBX7_PollEphemeris(uint8_t *t, uint16_t sz);
	bool TD_UBX7_SetRate(uint16_t mes_rate, uint8_t nav_rate);
	bool TD_UBX7_isFused(void);
	void TD_UBX7_Dump(void);
	void TD_UBX7_SendStart(TD_UBX7_StartMode_t type);
	/** @} */

	/** @} */
	/** @} (end addtogroup TD_UBX7) */

#ifdef __cplusplus
}
#endif

#endif // __UBX7_H
