/***************************************************************************//**
 * @file
 * @brief Timer peripheral API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 2.0.2
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012-2014 Telecom Design S.A., http://www.telecomdesign.fr</b>
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

#ifndef __TD_TIMER_H
#define __TD_TIMER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup TIMER
	 * @brief Timer peripheral API for the TDxxxx RF modules
	 * @{
	 *
	 * @note
	 *   Actually, the TIMER is used to implement a 50% duty-cycle output with
	 *   adjustable frequency only.
	 ******************************************************************************/

	/*******************************************************************************
	 *************************   PROTOTYPES   **************************************
	 ******************************************************************************/

	/** @addtogroup TIMER_USER_FUNCTIONS User Functions
	 * @{ */

	void TD_TIMER_Start(uint32_t frequency);
	void TD_TIMER_Stop(void);

	/** @} */

	/*******************************************************************************
	 **************************   PUBLIC VARIABLES   *******************************
	 ******************************************************************************/

	/** @addtogroup TIMER_GLOBAL_VARIABLES Global Variables
	 * @{ */
	/** @addtogroup TIMER_EXTERN External Declarations
	 * @{ */

	extern bool TD_TIMER_Enabled;

	/** @} */
	/** @} */

	/** @} (end addtogroup TIMER) */

#ifdef __cplusplus
}
#endif

#endif // __TD_TIMER_H
