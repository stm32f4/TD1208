/***************************************************************************//**
 * @file
 * @brief CMU (Clock Management Unit) peripheral API for the TDxxxx RF modules.
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

#include <efm32.h>
#include <em_cmu.h>

#include "td_core.h"
#include "td_cmu.h"

/***************************************************************************//**
 * @addtogroup CMU
 * @brief Clock management unit (CMU) Peripheral API for the TDxxxx RF modules
 * @{
 ******************************************************************************/

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup CMU_GLOBAL_FUNCTIONS Global Functions
 * @{ */

/***************************************************************************//**
 * @brief Initialize the CMU device
 * @param[in] external Use external crystal if true
 * otherwise use internal RC if false
 ******************************************************************************/
void TD_CMU_Init(bool external)
{
	if (external) {

		// Start LFXO, and use LFXO for low-energy modules
		CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
		CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
	} else {
		CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);
	}

	// Enable the clock to the interface of the low energy modules
	CMU_ClockEnable(cmuClock_CORELE, true);
}

/** @} */

/** @} (end addtogroup CMU) */
