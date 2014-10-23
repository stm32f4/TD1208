/***************************************************************************//**
 * @file
 * @brief Watchdog peripheral API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Telecom Design S.A., http://www.telecomdesign.fr</b>
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

#include <stdint.h>
#include <stdbool.h>

#include <em_rmu.h>
#include <em_wdog.h>

#include "td_core.h"
#include "td_rtc.h"
#include "td_printf.h"
#include "td_scheduler.h"
#include "td_watchdog.h"

/***************************************************************************//**
 * @addtogroup WATCHDOG
 * @brief Watchdog peripheral API for the TD RF modules
 * @{
 ******************************************************************************/

/*******************************************************************************
 ************************   PRIVATE VARIABLES   ********************************
 ******************************************************************************/

/** @addtogroup WATCHDOG_LOCAL_VARIABLES Local Variables
 * @{ */

/** Watchdog interval in seconds (8, 16, 32, 64, 128 or 256 only) */
static uint8_t TD_WATCHDOG_Interval = 0;

/** Watchdog scheduler timer */
static uint8_t TD_Watchdog_Timer = 0xFF;

/** Watchdog enable flag */
static bool TD_Watchdog_Enabled = false;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup WATCHDOG_LOCAL_FUNCTIONS Local Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Watchdog scheduler callback.
 *
 * @param[in] arg
 *   Generic argument registered with the scheduler.
 *
 * @param[in] repetition
 *   Timer repetition, 0xFF for infinite.
 ******************************************************************************/
static void TD_WATCHDOG_Callback(uint32_t arg, uint8_t repetition)
{
	TD_WATCHDOG_Feed();
}

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup WATCHDOG_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Initialize the watchdog.
 *
 * @param[in] period
 *   Period in seconds at which the watchdog will trigger (only 8, 16, 32, 64,
 *   128 and 256 seconds are valid).
 *
 * @return
 *   Returns true if given period is correct, false otherwise.
 ******************************************************************************/
bool TD_WATCHDOG_Init(uint16_t period)
{
	WDOG_Init_TypeDef WDOGInit;
	int power;

	for (power = 0; power <= 8; power++, period >>= 1) {
		if (period & 1) {
			break;
		}
	}
	period >>= 1;
	if (period || power < 3) {

		// Not a power of 2, > 2^8 or < 2^3
		return false;
	}

	// Refresh interval is half the trigger interval, to be on the
	// safe side
	TD_WATCHDOG_Interval = 1 << (power - 1);
	WDOGInit.enable = false;
	WDOGInit.debugRun = false;

	// Should count while EM2... bug, false does not work!
	WDOGInit.em2Run = true;

	// Should not count while EM3... bug, true does not work!
	WDOGInit.em3Run = false;

	// Allow EM4
	WDOGInit.em4Block = false;
	WDOGInit.swoscBlock = false;
	WDOGInit.lock = false;

	// 1 kHz clock for longest delay
	WDOGInit.clkSel = wdogClkSelULFRCO;
	WDOGInit.perSel = (WDOG_PeriodSel_TypeDef)(power + 7);
	WDOG_Init(&WDOGInit);
	return true;
}

/***************************************************************************//**
 * @brief
 *   Start/Stop the watchdog
 *
 * @param[in] enable
 *   Set to true to enable the watchdog, false to disable it.
 *
 * @param[in] automatic
 *   Set to true to feed the watchdog by a scheduler process, i.e. while in the
 *   main() idle loop.
 *
 * @return
 *   Returns true if the watchdog interval is set and timer armed, false
 *   otherwise.
 ******************************************************************************/
bool TD_WATCHDOG_Enable(bool enable, bool automatic)
{
	if (TD_WATCHDOG_Interval > 0) {

		// If not already activated and we want activation
		if (enable && !TD_Watchdog_Enabled) {

			TD_Watchdog_Enabled = true;
			// Force a wake-up every period to allow feeding watchdog in time
			//(before RTC overflow)
			if (!automatic) {
				TD_Watchdog_Timer = TD_SCHEDULER_Append(TD_WATCHDOG_Interval,
														0,
														0,
														0xFF,
														0,
														0);


			} else {
				TD_Watchdog_Timer = TD_SCHEDULER_Append(TD_WATCHDOG_Interval,
														0,
														0,
														0xFF,
														TD_WATCHDOG_Callback,
														0);
			}

			if (TD_Watchdog_Timer != 0xFF) {
				WDOG_Enable(true);

				// Feed
				TD_WATCHDOG_Feed();
				return true;
			}
		} else if (!enable && TD_Watchdog_Enabled) {

			// If not activated and activation wanted
			if (TD_Watchdog_Timer != 0xFF) {

				// If auto-stop timer
				TD_SCHEDULER_Remove(TD_Watchdog_Timer);
				TD_Watchdog_Timer = 0xFF;
			}
			WDOG_Enable(false);
			return true;
		} else if (enable && automatic && (TD_Watchdog_Timer == 0xFF)) {

			// If activate + auto + auto not activated, start auto
			TD_Watchdog_Timer = TD_SCHEDULER_Append(TD_WATCHDOG_Interval,
													0,
													0,
													0xFF,
													TD_WATCHDOG_Callback,
													0);
			return true;
		}	else if (enable && !automatic && TD_Watchdog_Timer != 0xFF) {

			// If activate + !auto + auto activated, stop auto
			TD_SCHEDULER_Remove(TD_Watchdog_Timer);
			TD_Watchdog_Timer = 0xFF;
			return true;
		}
	}
	return false;
}

/***************************************************************************//**
 * @brief
 *   Feed the watchdog
 *
 * @details
 *   This function must be called before the watchdog period expires to avoid
 *   triggering the watchdog and reset the CPU.
 ******************************************************************************/
void TD_WATCHDOG_Feed(void)
{
	WDOG_Feed();
}

/** @} */
/** @} (end addtogroup WATCHDOG) */
