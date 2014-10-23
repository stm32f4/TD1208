/***************************************************************************//**
 * @file
 * @brief Timer peripheral API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 2.0.1
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

#include <stdint.h>
#include <stdbool.h>

#include <em_cmu.h>
#include <em_timer.h>
#include <em_gpio.h>

#include "td_core.h"
#include "td_timer.h"
#include "td_gpio.h"

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
 **************************   PUBLIC VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup TIMER_GLOBAL_VARIABLES Global Variables
 * @{ */

/** Flag for timer enabled */
bool TD_TIMER_Enabled = false;

/** User tick count */
uint32_t TickUser = 0;

/** @} */

/*******************************************************************************
 *************************   PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup TIMER_LOCAL_VARIABLES Local Variables
 * @{ */

/** Select CC channel parameters */
static TIMER_InitCC_TypeDef timerCCInit = {
	.eventCtrl  = timerEventEveryEdge,
	.edge       = timerEdgeBoth,
	.prsSel     = timerPRSSELCh0,
	.cufoa      = timerOutputActionNone,
	.cofoa      = timerOutputActionNone,
	.cmoa       = timerOutputActionToggle,
	.mode       = timerCCModePWM,
	.filter     = false,
	.prsInput   = false,
	.coist      = false,
	.outInvert  = false,
};

/** Select timer parameters */
static TIMER_Init_TypeDef timerInit = {
	.enable     = true,
	.debugRun   = false,
	.prescale   = timerPrescale1,
	.clkSel     = timerClkSelHFPerClk,
	.fallAction = timerInputActionNone,
	.riseAction = timerInputActionNone,
	.mode       = timerModeUp,
	.dmaClrAct  = false,
	.quadModeX4 = false,
	.oneShot    = false,
	.sync       = false,
};

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TIMER_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Interrupt Service Routine for TIMER1 Interrupt Line.
 ******************************************************************************/
void TIMER1_IRQHandler(void)
{
	// Clear flag for TIMER1 overflow interrupt
	TIMER_IntClear(TIMER1, TIMER_IF_OF);
	TickUser++;
}

/***************************************************************************//**
 * @brief
 *  Start the TIMER1 to generate a 50% duty cycle output.
 *
 * @param[in] frequency
 *  The output frequency in Hz.
 ******************************************************************************/
void TD_TIMER_Start(uint32_t frequency)
{
	uint32_t top;

	top = CMU_ClockFreqGet(cmuClock_TIMER1);
	top = top / frequency;

	// Enable clock for TIMER1 module
	CMU_ClockEnable(cmuClock_TIMER1, true);

	// Configure CC channel 0
	TIMER_InitCC(TIMER1, 0, &timerCCInit);

	// Route CC0 to location 0 (PC13) and enable pin
	//TIMER1->ROUTE |= (TIMER_ROUTE_CC0PEN | TIMER_ROUTE_LOCATION_LOC0);

	// Set Top Value
	TIMER_TopSet(TIMER1, top);

	// Set compare value starting at 0 - it will be incremented in the interrupt handler
	TIMER_CompareBufSet(TIMER1, 0, top >> 1);

	// Configure timer
	TIMER_Init(TIMER1, &timerInit);

	// Enable overflow interrupt
	TIMER_IntEnable(TIMER1, TIMER_IF_OF);

	// Disable interrupts
	//TIMER_IntDisable(TIMER1, TIMER_IF_OF);

	// Enable TIMER1 interrupt vector in NVIC
	NVIC_EnableIRQ(TIMER1_IRQn);

	// Enable timer
	TIMER_Enable(TIMER1, true);
	TD_TIMER_Enabled = true;
}

/***************************************************************************//**
 * @brief
 *   Stop the TIMER1.
 ******************************************************************************/
void TD_TIMER_Stop(void)
{
	// Disable timer
	TIMER_Enable(TIMER1, false);
	TD_TIMER_Enabled = false;

	// Unroute CC0 to location 0 (PC13) and disable pin
	TIMER1->ROUTE &= ~(TIMER_ROUTE_CC0PEN | TIMER_ROUTE_LOCATION_LOC0);
	//TIMER_IntDisable(TIMER1, TIMER_IF_OF);
}

/** @} */

/** @} (end addtogroup TIMER) */
