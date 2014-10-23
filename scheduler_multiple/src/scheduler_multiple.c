/******************************************************************************
 * @file
 * @brief Watchdog demonstration application for the TDxxxx RF modules.
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

#include <efm32.h>

#include <em_gpio.h>

#include <td_sigfox.h>
#include <td_core.h>
#include <td_uart.h>
#include <td_printf.h>
#include <td_watchdog.h>

/* This file declare all "dynamic" library data. It should be last included file
 * Standard size value can be override before including this file
 */
#define TD_SENSOR_USE_CODE 0
#define TD_GEOLOC_USE_CODE 0
#include <td_config.h>

/*******************************************************************************
 ******************************  VARIABLE  ************************************
 ******************************************************************************/
/** Time  */
uint32_t now;

/** Timer id */
uint8_t LedTimer = 0xFF;

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *  Timer callback
 *
 * @param[in] arg
 *  Argument passed by the Scheduler.
 *
 * @param[in] repetition
 *  Argument passed by the Scheduler.
 ******************************************************************************/
static void TimerFunction1(uint32_t arg, uint8_t repetition)
{
	// Print timer info and time
	now = TD_SCHEDULER_GetTime() >> 15;
	tfp_printf("Function 1 Timer %d Rep %d Time %d\r\n", arg, repetition,now);

	// When Timer 2 has only 27 repetitions left
	if (arg == 2 && repetition == 45) {

		// Remove the LED timer
		TD_SCHEDULER_Remove(LedTimer);

		// Switch off the LED
		GPIO_PinOutClear(TIM2_PORT, TIM2_BIT);
	}
}

/***************************************************************************//**
 * @brief
 *  LED blinking. This function is being called by the Scheduler.
 *
 * @param[in] arg
 *  Argument passed by Scheduler.
 *
 * @param[in] repetition
 *  Argument passed by Scheduler.
 ******************************************************************************/
static void LedBlink(uint32_t arg, uint8_t repetition)
{
	// Toggle the LED
	GPIO_PinOutToggle(TIM2_PORT, TIM2_BIT);
}

/***************************************************************************//**
 * @brief
 *   User setup function.
 ******************************************************************************/
void TD_USER_Setup(void)
{
	//Initialize the UART console
	init_printf(TD_UART_Init(9600, true, false),
		TD_UART_Putc,
		TD_UART_Start,
		TD_UART_Stop);

	// Initialize the LED
	GPIO_PinModeSet(TIM2_PORT, TIM2_BIT, gpioModePushPull, 0);

	// Initialize the timer with 10 repetitions and 3 s interval, Func1, Arg 1
	TD_SCHEDULER_Append(3, 0 , 0, 10, TimerFunction1, 1);

	// Init timer with 64 repetitions and 2s interval, Func1, Arg 2
	TD_SCHEDULER_Append(1, 0 , 0, 64, TimerFunction1, 2);

	// Initialize timer with infinite repetitions and 1s interval, Func2, Arg 3
	LedTimer = TD_SCHEDULER_AppendIrq(0, 8192 , 0, TD_SCHEDULER_INFINITE, LedBlink, 0);
}

/***************************************************************************//**
 * @brief
 *   User loop function.
 ******************************************************************************/
void TD_USER_Loop(void)
{
}
