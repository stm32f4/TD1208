/******************************************************************************
 * @file
 * @brief Simple button-polling application for the TDxxxx RF modules.
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

#include <efm32.h>
#include <em_gpio.h>
#include <td_sigfox.h>
#include <td_core.h>

/* This file declare all "dynamic" library data. It should be last included file
 * Standard size value can be override before including this file
 */
#define TD_SENSOR_USE_CODE 0
#define TD_GEOLOC_USE_CODE 0
#include <td_config.h>

/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

#define LED_PORT	TIM2_PORT			/**< LED port */
#define LED_BIT		TIM2_BIT			/**< LED bit */
#define BUTTON_PORT	RX_PORT				/**< Button port */
#define BUTTON_BIT	RX_BIT				/**< Button bit */
#define BUTTON_MASK	RX_MASK				/**< Button mask */

/*******************************************************************************
 ******************************  CONSTANTS  ************************************
 ******************************************************************************/

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *   User setup function.
 ******************************************************************************/
void TD_USER_Setup(void)
{
	uint8_t count = 0;

	// Define the LED pin as an output in push-pull mode
	GPIO_PinModeSet(LED_PORT, LED_PORT, gpioModePushPull, 0);

	// Define the button pin as an input with an internal pull-up resistor
	GPIO_PinModeSet(BUTTON_PORT, BUTTON_BIT, gpioModeInputPull, 1);

	// Forever
	while (true) {

		// If button pin goes low
		if (GPIO_PinInGet(BUTTON_PORT, BUTTON_BIT) == 0) {

			// Turn on the LED
			GPIO_PinOutSet(LED_PORT, LED_BIT);

			// send the single byte counter to the SIGFOX network
			TD_SIGFOX_Send(&count, 1, 2);
			count++;

			// Wait actively while the button pin is still low
			while (GPIO_PinInGet(BUTTON_PORT, BUTTON_BIT) == 0) {
				;
			}

			// Turn off the LED
			GPIO_PinOutClear(LED_PORT, LED_BIT);
		}
	}

	// Never reached
}

/***************************************************************************//**
 * @brief
 *   User loop function.
 ******************************************************************************/
void TD_USER_Loop(void)
{
}
