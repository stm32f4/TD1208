/******************************************************************************
 * @file
 * @brief Simple button-interrupt application for the TDxxxx RF modules.
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
#include <td_core.h>
#include <td_sigfox.h>
#include <td_gpio.h>

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
 *************************   PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** Button event flag */
static bool ButtonPressedEvent = false;

/*******************************************************************************
 *************************   PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *   Button interrupt handler.
 ******************************************************************************/
static void ButtonInterrupt(uint32_t mask)
{
	ButtonPressedEvent = true;
}

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *   User setup function.
 ******************************************************************************/
void TD_USER_Setup(void)
{
	int type;
	IRQn_Type irq_parity;

	// Define the LED pin as an output in push-pull mode
	GPIO_PinModeSet(LED_PORT, LED_BIT, gpioModePushPull, 0);

	// Define the button pin as an input with an internal pull-up resistor
	GPIO_PinModeSet(BUTTON_PORT, BUTTON_BIT, gpioModeInputPull, 1);

	// Set up a user hook on button pin interrupt
	type = (BUTTON_MASK & TD_GPIO_ODD_MASK) ?
			TD_GPIO_USER_ODD : TD_GPIO_USER_EVEN;
	TD_GPIO_SetCallback(type, ButtonInterrupt, BUTTON_MASK);

	// Enable falling edge interrupts on button pin
	GPIO_IntConfig(RX_PORT, RX_BIT, false, true, true);

	// Clear and enable the corresponding interrupt in the CPU's Nested Vector
	// Interrupt Controller
	irq_parity =
			(BUTTON_MASK & TD_GPIO_ODD_MASK) ? GPIO_ODD_IRQn : GPIO_EVEN_IRQn;
	NVIC_ClearPendingIRQ(irq_parity);
	NVIC_EnableIRQ(irq_parity);
}

/***************************************************************************//**
 * @brief
 *   User loop function.
 ******************************************************************************/
void TD_USER_Loop(void)
{
	static uint8_t count = 1;
	uint8_t frame[3] = {0x00, 0x04, 0x00};

	// If RX module pin goes low
	if (ButtonPressedEvent) {

		// Turn on the LED
		GPIO_PinOutSet(LED_PORT, LED_BIT);

		// Send the single byte counter to the SIGFOX network
		frame[2] = count++;
		TD_SIGFOX_Send(frame, 3, 2);

		// Wait actively while the button pin is still low
		while (GPIO_PinInGet(BUTTON_PORT, BUTTON_BIT) == 0) {
			;
		}

		// Turn off the LED
		GPIO_PinOutClear(LED_PORT, LED_BIT);

		// Clear the button pressed event
		ButtonPressedEvent = false;
	}
}
