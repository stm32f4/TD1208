/******************************************************************************
 * @file
 * @brief TD LAN TX example for TDxxxx RF modules.
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
#include <efm32.h>
#include <em_gpio.h>
#include <td_core.h>
#include <td_sigfox.h>
#include <td_rtc.h>
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

#define LED_PORT		TIM2_PORT			/**< LED port */
#define LED_BIT			TIM2_BIT			/**< LED bit */

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
	int i;
	uint8_t message[12];

	// Define the LED pin as an output in push-pull mode
	GPIO_PinModeSet(LED_PORT, LED_BIT, gpioModePushPull, 0);

   //Build hex message
   for(i=0;i<12;i++) {
	   message[i] = i;
   }

   //Send hex message with 2 repetitions (recommended by Sigfox)
   GPIO_PinOutSet(LED_PORT,LED_BIT);
   TD_SIGFOX_Send(message,12,2);
   GPIO_PinOutClear(LED_PORT,LED_BIT);

   //Wait a bit
   TD_RTC_Delay(32768*2);

   //Send string message without repetitions (not recommended)
   GPIO_PinOutSet(LED_PORT,LED_BIT);
   TD_SIGFOX_Send((uint8_t *)"abcdefghijkl",12,0);
   GPIO_PinOutClear(LED_PORT,LED_BIT);
}

/***************************************************************************//**
 * @brief
 *   User loop function.
 ******************************************************************************/
void TD_USER_Loop(void)
{
	// void
}
